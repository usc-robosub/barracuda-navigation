#!/usr/bin/env python

import rospy
import numpy as np
from scipy.interpolate import CubicSpline
from barracuda_msgs.msg import Waypoints
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_matrix
import tf2_ros
import tf2_geometry_msgs

class WaypointInterpolator:
    def __init__(self):
        rospy.init_node('waypoint_interpolator', anonymous=True)
        
        # Parameters
        self.velocity = rospy.get_param('~velocity', 1.0)  # m/s (used for time parameterization)
        self.max_speed = rospy.get_param('~max_speed', 2.0)  # m/s (maximum allowed speed)
        self.base_dt = rospy.get_param('~base_dt', 0.1)  # controller time step (seconds)
        self.output_dt = rospy.get_param('~output_dt', 0.02)  # high-resolution sampling interval (seconds)
        self.min_waypoints = rospy.get_param('~min_waypoints', 2)  # minimum waypoints for interpolation
        
        # Publishers and Subscribers
        self.waypoints_sub = rospy.Subscriber('/rrt_waypoints', Waypoints, self.waypoints_callback)
        self.path_pub = rospy.Publisher('/interpolated_path', Path, queue_size=1)
        
        # TF2 for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("Waypoint Interpolator Node initialized")
        rospy.loginfo(f"Velocity: {self.velocity} m/s, Max Speed: {self.max_speed} m/s")
        rospy.loginfo(f"Base dt: {self.base_dt} s, Output dt: {self.output_dt} s")
        
    def waypoints_callback(self, msg):
        """Callback for RRT waypoints"""
        if len(msg.points) < self.min_waypoints:
            rospy.logwarn(f"Not enough waypoints ({len(msg.points)} < {self.min_waypoints})")
            return
            
        # Extract points
        points = np.array([[p.x, p.y, p.z] for p in msg.points])
        
        # Handle different cases based on number of waypoints
        if len(points) == 2:
            # Linear interpolation for 2 waypoints
            spline, t_max = self.create_linear_interpolation(points)
        else:
            # Cubic spline for 3+ waypoints
            spline, t_params = self.create_parametric_spline(points)
            t_max = t_params[-1]
        
        # Estimate maximum speed and apply global scaling if needed
        max_estimated_speed = self.estimate_max_speed(points)
        time_scale = 1.0
        if max_estimated_speed > self.max_speed:
            time_scale = max_estimated_speed / self.max_speed
            t_max *= time_scale
            rospy.loginfo(f"Speed scaling applied: {time_scale:.2f}x (max estimated: {max_estimated_speed:.2f} m/s)")
        
        # Generate discrete targets with constant output_dt
        discrete_targets = self.generate_discrete_targets(spline, t_max, time_scale)
        
        # Publish as Path message
        path_msg = self.create_path_message(discrete_targets, msg.header)
        self.path_pub.publish(path_msg)
        
        interpolation_type = "linear" if len(points) == 2 else "cubic spline"
        rospy.loginfo(f"Published {interpolation_type} interpolated path with {len(path_msg.poses)} poses")
    
    def estimate_max_speed(self, points):
        """Estimate maximum speed based on waypoint distances and base_dt"""
        max_speed = 0.0
        
        for i in range(len(points) - 1):
            # Calculate distance between consecutive waypoints
            distance = np.linalg.norm(points[i + 1] - points[i])
            # Estimate speed if we traverse this distance in base_dt time
            estimated_speed = distance / self.base_dt
            max_speed = max(max_speed, estimated_speed)
        
        rospy.loginfo(f"Estimated max speed: {max_speed:.2f} m/s (limit: {self.max_speed:.2f} m/s)")
        return max_speed
    
    def create_linear_interpolation(self, points):
        """Create linear interpolation for 2 waypoints (straight line)"""
        start_point = points[0]
        end_point = points[1]
        
        # Calculate total distance and time
        total_distance = np.linalg.norm(end_point - start_point)
        
        # Handle edge case where start and end are the same point
        if total_distance < 1e-6:
            rospy.logwarn("Start and end points are identical, creating stationary path")
            total_time = 0.1  # Small non-zero time
        else:
            total_time = total_distance / self.velocity
        
        # Create linear interpolation function
        def linear_spline(t):
            # Clamp t to valid range
            t = np.clip(t, 0, total_time)
            # Linear interpolation: start + (end - start) * (t / total_time)
            return start_point + (end_point - start_point) * (t / total_time)
        
        # Create constant derivative function (direction vector)
        if total_distance < 1e-6:
            # For stationary path, use zero velocity
            direction = np.zeros(3)
        else:
            direction = (end_point - start_point) / total_distance
            
        def linear_derivative(t):
            return direction * self.velocity
        
        linear_spline.derivative = linear_derivative
        
        rospy.loginfo(f"Created linear interpolation: distance={total_distance:.2f}m, time={total_time:.2f}s")
        
        return linear_spline, total_time
        
    def create_parametric_spline(self, points):
        """Create parametric cubic spline with arc-length parameterization"""
        n_points = len(points)
        
        # Calculate distances between consecutive points
        distances = np.zeros(n_points)
        for i in range(1, n_points):
            distances[i] = np.linalg.norm(points[i] - points[i-1])
            
        # Create time parameters proportional to distance (for constant velocity)
        t_params = np.zeros(n_points)
        for i in range(1, n_points):
            t_params[i] = t_params[i-1] + distances[i] / self.velocity
            
        # Create cubic splines for x, y, z
        spline_x = CubicSpline(t_params, points[:, 0])
        spline_y = CubicSpline(t_params, points[:, 1])
        spline_z = CubicSpline(t_params, points[:, 2])
        
        # Combine into single parametric function
        def spline(t):
            return np.array([spline_x(t), spline_y(t), spline_z(t)])
        
        # Add derivative function for tangent vectors
        def spline_derivative(t):
            return np.array([spline_x(t, 1), spline_y(t, 1), spline_z(t, 1)])
        
        spline.derivative = spline_derivative
        
        return spline, t_params
        
    def generate_discrete_targets(self, spline, t_max, time_scale):
        """Generate discrete targets with constant output_dt"""
        targets = []
        
        # Generate time stamps at constant output_dt intervals
        t_values = np.arange(0, t_max + self.output_dt, self.output_dt)
        
        for t in t_values:
            # Apply time scaling to the spline sampling
            scaled_t = t / time_scale if time_scale > 1.0 else t
            
            # Get position and velocity at current time
            position = spline(scaled_t)
            velocity_vector = spline.derivative(scaled_t)
            
            # Apply time scaling to velocity (if path was slowed down, velocity is reduced)
            if time_scale > 1.0:
                velocity_vector = velocity_vector / time_scale
            
            speed = np.linalg.norm(velocity_vector)
            
            # Normalize tangent for orientation (handle zero velocity case)
            if speed > 1e-6:
                tangent_norm = velocity_vector / speed
            else:
                # Use previous tangent or default forward direction
                if targets:
                    tangent_norm = targets[-1]['tangent']
                else:
                    tangent_norm = np.array([1.0, 0.0, 0.0])
            
            # Calculate quaternion from tangent vector
            quaternion = self.tangent_to_quaternion(tangent_norm)
            
            targets.append({
                'position': position,
                'orientation': quaternion,
                'speed': speed,
                'tangent': tangent_norm,
                'time': t
            })
        
        # Calculate speed statistics
        speeds = [target['speed'] for target in targets]
        if speeds:
            max_actual_speed = max(speeds)
            avg_speed = sum(speeds) / len(speeds)
            rospy.loginfo(f"Generated {len(targets)} targets with constant output_dt ({self.output_dt:.3f} s)")
            rospy.loginfo(f"Speed stats - Max: {max_actual_speed:.2f} m/s, Avg: {avg_speed:.2f} m/s, Limit: {self.max_speed} m/s")
        
        return targets
        
    def tangent_to_quaternion(self, tangent):
        """Convert tangent vector to quaternion orientation"""
        # Normalize tangent
        forward = tangent / np.linalg.norm(tangent)
        
        # Choose up vector (avoid parallel with forward)
        world_up = np.array([0, 0, 1])
        if abs(np.dot(forward, world_up)) > 0.99:
            world_up = np.array([0, 1, 0])
            
        # Calculate right and up vectors
        right = np.cross(forward, world_up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)
        
        # Create rotation matrix
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0] = forward  # x-axis points forward
        rotation_matrix[0:3, 1] = -right   # y-axis points left
        rotation_matrix[0:3, 2] = up       # z-axis points up
        
        # Convert to quaternion
        quaternion = quaternion_from_matrix(rotation_matrix)
        
        return quaternion
        
    def create_path_message(self, targets, header):
        """Create Path message from discrete targets"""
        path = Path()
        path.header = header
        path.header.stamp = rospy.Time.now()
        
        for target in targets:
            pose = PoseStamped()
            pose.header = header
            pose.header.stamp = rospy.Time.now() + rospy.Duration(target['time'])
            
            # Set position
            pose.pose.position.x = target['position'][0]
            pose.pose.position.y = target['position'][1]
            pose.pose.position.z = target['position'][2]
            
            # Set orientation
            q = target['orientation']
            pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            path.poses.append(pose)
            
        return path
        
def main():
    try:
        interpolator = WaypointInterpolator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
if __name__ == '__main__':
    main()
