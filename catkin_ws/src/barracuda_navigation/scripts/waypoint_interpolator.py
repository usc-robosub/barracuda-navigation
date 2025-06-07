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
        self.velocity = rospy.get_param('~velocity', 1.0)  # m/s
        self.dt = rospy.get_param('~dt', 0.1)  # seconds between discrete targets
        self.min_waypoints = rospy.get_param('~min_waypoints', 3)  # minimum waypoints for spline
        
        # Publishers and Subscribers
        self.waypoints_sub = rospy.Subscriber('/rrt_waypoints', Waypoints, self.waypoints_callback)
        self.path_pub = rospy.Publisher('/interpolated_path', Path, queue_size=1)
        
        # TF2 for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("Waypoint Interpolator Node initialized")
        rospy.loginfo(f"Velocity: {self.velocity} m/s, dt: {self.dt} s")
        
    def waypoints_callback(self, msg):
        """Callback for RRT waypoints"""
        if len(msg.points) < self.min_waypoints:
            rospy.logwarn(f"Not enough waypoints ({len(msg.points)} < {self.min_waypoints})")
            return
            
        # Extract points
        points = np.array([[p.x, p.y, p.z] for p in msg.points])
        
        # Create parametric spline
        spline, t_params = self.create_parametric_spline(points)
        
        # Generate discrete targets
        discrete_targets = self.generate_discrete_targets(spline, t_params[-1])
        
        # Publish as Path message
        path_msg = self.create_path_message(discrete_targets, msg.header)
        self.path_pub.publish(path_msg)
        
        rospy.loginfo(f"Published interpolated path with {len(path_msg.poses)} poses")
        
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
        
    def generate_discrete_targets(self, spline, t_max):
        """Generate discrete targets at constant dt intervals"""
        # Generate time stamps
        t_values = np.arange(0, t_max + self.dt, self.dt)
        
        targets = []
        for t in t_values:
            # Get position
            position = spline(t)
            
            # Get tangent vector for orientation
            tangent = spline.derivative(t)
            
            # Normalize tangent
            tangent_norm = tangent / np.linalg.norm(tangent)
            
            # Calculate quaternion from tangent vector
            quaternion = self.tangent_to_quaternion(tangent_norm)
            
            targets.append({
                'position': position,
                'orientation': quaternion,
                'time': t
            })
            
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
