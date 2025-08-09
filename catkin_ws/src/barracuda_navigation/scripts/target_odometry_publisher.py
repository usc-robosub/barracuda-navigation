#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def normalize_angle(angle):
    a = (angle + np.pi) % (2 * np.pi) - np.pi
    return a


class LQRTargetOdometry:
    def __init__(self):
        rospy.init_node('target_odometry_publisher', anonymous=True)

        # Parameters
        self.path_topic = rospy.get_param('~path_topic', 'interpolated_path')
        self.odom_topic = rospy.get_param('~odom_topic', 'odometry/filtered/global')
        self.target_topic = rospy.get_param('~target_topic', 'target_odometry')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.desired_speed = rospy.get_param('~desired_speed', 0.8)  # m/s constant translational speed
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 1.5)  # m
        self.max_lookahead_dist = rospy.get_param('~max_lookahead_dist', 3.0)  # m
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 0.8)  # rad/s
        self.z_gain = rospy.get_param('~z_gain', 0.8)  # simple P for depth tracking
        self.max_climb_rate = rospy.get_param('~max_climb_rate', 0.5)  # m/s
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)

        # Subscriptions and publishers
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        self.target_pub = rospy.Publisher(self.target_topic, Odometry, queue_size=1)

        # Path cache
        self.path_frame = self.frame_id
        self.path_positions = []  # list of np.array([x,y,z])
        self.path_yaws = []       # list of float yaw (rad)
        self.cum_s = []           # cumulative distance along path
        self.last_progress_idx = 0

        # Robot state
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_yaw = 0.0

        # Timer loop
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("Target Odometry node ready: path='%s', odom='%s', target='%s'",
                      self.path_topic, self.odom_topic, self.target_topic)

    def path_cb(self, msg: Path):
        if not msg.poses:
            return
        self.path_frame = msg.header.frame_id or self.frame_id
        n = len(msg.poses)
        positions = np.zeros((n, 3), dtype=float)
        yaws = np.zeros(n, dtype=float)

        for i, ps in enumerate(msg.poses):
            p = ps.pose.position
            positions[i] = np.array([p.x, p.y, p.z])
            q = ps.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            yaws[i] = yaw

        # Compute cumulative distances along the path
        cum_s = np.zeros(n, dtype=float)
        for i in range(1, n):
            ds = np.linalg.norm(positions[i] - positions[i - 1])
            cum_s[i] = cum_s[i - 1] + ds

        self.path_positions = [positions[i] for i in range(n)]
        self.path_yaws = [yaws[i] for i in range(n)]
        self.cum_s = [cum_s[i] for i in range(n)]
        self.last_progress_idx = 0

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pos = np.array([p.x, p.y, p.z], dtype=float)
        self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def find_progress_index(self):
        if not self.path_positions:
            return None
        # Search in a window ahead of last_progress_idx to avoid oscillations
        start = max(0, self.last_progress_idx - 5)
        end = len(self.path_positions)
        dists = [np.linalg.norm(self.robot_pos[:2] - np.array(self.path_positions[i][:2]))
                 for i in range(start, end)]
        local_idx = int(np.argmin(dists))
        idx = start + local_idx
        # Enforce monotonic progress
        idx = max(idx, self.last_progress_idx)
        self.last_progress_idx = idx
        return idx

    def choose_target_index(self, idx_near):
        if idx_near is None:
            return None
        s0 = self.cum_s[idx_near]
        desired_la = np.clip(self.lookahead_dist, 0.0, self.max_lookahead_dist)
        target_idx = idx_near
        for i in range(idx_near, len(self.cum_s)):
            if self.cum_s[i] - s0 >= desired_la:
                target_idx = i
                break
        return target_idx

    def timer_cb(self, _):
        if not self.path_positions:
            return

        v = float(self.desired_speed)
        idx_near = self.find_progress_index()
        if idx_near is None:
            return
        idx_tgt = self.choose_target_index(idx_near)

        # Target pose at lookahead index
        if idx_tgt is None:
            idx_tgt = idx_near
        p_tgt = self.path_positions[idx_tgt]
        yaw_tgt = self.path_yaws[idx_tgt]

        # Desired translational velocity along the 3D path tangent (constant speed)
        if idx_tgt + 1 < len(self.path_positions):
            p_next = self.path_positions[idx_tgt + 1]
            tangent = np.array(p_next) - np.array(p_tgt)
        else:
            # At end of path, use backward tangent if possible
            if idx_tgt > 0:
                p_prev = self.path_positions[idx_tgt - 1]
                tangent = np.array(p_tgt) - np.array(p_prev)
            else:
                tangent = np.array([np.cos(yaw_tgt), np.sin(yaw_tgt), 0.0])
        norm = np.linalg.norm(tangent)
        if norm < 1e-6:
            vx, vy, vz = 0.0, 0.0, 0.0
        else:
            t_hat = tangent / norm
            vx, vy, vz = (v * float(t_hat[0]), v * float(t_hat[1]), v * float(t_hat[2]))

        # Publish target odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.path_frame or self.frame_id
        odom.child_frame_id = 'lqr_target'

        odom.pose.pose.position.x = float(p_tgt[0])
        odom.pose.pose.position.y = float(p_tgt[1])
        odom.pose.pose.position.z = float(p_tgt[2])
        q = quaternion_from_euler(0.0, 0.0, yaw_tgt)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.target_pub.publish(odom)


def main():
    try:
        LQRTargetOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
