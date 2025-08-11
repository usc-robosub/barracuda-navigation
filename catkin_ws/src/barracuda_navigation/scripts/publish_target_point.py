#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from barracuda_msgs.srv import PlanPath, PlanPathRequest


def publish_robot_pose(pub, frame_id, x, y, z):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    pub.publish(pose)


def main():
    rospy.init_node("publish_target_point", anonymous=True)

    frame_id = rospy.get_param("~frame_id", "map")
    goal_x = rospy.get_param("~goal_x", 5.0)
    goal_y = rospy.get_param("~goal_y", 3.0)
    goal_z = rospy.get_param("~goal_z", 1.0)
    plan_service_name = rospy.get_param("~plan_service", "plan_path")

    publish_pose = rospy.get_param("~publish_pose", True)
    pose_topic = rospy.get_param("~pose_topic", "robot_pose")
    start_x = rospy.get_param("~start_x", 0.0)
    start_y = rospy.get_param("~start_y", 0.0)
    start_z = rospy.get_param("~start_z", 0.5)

    if publish_pose:
        pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1, latch=True)
        # Give publisher time to connect
        rospy.sleep(0.2)
        publish_robot_pose(pose_pub, frame_id, start_x, start_y, start_z)
        rospy.loginfo("Published start pose on '%s' in frame '%s'", pose_topic, frame_id)

    rospy.loginfo("Waiting for plan service '%s'...", plan_service_name)
    rospy.wait_for_service(plan_service_name)
    plan_srv = rospy.ServiceProxy(plan_service_name, PlanPath)

    req = PlanPathRequest()
    req.goal_pose.header.frame_id = frame_id
    req.goal_pose.header.stamp = rospy.Time.now()
    req.goal_pose.pose.position.x = float(goal_x)
    req.goal_pose.pose.position.y = float(goal_y)
    req.goal_pose.pose.position.z = float(goal_z)
    req.goal_pose.pose.orientation.w = 1.0

    try:
        rospy.loginfo(
            "Requesting plan to (%.2f, %.2f, %.2f) in '%s'",
            goal_x,
            goal_y,
            goal_z,
            frame_id,
        )
        resp = plan_srv(req)
        n = len(resp.waypoints.points)
        if n > 0:
            rospy.loginfo("Planner returned %d waypoints on 'rrt_waypoints'", n)
        else:
            rospy.logwarn("Planner returned no waypoints")
    except rospy.ServiceException as e:
        rospy.logerr("Plan service call failed: %s", str(e))

    # Keep node alive briefly so latched topics are available
    rospy.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

