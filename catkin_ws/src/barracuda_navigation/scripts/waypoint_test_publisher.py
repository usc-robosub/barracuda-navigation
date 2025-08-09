#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from barracuda_msgs.msg import Waypoints


def build_sample_waypoints(frame_id):
    msg = Waypoints()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    # Simple S-curve in XY plane
    pts = [
        (0.0, 0.0, 0.0),
        (1.0, 0.5, 0.0),
        (2.0, 1.0, 0.0),
        (3.0, 1.0, 0.0),
        (4.0, 0.5, 0.0),
        (5.0, 0.0, 0.0),
    ]
    msg.points = [Point(x=p[0], y=p[1], z=p[2]) for p in pts]
    return msg


def main():
    rospy.init_node("waypoint_test_publisher", anonymous=True)
    frame_id = rospy.get_param("~frame_id", "map")
    topic = rospy.get_param("~topic", "rrt_waypoints")
    repeat = rospy.get_param("~repeat", False)
    rate_hz = rospy.get_param("~rate", 0.5)

    pub = rospy.Publisher(topic, Waypoints, queue_size=1, latch=True)
    rospy.sleep(0.2)  # allow publisher to register

    msg = build_sample_waypoints(frame_id)
    rospy.loginfo("Publishing sample waypoints on %s (frame: %s)", topic, frame_id)
    pub.publish(msg)

    if not repeat:
        rospy.loginfo("Published once (latched). Set ~repeat:=true to republish.")
        return

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

