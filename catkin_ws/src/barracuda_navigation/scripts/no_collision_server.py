#!/usr/bin/env python3

import rospy
from barracuda_msgs.srv import CheckCollision, CheckCollisionResponse


def handle_check_collision(req):
    # Always report no collision for testing/demo
    return CheckCollisionResponse(collision=False)


def main():
    rospy.init_node("no_collision_server", anonymous=True)
    srv_name = rospy.get_param("~service", "check_collision")
    rospy.Service(srv_name, CheckCollision, handle_check_collision)
    rospy.loginfo("'%s' service ready (always returns collision=False)", srv_name)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

