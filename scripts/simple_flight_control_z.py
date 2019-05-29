#!/usr/bin/env python
import sys
import rospy
from aerial_robot_msgs.msg import FlightNav

rospy.init_node("simple_flight_control_z")
nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)

rospy.sleep(1)

z = rospy.get_param("~z", 1.2)
desire_nav = FlightNav()

desire_nav.pos_z_nav_mode = 2
desire_nav.target_pos_z = z

nav_control_pub.publish(desire_nav)




