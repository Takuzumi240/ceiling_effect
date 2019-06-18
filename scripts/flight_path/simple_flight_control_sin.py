#!/usr/bin/env python

import sys
import rospy
import math
from aerial_robot_msgs.msg import FlightNav

class ceil_approach_nav():
    def __init__(self):

        self.t = 0
        self.dt = 0.01
        self.max = rospy.get_param("~max", 0.8)
        self.min = rospy.get_param("~min", 0.5)
        self.mid = (self.max + self.min)/2.0
        self.duration = rospy.get_param("~duration", 60)

        self.nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1000)
        self.desire_nav = FlightNav()

    def tar_track_sin(self):

        while True:
            tar_z = self.mid - (self.max - self.mid)*math.cos(2*math.pi*self.t/self.duration)

            self.desire_nav.pos_z_nav_mode = 2
            self.desire_nav.target_pos_z = tar_z

            self.nav_control_pub.publish(self.desire_nav)
            rospy.sleep(self.dt)

            self.t = self.t + self.dt

            if (self.t > self.duration*3):
                break



if __name__=="__main__":

    rospy.init_node("simple_flight_control_sin")

    nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
    desire_nav = FlightNav()
    desire_nav.pos_z_nav_mode = 2
    desire_nav.target_pos_z = 0.5
    nav_control_pub.publish(desire_nav)
    rospy.sleep(5.0)

    ceil_approacher = ceil_approach_nav()
    ceil_approacher.tar_track_sin()

