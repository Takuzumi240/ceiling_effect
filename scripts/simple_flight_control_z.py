#!/usr/bin/env python

import sys
import rospy
import math
from aerial_robot_msgs.msg import FlightNav

class ceil_approach_nav():
    def __init__(self):

        self.t = 0
        self.dt = 0.001
        self.z = rospy.get_param("~z", 0.5)
        self.z_diff = rospy.get_param("~z_diff", 0.2)
        self.curve_factor = rospy.get_param("~curve_factor", 0.01)

        self.nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.desire_nav = FlightNav()


    def tar_track_sigmoid(self):
        sigmoid = 2*((math.exp(self.curve_factor*self.t)/(math.exp(self.curve_factor*self.t) + 1)) - 0.5)
        tar_z = self.z + self.z_diff*sigmoid

        self.desire_nav.pos_z_nav_mode = 2
        self.desire_nav.target_pos_z = tar_z

        self.nav_control_pub.publish(self.desire_nav)
        self.t = self.t + self.dt
        print(tar_z)

        return  self.z + self.z_diff - tar_z



if __name__=="__main__":

    rospy.init_node("simple_flight_control_z")
    ceil_approacher = ceil_approach_nav()

    while True:
        try:
            diff = ceil_approacher.tar_track_sigmoid()

            if diff < 0.001:
                break

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print (e)

