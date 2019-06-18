#!/usr/bin/env python
import sys
import rospy
import math
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav

class ceil_approach_nav():
    def __init__(self):

        self.t = 0
        self.dt = 0.001
        self.duration = rospy.get_param("~duration", 10)
        self.z = rospy.get_param("~z", 0.5)
        self.z_diff = rospy.get_param("~z_diff", 0.05)

        self.num = 6
        self.nav_control_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.desire_nav = FlightNav()


    def tar_track_step_up(self):
        i = 0
        print(1)

        while True:

            tar_z = self.z + self.z_diff*i
            self.desire_nav.pos_z_nav_mode = 2
            self.desire_nav.target_pos_z = tar_z
            self.nav_control_pub.publish(self.desire_nav)
            rospy.sleep(self.duration)
            print(1, tar_z)
            if (i == self.num):
                self.max_z = tar_z
                rospy.sleep(self.duration)
                break

            i = i + 1



    def tar_track_step_down(self):
        i = 1

        while True:

            tar_z = self.max_z - self.z_diff*i
            self.desire_nav.pos_z_nav_mode = 2
            self.desire_nav.target_pos_z = tar_z
            self.nav_control_pub.publish(self.desire_nav)
            print(2, tar_z)
            if (i ==self.num):
                break

            i = i + 1

            rospy.sleep(self.duration)




if __name__=="__main__":

    rospy.init_node("simple_flight_control_step")
    ceil_approacher = ceil_approach_nav()

    ceil_approacher.tar_track_step_up()
    ceil_approacher.tar_track_step_down()

