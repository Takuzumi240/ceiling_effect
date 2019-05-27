#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf


from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import TransformStamped
from jsk_recognition_msgs.msg import ModelCoefficientsArray
from ceil_effect_control.msg import distance_to_ceilingwall
from spinal.msg import FourAxisCommand
from nav_msgs.msg import Odometry

class ceil_effect_simulator():
    def __init__(self):

        self.rotor_num = 4
        self.rotor_diameter = 0.355
        self.offset = 0.02 #0.053 #between cog and rotor 
        self.a_without_ceil = 0.0045800748
        self.b_without_ceil = -0.2564642520
        self.alfa = 0.035
        self.kappa = 1.4
        self.ceil_dist_inf = 3 #nondimensional

        self.throttle_ceil_pub = rospy.Publisher("/aerial_robot_control_four_axis",FourAxisCommand, queue_size=10)

        self.ceil_dist_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.real_plane_dist)
        self.throttle_sub = rospy.Subscriber("/aerial_robot_control_four_axis_temp",FourAxisCommand, self.thrust_conversion_to_ceil)

    #TO DO sub
    #def ceil_detect(self):

        self.real_ceil_z =1.5

    def real_plane_dist(self, odom):

        self.real_ceil_dist = self.real_ceil_z - odom.pose.pose.position.z -self.offset

    def thrust_conversion_to_ceil(self, cmd_temp):

        self.thrust_conversion_to_pwm(cmd_temp.base_throttle)
        self.pwm_to_thrust_ceil(self.pulse_length, self.real_ceil_dist)
        cmd_temp.base_throttle = self.thrust_ceil
        self.throttle_ceil_pub.publish(cmd_temp)

    def thrust_conversion_to_pwm(self,thrust_without_ceil):

        self.pulse_length = []
        for i in range(0, self.rotor_num):
            mu_sqrt = math.sqrt(self.b_without_ceil*self.b_without_ceil + 4*self.a_without_ceil*thrust_without_ceil[i])
            mu = (-self.b_without_ceil + mu_sqrt)/(2 * self.a_without_ceil)
            self.pulse_length.append(mu*20)

    def pwm_to_thrust_ceil(self, pulse_length, real_ceil_dist):

        duty_ratio = map(lambda mu: mu/20, pulse_length)
        self.thrust_ceil_coefficients(real_ceil_dist)
        self.thrust_ceil = []

        for i in range (0, self.rotor_num):
            thrust = self.a_ceil*duty_ratio[i]*duty_ratio[i]+self.b_ceil*duty_ratio[i]
            self.thrust_ceil.append(thrust)
        return self.thrust_ceil

    def thrust_ceil_coefficients(self, real_ceil_dist):
        ceil_func = 1 + self.alfa / math.pow((real_ceil_dist / self.rotor_diameter), self.kappa)
        self.a_ceil = 0.0045800748*ceil_func
        self.b_ceil = -0.2564642520*ceil_func

if __name__ == "__main__":

    rospy.init_node("hydrusx_ceil_effect_simulation")

    while not rospy.is_shutdown():
        try:
            ceil_sim = ceil_effect_simulator()

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        rospy.spin()

