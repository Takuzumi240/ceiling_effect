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
from spinal.msg import PwmInfo
from spinal.msg import MotorInfo
from nav_msgs.msg import Odometry


class ceil_effect_simulator():
    def __init__(self):

        self.rotor_num = 4
        self.rotor_diameter = 0.355
        self.offset = 0.02 #0.053 #between cog and rotor 
        self.a_without_ceil = 0.0045800748
        self.b_without_ceil = -0.2564642520
        self.a = self.a_without_ceil
        self.b = self.b_without_ceil

        self.alfa = 0.048#Best:0.048#experiment:0.035
        self.kappa = 1.5#Best:1.5#experiment:1.6
        self.ceil_dist_inf = 3 #nondimensional

        self.CEIL_EFFECT_CONTROL_FLAG = rospy.get_param('~controller', 1)

        self.throttle_ceil_pub = rospy.Publisher("/aerial_robot_control_four_axis",FourAxisCommand, queue_size=10)
        self.ceil_dist_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.real_plane_dist)
        self.throttle_sub = rospy.Subscriber("/aerial_robot_control_four_axis_temp",FourAxisCommand, self.thrust_conversion_to_ceil)
        self.ceil_coefficient_sub =rospy.Subscriber("/motor_info", PwmInfo, self.ceil_effect_control)

        self.real_ceil_z =1.0501

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
            mu_sqrt = math.sqrt(self.b*self.b + 4*self.a*thrust_without_ceil[i])
            mu = (-self.b + mu_sqrt)/(2 * self.a)
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
        self.a_ceil = self.a_without_ceil*ceil_func
        self.b_ceil = self.b_without_ceil*ceil_func

    def ceil_effect_control(self, msg):
        coef = msg.motor_info[0]

        if (self.CEIL_EFFECT_CONTROL_FLAG):
            self.a = coef.polynominal[2]/10
            self.b = coef.polynominal[1]/10


        else:
            self.a = self.a_without_ceil
            self.b = self.b_without_ceil

if __name__ == "__main__":

    rospy.init_node("ceil_effect_simulator")
    while not rospy.is_shutdown():
        try:
            ceil_sim = ceil_effect_simulator()
            print(1)

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        rospy.spin()

