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
from spinal.msg import PwmInfo
from spinal.msg import MotorInfo
from spinal.msg import Pwms  ############################################

class ceil_effect_controller():
    def __init__(self):

        self.rotor_num = 4
        self.rotor_diameter = 0.355
        self.offset = 0.04 #between cog and rotor 
        self.a_without_ceil = 0.0045800748
        self.b_without_ceil = -0.2564642520
        self.alfa = 0.035
        self.kappa = 1.6
        self.ceil_dist_inf = 3 #nondimensional

        self.ceil_dist_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.real_plane_dist)
        self.pwm_ceil_pub = rospy.Publisher("/motor_info",PwmInfo,queue_size=10)

        #self.pwm_sub =rospy.Subscriber("motor_pwms", Pwms, self.ceil_control_thrust)##############################
        #self.ceil_control_pub.Publisher("aerial_robot_contril_four_axis", FourAxisCommand, queue_size = 10)##################################################

        #TO DO sub
    #def ceil_detect(self):

        self.real_ceil_z =1.05

    def real_plane_dist(self, odom):
        self.real_ceil_dist = self.real_ceil_z - odom.pose.pose.position.z -self.offset
        if (self.real_ceil_dist < 0.062):
            self.real_ceil_dist = 0.062

        self.thrust_conversion_to_ceil(self.real_ceil_dist)

    def thrust_conversion_to_ceil(self, real_ceil_dist):
        self.thrust_ceil_coefficients(real_ceil_dist)

        coefficients_ceil = MotorInfo()
        coefficients_ceil.voltage = 24.0
        coefficients_ceil.polynominal[0] = 0.00
        coefficients_ceil.polynominal[1] = self.b_ceil*10
        coefficients_ceil.polynominal[2] = self.a_ceil*10
        coefficients_ceil.polynominal[3] = 0.00
        coefficients_ceil.polynominal[4] = 0.00

        pwm_msg = PwmInfo()
        pwm_msg.min_thrust = 1.0
        pwm_msg.max_thrust = 16.0
        pwm_msg.abs_max_pwm = 0.899999976158
        pwm_msg.force_landing_thrust = 6.5
        pwm_msg.pwm_conversion_mode = 0

        pwm_msg.motor_info.append(coefficients_ceil)

        self.pwm_ceil_pub.publish(pwm_msg)
        rospy.sleep(0.025)

    def thrust_ceil_coefficients(self, real_ceil_dist):
        ceil_func = 1 + self.alfa / math.pow((real_ceil_dist / self.rotor_diameter), self.kappa)
        self.a_ceil = self.a_without_ceil*ceil_func
        self.b_ceil = self.b_without_ceil*ceil_func


if __name__ == "__main__":

    rospy.init_node("hydrusx_ceil_effect_controller")

    while not rospy.is_shutdown():
        try:
            ceil_controller = ceil_effect_controller()

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        rospy.spin()

