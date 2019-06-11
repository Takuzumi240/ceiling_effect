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
from spinal.msg import Barometer

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


        self.average_data_num = 5###
        self.ceil_dist = []###
        self.laser_range_min = 0.055
        self.laser_range_max = 2.00

        self.pwm_ceil_pub = rospy.Publisher("/motor_info",PwmInfo,queue_size=10)
        self.ceil_dist_sub =rospy.Subscriber("/vl53l0x/range", Barometer, self.ceil_detect)
        self.ceil_dist_pub =rospy.Publisher("/ceil_dist", Barometer, queue_size = 10)

    def ceil_detect(self, dist):
        real_ceil_dist = dist.altitude


        if (real_ceil_dist > self.laser_range_max):
            return

        self.ceil_dist.append(real_ceil_dist)


        #Filter
        if (len(self.ceil_dist) == self.average_data_num):
            dist_z = sum(self.ceil_dist)/len(self.ceil_dist)
            self.ceil_dist.pop(0)

        elif(len(self.ceil_dist) < self.average_data_num):
             return
             self.ceil_dist.pop(0)

        else:
             print("Data num error!")
             return

        #Ceil Distance
        if (dist_z <= self.laser_range_max and dist_z >= self.laser_range_min):
            self.real_ceil_dist = dist_z

        elif (dist_z < self.laser_range_min or dist_z > self.laser_range_max):
            self.real_ceil_dist = self.laser_range_max

        self.thrust_conversion_to_ceil(self.real_ceil_dist)

        ceil = Barometer()
        ceil.altitude = self.real_ceil_dist
        self.ceil_dist_pub.publish(ceil)


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

        #CHECK!!!!!
        #rospy.sleep(0.01)

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

