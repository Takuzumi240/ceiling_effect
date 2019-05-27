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
from spinal.msg import MotorInfo
from spinal.msg import Pwms

class ceil_effect_controller():
    def __init__(self):

        self.rotor_num = 4
        self.rotor_diameter = 0.355
        self.offset = 0.02 #0.053 #between cog and rotor 
        self.a_without_ceil = 0.0045800748
        self.b_without_ceil = -0.2564642520
        self.alfa = 0.035
        self.kappa = 1.4
        self.ceil_dist_inf = 3 #nondimensional

        self.ceil_dist_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.real_plane_dist)
        self.pwm_ceil_pub = rospy.Publisher("/motor_info",MotorInfo,queue_size=10)

        self.pwm_sub = rospy.Subscriber("motor_pwms", Pwms, self.thrust_conversion_to_ceil)##########

        #TO DO sub
    #def ceil_detect(self):

        self.real_ceil_z =1.5

    def real_plane_dist(self, odom):
        self.real_ceil_dist = self.real_ceil_z - odom.pose.pose.position.z -self.offset
        #self.thrust_conversion_to_ceil(self.real_ceil_dist)

    def thrust_conversion_to_ceil(self, pwms):
        self.thrust_ceil_coefficients(self.real_ceil_dist)

        coefficients_ceil = MotorInfo()
        coefficients_ceil.voltage = 24.0
        coefficients_ceil.polynominal[0] = 0.00
        coefficients_ceil.polynominal[1] = self.b_ceil*10
        coefficients_ceil.polynominal[2] = self.a_ceil*10
        coefficients_ceil.polynominal[3] = 0.00
        coefficients_ceil.polynominal[4] = 0.00
        self.pwm_ceil_pub.publish(coefficients_ceil)
        print(coefficients_ceil)

        print("T=======",self.a_ceil*pwms.motor_value[1]*pwms.motor_value[1]/400+self.b_ceil*pwms.motor_value[1]/20)

    def thrust_ceil_coefficients(self, real_ceil_dist):
        ceil_func = 1 + self.alfa / math.pow((real_ceil_dist / self.rotor_diameter), self.kappa)
        self.a_ceil = self.a_without_ceil*ceil_func
        self.b_ceil = self.b_without_ceil*ceil_func




if __name__ == "__main__":

    rospy.init_node("hydrusx_ceil_effect_simulation")

    while not rospy.is_shutdown():
        try:
            ceil_controller = ceil_effect_controller()
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        rospy.spin()

