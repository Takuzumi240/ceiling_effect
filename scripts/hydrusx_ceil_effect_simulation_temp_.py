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

#class ceil_effect_control(object):


def temp_throttle(temp_cmd):
    #print(temp_cmd.base_throttle)


    thrust_without_ceil = temp_cmd.base_throttle

    pulse_length = thrust_conversion_to_pwm(thrust_without_ceil)
    thrust_ceil = pwm_to_thrust_ceil(pulse_length)
    temp_cmd.base_throttle = thrust_ceil
    print(temp_cmd.base_throttle)

    throttle_pub = rospy.Publisher("/aerial_robot_control_four_axis",FourAxisCommand, queue_size=10)
    throttle_pub.publish(temp_cmd)


def thrust_conversion_to_pwm(thrust_without_ceil):

    rotor_num = 4
    a_without_ceil = 0.0045800748
    b_without_ceil = -0.2564642520
    pulse_length = []
    for i in range(0, rotor_num):
        mu_sqrt = math.sqrt(b_without_ceil*b_without_ceil + 4*a_without_ceil*thrust_without_ceil[i])
        mu = (-b_without_ceil + mu_sqrt)/(2 * a_without_ceil)
        pulse_length.append(mu*20)

    return pulse_length

def pwm_to_thrust_ceil(pulse_length):
    duty_ratio = map(lambda mu: mu/20, pulse_length)
    rotor_num =4
    ceil_dist = 3 #nondimensional
    a_ceil, b_ceil = thrust_ceil_coefficients(ceil_dist)
    thrust_ceil = []

    for i in range (0, rotor_num):
        thrust = a_ceil*duty_ratio[i]*duty_ratio[i]+b_ceil*duty_ratio[i]
        thrust_ceil.append(thrust)

    return thrust_ceil


def real_plane_dist(odom):

    real_ceil_z = 1.5
    global real_ceil_dist
    real_ceil_dist = real_ceil_z - odom.pose.pose.position.z

def thrust_ceil_coefficients(ceil_dist):

    alfa = 0.035
    kappa = 1.4

    ceil_func = 1 + alfa / math.pow((real_ceil_dist / 0.355), kappa) 
    a_ceil = 0.0045800748*ceil_func
    b_ceil = -0.2564642520*ceil_func

    return a_ceil, b_ceil








def plane_dist(ceil_dist):
    print(ceil_dist)
    

if __name__ == "__main__":

    rospy.init_node("hydrusx_ceil_effect_simulation")


    ceil_dist = distance_to_ceilingwall()
    throttle = FourAxisCommand()

    
    time.sleep(1)



    while not rospy.is_shutdown():
        try:
            cog_odom_sub = rospy.Subscriber("/uav/cog/odom", Odometry, real_plane_dist)
            ceil_dist_sub = rospy.Subscriber('ceil_dist', distance_to_ceilingwall, plane_dist)
            temp_throttle_sub = rospy.Subscriber("/aerial_robot_control_four_axis_temp",FourAxisCommand, temp_throttle)

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        #rospy.sleep(1)

        rospy.spin()


