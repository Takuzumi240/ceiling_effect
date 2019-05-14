#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf


from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import TransformStamped
from jsk_recognition_msgs.msg import ModelCoefficientsArray

#class ceil_effect_control(object):

def plane_detect(message):

    #a*x+b*y+c*z+d=0
    a = message.coefficients[0].values[0]
    b = message.coefficients[0].values[1]
    c = message.coefficients[0].values[2]
    d = message.coefficients[0].values[3]

    #x0=y0=z0=0 in cam_choords
    k = -(a*0+b*0+c*0+d)/(a*a+b*b+c*c)
    ceil_x = k*a
    ceil_y = k*b
    ceil_z = k*c

    plane_br = tf.TransformBroadcaster()
    plane_br.sendTransform((ceil_x, ceil_y, ceil_z),
                           tf.transformations.quaternion_from_euler(0, 0, 0),
                           rospy.Time.now(),
                           "ceiling_wall",
                           "upward_camera_optical_frame")



if __name__ == "__main__":

    rospy.init_node("hydrus_simple_object_detection")

    cam_listener = tf.TransformListener()
    ceil_listener = tf.TransformListener()

    time.sleep(1)

    while not rospy.is_shutdown():
        try:
            plane_sub2 = rospy.Subscriber("/multi_plane_segmentation/output_refined_coefficients",ModelCoefficientsArray, plane_detect)
            (cam_trans, cam_rot) = cam_listener.lookupTransform("world", "upward_camera_optical_frame", rospy.Time())
            (ceil_trans, cail_rot) = ceil_listener.lookupTransform("world", "ceiling_wall", rospy.Time())
            print"Ceiling wall is{", ceil_trans[2]-cam_trans[2], "}(m) in the z direction"

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            print"error"
            continue

        rospy.sleep(1)

    rospy.spin()

