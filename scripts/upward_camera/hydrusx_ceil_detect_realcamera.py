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



def plane_detect(message):

    if len(message.coefficients) == 0:
        return

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
                           "camera_depth_optical_frame")



if __name__ == "__main__":

    rospy.init_node("hydrus_ceil_detect_realcamera")


    ceil_dist_pub = rospy.Publisher('ceil_dist', distance_to_ceilingwall, queue_size=1)
    cam_listener = tf.TransformListener()
    ceil_listener = tf.TransformListener()

    ceil = distance_to_ceilingwall() 

    time.sleep(1)

    while not rospy.is_shutdown():
        try:
            plane_sub = rospy.Subscriber("/multi_plane_segmentation/output_refined_coefficients",ModelCoefficientsArray, plane_detect)
            (ceil_trans, cail_rot) = ceil_listener.lookupTransform("camera_depth_optical_frame", "ceiling_wall", rospy.Time())
            ceil.distance = ceil_trans[2]
            ceil_dist_pub.publish(ceil)
            #print"Ceiling wall is{", ceil.distance, "}(m) in the z direction"

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(1)
            continue

        rospy.sleep(1)



    rospy.spin()

