#! /usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
"""
class crack_point():
    def __init__(self):
        
    def position_callback(self, x, y):
"""        








rospy.init_node("markers_pub")

pub = rospy.Publisher("crack", MarkerArray, queue_size = 10)
rate = rospy.Rate(25)

#w=0

while not rospy.is_shutdown():

    marker_array = MarkerArray()

    for i in range(0, 3):
        marker_data = Marker()

        marker_data.header.frame_id = "/base_link"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "basic_shapes"
        #marker_data.id = 0

        marker_data.action = Marker.ADD

        if i == 0:
            marker_data.id = 0
            marker_data.pose.position.x = 0.0
            marker_data.pose.position.y = 0.0

        if i == 1:
            marker_data.id = 1
            marker_data.pose.position.x = 1.0
            marker_data.pose.position.y = 0.0

        if i == 2:
            marker_data.id = 2
            marker_data.pose.position.x = 0.0
            marker_data.pose.position.y = 1.0


        marker_data.pose.position.z = 1.05

        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=1.0
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.3
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()

        marker_data.type = 3

        marker_array.markers.append(marker_data)


    pub.publish(marker_array)

    rate.sleep()
