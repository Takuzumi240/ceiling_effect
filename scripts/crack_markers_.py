#! /usr/bin/env python

import rospy
import sys, select, termios, tty

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class crack_point():
    def __init__(self):
        self.marker_array = MarkerArray()
        self.crack_num = 0
        self.pub = rospy.Publisher("cracks", MarkerArray, queue_size = 10)
        self.rate = rospy.Rate(25)

        self.x = 0.0
        self.y = 0.0

    def position_callback(self, pos):
        self.x = pos.pose.pose.position.x
        self.y = pos.pose.pose.position.y

    def marker_pos(self):
        print"called!!",(self.x, self.y)
        self.crack_marker()
        self.crack_num += 1

    def crack_marker(self):
        marker_data = Marker()
        marker_data.header.frame_id = "/world"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.action = Marker.ADD

        marker_data.pose.position.z = 1.05

        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=1.0
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.05
        marker_data.scale.y = 0.05
        marker_data.scale.z = 0.01

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 3

        marker_data.id = self.crack_num
        marker_data.pose.position.x = self.x
        marker_data.pose.position.y = self.y

        self.marker_array.markers.append(marker_data)
        self.pub.publish(self.marker_array)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("markers_pub")
    crack = crack_point()
    pos_sub = rospy.Subscriber("/uav/cog/odom", Odometry, crack.position_callback)

    try:
        while(True):
                crack.crack_marker()
                key = getKey()
                print "the key value is %d" % ord(key)

                if (key == ' '):
                    crack.marker_pos()
                if (key == '\x03'):
                    break
    except Exception as e:
        print e
        print repr(e)


    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
