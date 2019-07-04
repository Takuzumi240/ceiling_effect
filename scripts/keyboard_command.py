#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav

import sys, select, termios, tty

msg = """

    r: Right
    l: Left
    f: Forward
    b: Back
    u: Up
    d: Down


    please don't have caps lock on.
    CTRL+c to quit
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

class simple_keyboard_tracker():

        def __init__(self):
                self.x = 0
                self.y = 0
                self.z = 0.5
                self.step = 0.05
                self.uav_nav_pub = rospy.Publisher('/uav/nav', FlightNav, queue_size = 1)
                self.desire_nav = FlightNav()

        def target(self, key):


                if key == 'l':
                        self.y = self.y + self.step

                        self.desire_nav.pos_xy_nav_mode = 2
                        self.desire_nav.target_pos_y = self.y

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Left : ", self.y)

                if key == 'r':
                        self.y = self.y - self.step

                        self.desire_nav.pos_xy_nav_mode = 2
                        self.desire_nav.target_pos_y = self.y

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Right : ", self.y)

                if key == 'f':
                        self.x = self.x + self.step

                        self.desire_nav.pos_xy_nav_mode = 2
                        self.desire_nav.target_pos_x = self.x

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Forward : ", self.x)

                if key == 'b':
                        self.x = self.x - self.step

                        self.desire_nav.pos_xy_nav_mode = 2
                        self.desire_nav.target_pos_x = self.x

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Back : ", self.x)
                if key == 'u':
                        self.z = self.z + self.step

                        self.desire_nav.pos_z_nav_mode = 2
                        self.desire_nav.target_pos_z = self.z

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Up : ", self.z)

                if key == 'd':
                        self.z = self.z - self.step

                        self.desire_nav.pos_z_nav_mode = 2
                        self.desire_nav.target_pos_z = self.z

                        self.uav_nav_pub.publish(self.desire_nav)
                        print("Down : ", self.z)

                if (key == '\x03'):
                        return

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        print msg

        rospy.init_node('simple_keyboard_tracker')
        #the way to write publisher in python
        comm=Int8()
        gain=UInt16()
        desire_point = simple_keyboard_tracker()

        try:
                while(True):
                        key = getKey()
                        print "the key value is %d" % ord(key)
                        desire_point.target(key)

                        if (key == '\x03'):
                                break

                        else:
                                if (key == '\x03'):
                                        break
        except Exception as e:
                print e
                print repr(e)


        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


