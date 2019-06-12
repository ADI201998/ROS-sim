#!/usr/bin/env python
import rospy
import numpy as np
import math
from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
pub=rospy.Publisher('/bot_diff_drive_controller/cmd_vel',Twist,queue_size=10)
def callback(data):
    global pub
    print("asasasasasasasasa")
    if data.data=="left":
        pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,100)))
        print(data)
    elif data.data=='right':
        pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-100)))
        print(data)
    else:
        pub.publish(Twist(Vector3(100,0,0),Vector3(0,0,0)))  
        print(data)
	print("oooooooooooo")


def kin_listener():
    rospy.init_node('kin_listener', anonymous=True)
    print('lalalala')
    rospy.Subscriber("/kinect_data", String,callback)
    rospy.spin()
    #sleep(rospy.sleep(1))

if __name__ == '__main__':
    kin_listener()

