#!/usr/bin/env python
import rospy
import numpy as np
import math
from time import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
pub=rospy.Publisher('/bot_diff_drive_controller/cmd_vel',Twist,queue_size=10)
def callback(data):
    global pub
    angle = data.angle_max
    Vx = 250
    Vy = 250
    a = data.ranges[120:240]
    a = np.array(a)
    #print(data.ranges)

    for r in data.ranges:
        if r == float ('Inf'):
            r = data.range_max
        x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
        y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
        Vx+=x
        Vy+=y
        angle= angle - data.angle_increment
    ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
    print(ang)
    if ang > 5 and ang <90:
        pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-100)))
	print('right')
    elif ang <355 and ang >270:
        pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,100)))
	print('left')
    else:
	if data.ranges[int(angle/2)] < 0.5:        
		pub.publish(Twist(Vector3(-100,0,0),Vector3(0,0,0)))        
		print('Backward')
	else:
		pub.publish(Twist(Vector3(100,0,0),Vector3(0,0,0)))        		
		print('Straight')
	


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    print('lalalala')
    rospy.Subscriber("/scan", LaserScan,callback)
    rospy.spin()
    sleep(rospy.sleep(1))

if __name__ == '__main__':
    laser_listener()

