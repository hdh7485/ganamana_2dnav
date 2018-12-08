#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import copy
from collections import deque

pub = rospy.Publisher('scan_rotate', LaserScan, queue_size=10)

def callback(data):
    rotate_copy = copy.deepcopy(data)
    rotate_copy.ranges = data.ranges[len(data.ranges)//2:] + data.ranges[:len(data.ranges)//2]
    pub.publish(rotate_copy)

def rotate():
    rospy.init_node('laser_rotator', anonymous=True)
    sub = rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    rotate()
