#!/usr/bin/env python
import rospy
import copy
import csv
import math

from std_msgs.msg import Header
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    def __div__(self, other):
        return Point(self.x/other, self.y/other)
    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False
    def __str__(self):
        return "x:{}, y:{}".format(self.x, self.y)

def euclide_distance(point1, point2):
    return math.sqrt((point2.x-point1.x)**2 + (point2.y-point1.y)**2)

def LPF(x, pre_x, pre_y, tau, ts):
    return (tau * pre_y + ts * x )/(tau + ts) 

def calculate_angle(point1, point2, point3):
    vector1 = point3 - point2
    vector2 = point1 - point2
    x1, y1 = vector1.x, vector1.y
    x2, y2 = vector2.x, vector2.y
    inner_product = x1*x2 + y1*y2
    len1 = math.hypot(x1, y1)
    len2 = math.hypot(x2, y2)
    if len1*len2 == 0:
        return 0
    else:
        temp = inner_product/(len1*len2)
        if temp > 1.0: 
            temp = 1.0
        elif temp < -1.0: 
            temp = -1.0
        return math.acos(temp)

def pointlist_2_path(point_list, header):
    path = Path()
    path.header = header
    
    for point in point_list:
        pose_stamped = PoseStamped()
        pose_stamped.header = header

        pose_stamped.pose.position.x = point.x
        pose_stamped.pose.position.y = point.y
        pose_stamped.pose.position.z = 0

        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1

        a = copy.deepcopy(pose_stamped)
        path.poses.append(a)

    return path

with open('/home/odroid/catkin_ws/src/ganamana_2dnav/script/lidar_path.csv', 'rb') as f:
    reader = csv.reader(f)
    gps_data = list(reader)
    gps_data = [Point(float(x[0]), float(x[1])) for x in gps_data]

rospy.init_node('filtered_path_node')
raw_path_pub = rospy.Publisher('/raw_path', Path, queue_size=10)
filtered_path_pub = rospy.Publisher('/filtered_path', Path, queue_size=10)
LPF_path_pub = rospy.Publisher('/LPF_path', Path, queue_size=10)

header1 = Header()
header1.seq = 0
header1.stamp = rospy.Time.now()
header1.frame_id = 'map'

header2 = Header()
header2.seq = 1
header2.stamp = rospy.Time.now()
header2.frame_id = 'map'

header3 = Header()
header3.seq = 2
header3.stamp = rospy.Time.now()
header3.frame_id = 'map'

before_theta = 0
before_gps = gps_data[0]
before2_gps = gps_data[0]
filtered_gps = []
first = True

LPF_gps = []
before_gps = gps_data[0]
before_result = gps_data[0]
for gps in gps_data:
#for gps in gps_data:
    result_x = LPF(gps.x, before_gps.x, before_result.x, 3.0, 1.0)
    result_y = LPF(gps.y, before_gps.y, before_result.y, 3.0, 1.0)

    LPF_gps.append(Point(result_x, result_y))
    before_result = Point(result_x, result_y)
    before_gps = gps

raw_path = pointlist_2_path(gps_data, header1)
filtered_path = pointlist_2_path(filtered_gps, header2)
LPF_path = pointlist_2_path(LPF_gps, header3)

#for gps in gps_data:
for gps in LPF_gps:
    theta = math.degrees(calculate_angle(before2_gps, before_gps, gps))
    if gps == before_gps: continue
    #if euclide_distance(gps, before_gps) >= 0.6:
    if euclide_distance(gps, before_gps) >= 0.5:
        print("1!!!")
        #gps = before_gps + (before_gps - before2_gps)/1.4
        gps = before_gps + (before_gps - before2_gps)/1.0

    filtered_gps.append(gps)
    before2_gps = before_gps
    before_gps = gps
    before_theta = theta
    print("{},{},theta:{},dist:{}".format(before_gps, gps, theta, euclide_distance(gps, before_gps)))


LPF_gps = []
before_gps = filtered_gps[0]
before_result = filtered_gps[0]
for gps in filtered_gps:
#for gps in gps_data:
    #result_x = LPF(gps.x, before_gps.x, before_result.x, 4.0, 1.0)
    #result_y = LPF(gps.y, before_gps.y, before_result.y, 4.0, 1.0)
    result_x = LPF(gps.x, before_gps.x, before_result.x, 3.0, 1.0)
    result_y = LPF(gps.y, before_gps.y, before_result.y, 3.0, 1.0)

    LPF_gps.append(Point(result_x, result_y))
    before_result = Point(result_x, result_y)
    before_gps = gps

raw_path = pointlist_2_path(gps_data, header1)
filtered_path = pointlist_2_path(filtered_gps, header2)
LPF_path = pointlist_2_path(LPF_gps, header3)

r = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    raw_path_pub.publish(raw_path)
    filtered_path_pub.publish(filtered_path)
    LPF_path_pub.publish(LPF_path)
