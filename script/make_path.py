import rospy
import math
import copy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

rospy.init_node('path_node')
path_pub = rospy.Publisher('/path', Path, queue_size=10)

path = Path()

def route(point1, point2):
    global path
    path.header.seq = 111
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'test_path'

    distance = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose.position.x = point1.x
    pose_stamped.pose.position.y = point1.y
    pose_stamped.pose.position.z = 0
    for i in range(int(distance*100)):
        print(pose_stamped.pose.position.x)
        print(pose_stamped.pose.position.y)
        print(pose_stamped)
        a = copy.deepcopy(pose_stamped)
        path.poses.append(a)

        pose_stamped.pose.position.x += (point2.x - point1.x)/(distance*100)
        pose_stamped.pose.position.y += (point2.y - point1.y)/(distance*100)
        pose_stamped.pose.position.z = 0

        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1

point1 = Point(0.0, 0.0)
point2 = Point(1.5, 0.0)
route(point1, point2)

point1 = Point(1.7, 0.2)
point2 = Point(1.7, 1.2)
route(point1, point2)

point1 = Point(1.3, 1.2)
point2 = Point(1.3, 0.6)
route(point1, point2)

point1 = Point(1.1, 0.4)
point2 = Point(0.4, 0.4)
route(point1, point2)

point1 = Point(0.2, 0.6)
point2 = Point(0.2, 1.2)
route(point1, point2)

point1 = Point(-0.2, 1.2)
point2 = Point(-0.2, 0.4)
route(point1, point2)

point1 = Point(-0.2, 0.2)
point2 = Point(0, 0)
route(point1, point2)

r = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    path_pub.publish(path)
