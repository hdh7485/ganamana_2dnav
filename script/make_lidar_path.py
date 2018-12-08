import rospy
import copy
import pickle
import csv
import tf
import roslib
import math

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

rospy.init_node('lidar_path_node')

path = Path()
path.header.seq = 0
path.header.stamp = rospy.Time.now()
path.header.frame_id = 'scanmatcher_frame'

lidarWriter = csv.writer(open('lidar_path.csv', 'wb'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

rospy.loginfo('start')

path_pub = rospy.Publisher('/path', Path, queue_size=10)
listener = tf.TransformListener()
#(trans, rot) = listener.lookupTransform('scanmatcher_frame', '/base_link', rospy.Time(0)) 

listener.waitForTransform("/map", "/base_link", rospy.Time.now(), rospy.Duration(1.0))

while not rospy.is_shutdown():
    (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time()) 
    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose.position.x = trans[0]
    pose_stamped.pose.position.y = trans[1]
    pose_stamped.pose.position.z = 0
    a = copy.deepcopy(pose_stamped)
    path.poses.append(a)
    path_pub.publish(path) 
    rospy.loginfo(pose_stamped.pose.position.x)
    rospy.loginfo(pose_stamped.pose.position.y)
    lidarWriter.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y])

    '''
    angular = 4 * math.atan2(trans[1], trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    '''

    path_pub = rospy.Publisher('/path', Path, queue_size=10)

