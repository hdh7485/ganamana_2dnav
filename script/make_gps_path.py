import rospy
import copy
import pickle
import csv

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from marvelmind_nav.msg import hedge_pos_ang

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

rospy.init_node('gps_path_node')

path = Path()
path.header.seq = 0
path.header.stamp = rospy.Time.now()
path.header.frame_id = 'gps_path'

gpsWriter = csv.writer(open('gps_path.csv', 'wb'), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

rospy.loginfo('start')
    
def gps_callback(gps_msg):
    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose.position.x = gps_msg.x_m
    pose_stamped.pose.position.y = gps_msg.y_m
    pose_stamped.pose.position.z = 0
    a = copy.deepcopy(pose_stamped)
    path.poses.append(a)
    path_pub.publish(path) 
    rospy.loginfo(pose_stamped.pose.position.x)
    rospy.loginfo(pose_stamped.pose.position.y)
    gpsWriter.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y])

path_pub = rospy.Publisher('/path', Path, queue_size=10)
rospy.Subscriber('/hedge_pos_ang', hedge_pos_ang, gps_callback)

rospy.spin()
