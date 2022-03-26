import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import message_filters


def callback(scan,odom):
    pass

lasersub = message_filters.Subscriber('/PointClouds',PointCloud2)
odomsub = message_filters.Subscriber('/aft_mapped_to_init',Odometry)


sub = message_filters.TimeSynchronizer([lasersub,odomsub],10)
sub.registerCallback(callback)