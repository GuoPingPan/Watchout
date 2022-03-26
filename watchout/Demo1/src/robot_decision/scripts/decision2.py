import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


init = 1

def main():
    if init:
        pcl_raw = rospy.wait_for_message("/ObstaclePoint",PointCloud2,timeout=rospy.Duration(1.0))
        pcd_last = pc2.read_points_list(pcl_raw,field_names=("x","y"),skip_nans=True)
    else:
        pcl_raw = rospy.wait_for_message("/ObstaclePoint",PointCloud2,timeout=rospy.Duration(1.0))
        pcd_last = pc2.read_points_list(pcl_raw,field_names=("x","y"),skip_nans=True)
        compare()
        odom = rospy.wait_for_message("/Odom",Odometry,timeout=rospy.Duration(1.0))
        