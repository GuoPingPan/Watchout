#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def callback(msg):
    point_cloud_data = pc2.read_points_list(msg,field_names=("x", "y", "z"),skip_nans=True)
    for each in point_cloud_data:
        print(each.x,each.y,each.z)


if __name__=='__main__':
    rospy.init_node('showpoint',anonymous=True)
    rospy.Subscriber('/PointClouds',PointCloud2,callback)
    rospy.spin()