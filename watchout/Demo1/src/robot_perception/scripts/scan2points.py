#!/usr/bin/env python

import rospy
from sensor_msgs.msgs import PointCloud2,LaserScan
import laser_geometry.laser_geometry as laser_geometry
import math

rospy.init_node('scan2points')
lp = lg.LaserProjection()
pc_pub = rospy.Publisher("/PointClouds",PointCloud2,queue_size=10)


def callback(scan_msg):
    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)
    

if __name__=='__main__':
rospy.Subscriber("/scan",LaserScan,callback,queue_size=1)
rospy.spin()