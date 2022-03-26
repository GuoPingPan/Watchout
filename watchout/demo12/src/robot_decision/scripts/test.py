#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan,PointCloud2
import threading
import time
def callback1(msg):
    pcd = rospy.wait_for_message("/PointClouds",PointCloud2,timeout=rospy.Duration(3.0))
    
    print("one")

def callback2(msg):
    print("two")

def one():
    rospy.spin()

def main():
    print("main")


if __name__=="__main__":
    rospy.init_node("test")
    sub1 = rospy.Subscriber("/scan",LaserScan,callback1)
    # thread = threading.Thread(target=one)
    # thread.start()
    while True:
        time.sleep(0.1)
        main()
    rospy.spin()
