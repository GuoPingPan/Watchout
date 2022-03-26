from home.pgp.reconstruct.running import callback
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from multiprocessing import Process

point = PointCloud2()
odom = Odometry()

def lasercallback(scan_msgs):
    point = scan_msgs 


def odomcallback(odom_msgs):
    odom = odom_msgs

def runing():

    if(point.header.stamp == odom.header.stamp):
        print(1)


if __name__=='__main__':
    rospy.init_node('runing')
    
    lasersub = rospy.Subscriber('/PointClouds',PointCloud2,lasercallback)
    odomsub = rospy.Subscriber('/aft_mapped_to_init',Odometry,odomcallback)

    p = Process(target=runing)
    p.start()

    rospy.spin()
