#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("talker")
    pub1 = rospy.Publisher("/pub1",String,queue_size=10)
    pub2 = rospy.Publisher("/pub2",String,queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        helloworld1 = "hellow_world_1 %s" % rospy.get_rostime()
        helloworld2 = "hellow_world_2 %s" % rospy.get_rostime()

        print(helloworld1)
        pub1.publish(helloworld1)
        pub2.publish(helloworld2)
        rate.sleep()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSException:
        pass