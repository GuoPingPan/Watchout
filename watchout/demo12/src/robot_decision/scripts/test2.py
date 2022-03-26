#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node("ters")
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        broadcaster.sendTransform((1,0,0),(0.0,0.0,0.0, 1.0),
        rospy.Time.now(),"laser","map")
        rate.sleep()