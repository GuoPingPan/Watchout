import rospy
from nav_msgs.msg import Odometry
from rospy.timer import Rate


if __name__ == '__main__':
	rospy.init_node('testodom')
	pub = rospy.Publisher('/aft_mapped_to_init',Odometry,queue_size=1)
	rate = Rate(10)

	while not rospy.is_shutdown():
		odom = Odometry()
		odom.header.frame_id = "/camera_init"
		odom.child_frame_id = "/aft_mapped"
		odom.pose.pose.position.x=1
		odom.pose.pose.orientation.w=1

		pub.publish(odom)

		rate.sleep()
		
