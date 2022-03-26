import rospy
from std_msgs.msg import String
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TransformStamped

rospy.init_node('pub')

rate = rospy.Rate(10)

laserpub = rospy.Publisher('/stringone',TransformStamped,queue_size=1)
odompub = rospy.Publisher('/stringtwo',TransformStamped,queue_size=1)
while not rospy.is_shutdown():
    s1 = TransformStamped()
    s2 = TransformStamped()

    s1.header.frame_id = 'one'
    s2.header.frame_id = 'two'
    s1.header.stamp = rospy.Time.now()
    s2.header.stamp = rospy.Time.now()

    laserpub.publish(s1)
    odompub.publish(s2)
    # rate.sleep()