from time import time
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import rospy


def callback(s1,s2):
    print('b')
    print(s1.header.frame_id)
    print(s2.header.frame_id)

t1 = TransformStamped()
t2 = TransformStamped()

def callback1(s1):
    t1 = s1
    print('1')
def callback2(s2):
    t2 = s2
    print('2')

import time
def kin():
    while 1:
        if (t1.header.stamp == t2.header.stamp):
            print('3')
        time.sleep(0.1) 

if __name__=='__main__':
    rospy.init_node('sub')
    # sub1 = message_filters.Subscriber('/stringone',TransformStamped)
    # sub2 = message_filters.Subscriber('/stringtwo',TransformStamped)

    sub1 = rospy.Subscriber('stringone',TransformStamped,callback=callback1)
    sub2 = rospy.Subscriber('stringtwo',TransformStamped,callback=callback2)
    # import threading
    # thread = threading.Thread(target=kin)
    from multiprocessing import Process
    p1 = Process(target=kin)
    p1.start()
    # ts = message_filters.TimeSynchronizer([sub1,sub2],queue_size=10)
    print('a')
    rospy.spin()