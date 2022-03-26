#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading

msg1 = "hellow"
msg2 = "hellow"



def callback1(msg):
    print("callback1")
    print(msg)

    msg1 = msg

def callback2(msg):
    print("callback2")
    print(msg)
    msg2 = msg

def subone():
    print("1")
    sub1 = rospy.Subscriber("/pub1",String,callback1,queue_size=1)
    rospy.spin()


def subtwo():
    print("1")
    sub2 = rospy.Subscriber("/pub2",String,callback2,queue_size=1)
    rospy.spin()




def main():
    rospy.init_node("listener")
    thread1 = threading.Thread(target=subone)
    thread2 = threading.Thread(target=subtwo) 
    thread1.start()
    thread2.start()
    # while(1):
    #     print("main")
    #     print(msg1)
    #     print(msg2)
    
def main2():
    rospy.init_node("listener")
    data1 = rospy.wait_for_message("/pub1",String,timeout=None)
    data2 = rospy.wait_for_message("/pub2",String,timeout=None)
    print(data1)
    print(data2)
    


if __name__=="__main__":
    try:
        main2()
    except rospy.ROSException:
        print("error")
        pass    