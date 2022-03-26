#include<std_msgs/String.h>
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/UInt16.h> 


/* 注意这里的ros::spinOnce()要和发送速率对齐，及发送数据以10Hz发送，则以5Hz接收时需保证 queue_size = 2 以至于不掉包*/

void callback1(const std_msgs::String::ConstPtr &s){
    std::cout<<s->data.c_str()<<std::endl;
    
}

void subone(){
    ros::NodeHandle n1;
    ros::Subscriber sub1 = n1.subscribe("/helloworld1",10,callback1);
    ros::spinOnce();
}

void callback2(const std_msgs::String::ConstPtr &s){
    std::cout<<s->data.c_str()<<std::endl;
    
}

void subtwo(){
    ros::NodeHandle n2("~");
    ros::Subscriber sub2 = n2.subscribe("/helloworld2",10,callback2);
    ros::spinOnce();
}


void callback(const std_msgs::String::ConstPtr &s){
    std::cout<<s->data.c_str()<<std::endl;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"listener",ros::init_options::AnonymousName);
    ros::NodeHandle nh("listener");
    // subone();
    // subtwo();
    ros::Subscriber sub = nh.subscribe("/helloworld1",1,callback1);
    ros::Rate rate(5);
    int i = 0;
    while (1)
    {
        ros::spinOnce();
        rate.sleep();   
        i++;

    }

    std::cout<<"error"<<std::endl;
    return 0;
    // }
    // ros::shutdown();
}