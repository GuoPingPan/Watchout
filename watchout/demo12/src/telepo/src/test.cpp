#include<ros/ros.h>
#include<geometry_msgs/Twist.h>


void callback(const geometry_msgs::Twist &cmd){
    ROS_INFO("%d %d %d",cmd.linear.x,cmd.linear.y,cmd.angular.z);
}


int main(int argc,char** argv){
    ros::init(argc,argv,"tests");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/cmd_vel",10,callback);
    ros::spin();
}

