#include<ros/ros.h>
#include<string>
#include<iostream>

int main(int argc,char ** argv){
    ros::init(argc,argv,"name");
    ros::NodeHandle nh;
    std::string base_frame;
    nh.param<std::string>("/base_frame",base_frame,"map");
    std::cout<<base_frame<<std::endl;
    std::string odom_frame;

    ros::param::param<std::string>("/odom_frame",odom_frame,"odom");
    std::cout<<odom_frame<<std::endl;

}