#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<iostream>

using namespace std;

// tf::StampedTransform并不支持访问数据


int main(int argc,char **argv){
    ros::init(argc,argv,"checktf");
    ros::NodeHandle nh;
    tf::TransformListener checktf;
    checktf.waitForTransform("/laser","/base_footprint",ros::Time(0),ros::Duration(3.0));
    tf::StampedTransform tr1;
    ros::Rate rate(20);
    while(1){
        checktf.lookupTransform("/laser","/base_footprint",ros::Time(0),tr1);
        rate.sleep();
    }
    
}