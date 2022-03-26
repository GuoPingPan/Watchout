#include<ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<string>


int main(int argc,char**argv){
    ros::init(argc,argv,"broader_tf");
    
    ros::NodeHandle nh;
    std::string laser_frame;
    std::string base_frame;

    ros::param::param<std::string>("child_frame",laser_frame,"/laser");
    ros::param::param<std::string>("parent_frame",base_frame,"/odom");
    
    tf::TransformBroadcaster br;
    ros::Rate rate(50);
    while (ros::ok())
    {
        
        tf::Transform transfrom;
        tf::Quaternion q;

        transfrom.setOrigin(tf::Vector3(0,0,0));
        q.setRPY(0,0,1.57);
        transfrom.setRotation(q);
        br.sendTransform(tf::StampedTransform(
        transfrom,
        ros::Time::now(),
        base_frame,
        laser_frame
        ));
        ROS_INFO("sending the tf");
        rate.sleep();   
    }

    return 0; 
}