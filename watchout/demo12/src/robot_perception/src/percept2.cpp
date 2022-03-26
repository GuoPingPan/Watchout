#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/common/transforms.h>
#include<pcl-1.8/pcl/conversions.h>
#include<pcl_ros/impl/transforms.hpp>
#include<pcl_ros/transforms.h>
#include<pcl_conversions/pcl_conversions.h>


class MessageManager{
public:
    MessageManager(ros::NodeHandle &n,ros::Rate &r):nh(n),rate(r){
        obstaclepub = nh.advertise<sensor_msgs::PointCloud2>("/ObstaclePoints",10);
        perceptpub = nh.advertise<sensor_msgs::PointCloud2>("/PerceptPoints",10);
        lasersub = nh.subscribe("/scan",10,&MessageManager::callback,this);
        ros::spin();
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Publisher obstaclepub;
    ros::Subscriber lasersub;
    ros::Publisher perceptpub;
    tf::TransformListener listener;
};


void MessageManager::callback(const sensor_msgs::LaserScan::ConstPtr &scan){
    int count = scan->scan_time / scan->time_increment;

    pcl::PointCloud<pcl::PointXYZ> Obstaclepoints,Predictpoints;

    for(int i=0;i<count;i++){
        if(scan->ranges[i]==std::numeric_limits<float>::infinity())
            continue;
        else if(scan->ranges[i]>4){
            float angle = scan->angle_min+scan->angle_increment*i;
            Predictpoints.points[i].x = scan->ranges[i]*cosf(angle);
            Predictpoints.points[i].y = -scan->ranges[i]*sinf(angle);
        }
        else{
            float angle = scan->angle_min+scan->angle_increment*i;
            Obstaclepoints.points[i].x = scan->ranges[i]*cosf(angle);
            Obstaclepoints.points[i].y = -scan->ranges[i]*sinf(angle);
        }
    }
    
    tf::StampedTransform laser2world_tf;
    Eigen::Matrix4f laser2world;
    
    try{
        listener.waitForTransform(scan->header.frame_id,"/base_footprint",scan->header.stamp,ros::Duration(3.0));

    }
    catch(tf::TransformException &ex){
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ> Obstacleworld,Perceptworld;
    
    // listener.lookupTransform(scan->header.frame_id,"/base_footprint",scan->header.stamp,laser2world_tf);
    
    
    pcl_ros::transformAsMatrix(laser2world_tf,laser2world);
    pcl::transformPointCloud(Obstaclepoints,Obstacleworld,laser2world);
    pcl::transformPointCloud(Predictpoints,Perceptworld,laser2world);
    
    sensor_msgs::PointCloud2 Obstaclepoint2,Perceptpoint2;
    
    pcl::toROSMsg(Obstacleworld,Obstaclepoint2);
    pcl::toROSMsg(Perceptworld,Perceptpoint2);

    Obstaclepoint2.header.stamp = scan->header.stamp;
    Obstaclepoint2.header.frame_id = "/base_footprint";
    Perceptpoint2.header.stamp = scan->header.stamp;
    Perceptpoint2.header.frame_id = "/base_footprint";
    
    obstaclepub.publish(Obstaclepoint2);
    perceptpub.publish(Perceptpoint2);
}



int main(int argc,char **argv){
    ros::init(argc,argv,"percept2");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    MessageManager manager(nh,rate);
    // ros::Subscriber lasersub = nh.subscribe("/scan",1,callback)

}