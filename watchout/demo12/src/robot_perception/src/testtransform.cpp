#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include<ros/ros.h>
#include<tf/transform_listener.h>
// #include<pcl-1.8/pcl/point_cloud.h>
// #include<pcl-1.8/pcl/point_types.h>
// #include<pcl-1.8/pcl/common/transforms.h>
// #include<pcl-1.8/pcl/conversions.h>
// #include<pcl_ros/impl/transforms.hpp>
// #include<pcl_ros/transforms.h>
// #include<pcl_conversions/pcl_conversions.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<sensor_msgs/point_cloud_conversion.h>
#include<iostream>
#include<string>
#include<laser_geometry/laser_geometry.h>


class MessageManager{
public:
    MessageManager(ros::NodeHandle &n,ros::Rate &r,std::string &base_name):nh(n),rate(r),base_frame(base_name){
        obstaclepub = nh.advertise<sensor_msgs::PointCloud2>("/ObstaclePoints",10);
        // perceptpub = nh.advertise<sensor_msgs::PointCloud2>("/PerceptPoints",10);
        lasersub = nh.subscribe("/scan",10,&MessageManager::callback,this);
        point_pub=nh.advertise<sensor_msgs::PointCloud2>("/PointClouds",10);

        ros::spin();
        
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
	std::string base_frame;
    ros::Rate rate;
    ros::NodeHandle nh;
    ros::Publisher obstaclepub;
    ros::Subscriber lasersub;
    ros::Publisher perceptpub;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    ros::Publisher point_pub;
};



void MessageManager::callback(const sensor_msgs::LaserScan::ConstPtr &scan){

    // ROS_INFO("yes");
    int count = scan->scan_time/ scan->time_increment;
    // ROS_INFO("yes");

    sensor_msgs::PointCloud Obstaclepoints,Predictpoints;
    sensor_msgs::PointCloud Obstacleworld,Predictworld;
    Obstaclepoints.header.frame_id = scan->header.frame_id;
    Predictpoints.header.frame_id = scan->header.frame_id;
    Obstaclepoints.points.resize(count);
    Predictpoints.points.resize(count);
    int p = 0;
    int o = 0;
    ROS_INFO("COUNT = %d",count );
    for(int i=0;i<count;i++){
        
        if(scan->ranges[i]==std::numeric_limits<float>::infinity()){
            continue;
        }                                                                //排除深度为无穷的数据
        float angle = scan->angle_min+scan->angle_increment*i;
        Obstaclepoints.points[o].x = scan->ranges[i]*cosf(angle);   //深度小于4的数据用于避障
        Obstaclepoints.points[o].y = scan->ranges[i]*sinf(angle);
        Obstaclepoints.points[o].z = 0;
        o++;
    }

    // ROS_INFO("yes");
    
    
    try{
        //tf::StampedTransform transform;
        listener.waitForTransform(scan->header.frame_id,
                                                                base_frame,
                                                                scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                                                                ros::Duration(3.0));
        // listener.lookupTransform("/base_footprint",scan->header.frame_id,scan->header.stamp,transform);
        listener.transformPointCloud(base_frame,Obstaclepoints,Obstacleworld);
        // listener.transformPointCloud(base_frame,Predictpoints,Predictworld);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }
    
      this->projector.transformLaserScanToPointCloud(
        base_frame,
        *scan,
        cloud,
        listener);
    this->point_pub.publish(cloud);

    // ROS_INFO("yes");
    
    sensor_msgs::PointCloud2 Obstaclepoint2,Perceptpoint2;
    
    sensor_msgs::convertPointCloudToPointCloud2(Obstacleworld,Obstaclepoint2);
    // sensor_msgs::convertPointCloudToPointCloud2(Predictworld,Perceptpoint2);

    Obstaclepoint2.header.stamp = scan->header.stamp;
    // Obstaclepoint2.header.frame_id = "/base_footprint";
    // Perceptpoint2.header.stamp = scan->header.stamp;
    // Perceptpoint2.header.frame_id = "/base_footprint";
    
    obstaclepub.publish(Obstaclepoint2);
    // perceptpub.publish(Perceptpoint2);
}



int main(int argc,char **argv){
    ros::init(argc,argv,"percept2");
    ros::NodeHandle nh;
    ros::Rate rate(10);
	std::string base_frame;
	nh.param<std::string>("base_frame",base_frame,"base_footprint");
    MessageManager manager(nh,rate,base_frame);

}
