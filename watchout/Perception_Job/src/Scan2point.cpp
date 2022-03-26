#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<laser_geometry/laser_geometry.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<string>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Transform.h>
#include<pcl_ros/transforms.h>
#include<iostream>

using namespace std;


// #include<pcl/filter/passthrough.h>
class scan2points{
public:
    
    //初始化
    scan2points(ros::NodeHandle &n,std::string &map_frame_):nh(n),map_frame(map_frame_){
        
        point_pub=nh.advertise<sensor_msgs::PointCloud2>("/PointClouds",10);
        scan_sub=nh.subscribe("/scan",10,&scan2points::scanCallback,this);
        ros::spin();
    };

    //雷达数据处理
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);

private:

    ros::NodeHandle nh;
    ros::Publisher point_pub;  
    ros::Publisher point2_pub;  
    ros::Subscriber scan_sub;
    ros::Publisher posepub;
    std::string map_frame;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
};


void scan2points::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{ 

    
   if(!listener.waitForTransform(
        map_frame,
        scan_in->header.frame_id,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
        return;
    }
    projector.transformLaserScanToPointCloud(map_frame,*scan_in,cloud,listener);
    ROS_INFO("PUSI");

    point_pub.publish(cloud);

}


int main(int argc,char**argv){
    
    ros::init(argc, argv, "scan2points");
    ros::NodeHandle nh;
    std::string map_frame;

    ros::param::get("~/map_frame",map_frame);

    // ros::param::get("~base_frame",base_frame);
    // nh.getParam("map_frame",map_frame);
    cout<<map_frame<<endl;

    // const char* scan_topic = ros::param::get("~/")

    scan2points Scan(nh,map_frame);

    // ros::spinOnce();//调用一次scancallback
    // ros::spin(); //死循环一直调用直到退出程序
    return 0;
}