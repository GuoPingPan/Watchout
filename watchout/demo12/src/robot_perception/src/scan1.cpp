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
    scan2points(ros::NodeHandle &n,std::string &base_frame_):nh(n),base_frame(base_frame_){
        
        point_pub=nh.advertise<sensor_msgs::PointCloud2>("/PointClouds",10);
        // posepub = nh.advertise<geometry_msgs::Transform>("/odom",10);
        scan_sub=nh.subscribe("/scan",10,&scan2points::scanCallback,this);
        ros::Rate rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        // ros::spin();
    };

    //雷达数据处理
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);

private:

    ros::NodeHandle nh;
    ros::Publisher point_pub;  
    ros::Subscriber scan_sub;
    ros::Publisher posepub;
    std::string base_frame;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
};


void scan2points::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{ 

    // tf::StampedTransform transform;
    
    if(!this->listener.waitForTransform(
        base_frame,
        scan_in->header.frame_id,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)
    )){
        
        // this->listener.lookupTransform(this->base_frame,
        // scan_in->header.frame_id,
        // scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        // transform);
        return;
    }



    this->projector.transformLaserScanToPointCloud(
        this->base_frame,
        *scan_in,
        this->cloud,
        listener);
    this->point_pub.publish(cloud);
    ROS_INFO("pubing the clouds!");
    
    geometry_msgs::Transform trans;
    // trans.rotation.x = transform.getRotation().getX();
    // trans.rotation.y = transform.getRotation().getY();
    // trans.rotation.z = transform.getRotation().getZ();
    // trans.rotation.w = transform.getRotation().getW();
    // trans.translation.x = transform.getOrigin().getX();
    // trans.translation.y = transform.getOrigin().getY();
    // trans.translation.z = transform.getOrigin().getZ();
    
    // posepub.publish(trans);


}


int main(int argc,char**argv){
    
    ros::init(argc, argv, "scan2points");
    ros::NodeHandle nh;

    std::string base_frame;
    // ros::param::get("~base_frame",base_frame);
    ros::param::param<std::string>("base_frame", base_frame, "/base_footprint");

    cout<<base_frame<<endl;
    // const char* scan_topic = ros::param::get("~/")

    scan2points Scan(nh,base_frame);

    // ros::spinOnce();//调用一次scancallback
    // ros::spin(); //死循环一直调用直到退出程序
    return 0;
}