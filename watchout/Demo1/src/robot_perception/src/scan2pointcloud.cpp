#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<laser_geometry/laser_geometry.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<string>
#include<sensor_msgs/PointCloud2.h>
// #include<pcl/filter/passthrough.h>
class scan2points{
public:
    
    scan2points(ros::NodeHandle &n,std::string &base_frame_):nh(n),base_frame(base_frame_){
        
        point_pub=nh.advertise<sensor_msgs::PointCloud2>("/PointClouds",10);

        scan_sub=nh.subscribe("/scan",10,&scan2points::scanCallback,this);
        
        ros::spin();
    };
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);

private:
    ros::NodeHandle nh;
    ros::Publisher point_pub;  
    ros::Subscriber scan_sub;
    std::string base_frame;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
};


void scan2points::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{ 

    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud();
    // pass.setFilterFieldName("x","y");
    // pass.setFilterLimits();

    
    if(!this->listener.waitForTransform(
        scan_in->header.frame_id,
        this->base_frame,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)
    )){
        return;
    }

    this->projector.transformLaserScanToPointCloud(
        this->base_frame,
        *scan_in,
        this->cloud,
        listener);
    this->point_pub.publish(cloud);
    ROS_INFO("pubing the clouds!");

  // Do something with cloud.
}




int main(int argc,char**argv){
    
    ros::init(argc, argv, "scan2points");
    ros::NodeHandle nh;

    std::string base_frame;
    // ros::param::get("~base_frame",base_frame);
    ros::param::param<std::string>("base_frame", base_frame, "base_footprint");

    // const char* scan_topic = ros::param::get("~/")

    scan2points Scan(nh,base_frame);

    // ros::spinOnce();//调用一次scancallback
    // ros::spin(); //死循环一直调用直到退出程序
    return 0;
}
