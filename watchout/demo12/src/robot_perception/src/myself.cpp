#include<ros/ros.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<string>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<vector>

#define depth_change 0.3
#define succession 10

class Frame{
public:
    Frame(ros::NodeHandle nh,std::string frame):n(nh),map_frame(frame){
        init=1;
        lasersub = n.subscribe("/scan",1,&Frame::callback,this);
        pointpub = n.advertise<sensor_msgs::PointCloud2>("/PointClouds",1);
        bboxpub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/Boxx",1);
        ros::spin();
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &scan);
    std::string map_frame;

private:
    int init;
    ros::NodeHandle n;
    ros::Subscriber lasersub;
    ros::Publisher pointpub;
    ros::Publisher bboxpub;
    tf::TransformListener tflistener;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cluster;
};


void Clustering(pcl::PointCloud<pcl::PointXYZ> &datashower,std::vector<pcl::PointCloud<pcl::PointXYZ>> &cluster){

    bool flag;
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::PointXYZ point;
    point.x =  datashower.points[0].x*cosf(datashower.points[0].y);
    point.y =  datashower.points[0].x*sinf(datashower.points[0].y); //以rad输入
    point.z = 0;
    pointcloud.push_back(point);
    for (int i = 1; i <datashower.size(); i++)
    {
        flag = 1;
        if (abs(datashower.points[i].x-datashower.points[i-1].x)>depth_change)
        {
            flag=0;
            for (int j = i+1; j < i+succession; j++)
                if (abs(datashower.points[j].x-datashower.points[i-1].x)<depth_change)
                {
                    flag = 1;
                    break;
                }
        }
        point.x = datashower.points[i].x*cosf(datashower.points[i].y);
        point.y = datashower.points[i].x*sinf(datashower.points[i].y);
        if(flag){
            pointcloud.push_back(point);
        }
        else{
            if(pointcloud.size()<=1){
                pointcloud.clear();
                pointcloud.push_back(point);
            }
            else{
                cluster.push_back(pointcloud);
                pointcloud.clear();
                pointcloud.push_back(point);
            }
        }


}



void Frame::callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    pcl::PointCloud<pcl::PointXYZ> datashower;
    int count = scan->scan_time/scan->time_increment;
    for(size_t i = 0;i<count;i++){
        pcl::PointXYZ point;
        if(scan->ranges[i]>30)
            continue;
        else{
            point.x = scan->ranges[i];
            point.y = scan->angle_min+scan->angle_increment*i;
            datashower.push_back(point);
        }
    }
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(datashower,pc2);
    pointpub.publish(pc2);
    //聚类
    if(init){
        Clustering(datashower,cluster);

    }


}
