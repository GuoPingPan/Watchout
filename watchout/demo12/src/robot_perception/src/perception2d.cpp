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
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<pcl/search/kdtree.h>
#include<pcl/segmentation/euclidean_cluster_comparator.h>
#include<pcl/segmentation/extract_clusters.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>

using namespace std;

//物体
struct object{
    jsk_recognition_msgs::BoundingBox bounding;
    pcl::PointXY minpoint;
    pcl::PointXY maxpoint;
    // pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointXY centroid;
    object(){
        bounding.header.frame_id = "/map";
        centroid.x=0;
        centroid.y=0;
    }
};


class scan2points{
public:
    
    //初始化
    scan2points(ros::NodeHandle &n,std::string &map_frame_):nh(n),map_frame(map_frame_){
        point2_pub=nh.advertise<sensor_msgs::PointCloud2>("/PointClouds",10);
        scan_sub=nh.subscribe("/scan",10,&scan2points::scanCallback,this);
        pub_bounding = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/Obstacles",10);
        pub_box1 = nh.advertise<jsk_recognition_msgs::BoundingBox>("/BOX",10);
        ros::spin();
    }

    //雷达数据处理
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
    void cluster_segment(pcl::PointCloud<pcl::PointXY>::Ptr point_in ,float max_distance,std::vector<object> &object_list);
private:

    ros::NodeHandle nh;
    ros::Publisher point2_pub;  
    ros::Publisher pub_bounding;
    ros::Publisher pub_box1;
    ros::Subscriber scan_sub;
    std::string map_frame;
    tf::TransformListener listener;
    laser_geometry::LaserProjection projector;
};



void scan2points::cluster_segment(pcl::PointCloud<pcl::PointXY>::Ptr point_in ,float max_distance,std::vector<object> &object_list){
    
    //建立kdtree
    pcl::search::KdTree<pcl::PointXY>::Ptr tree(new pcl::search::KdTree<pcl::PointXY>);

    //复制点云
    pcl::PointCloud<pcl::PointXY>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXY>);
    pcl::copyPointCloud(*point_in,*point_cloud );
    std::vector<pcl::PointIndices> local_indices;
    
    if(point_cloud->size()>0)
        tree->setInputCloud(point_cloud);


    //欧几里德聚类
    pcl::EuclideanClusterExtraction<pcl::PointXY> euclid;
    euclid.setInputCloud(point_cloud);
    euclid.setClusterTolerance(max_distance);
    euclid.setMaxClusterSize(10000);
    euclid.setMinClusterSize(20);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);
    //计算物体质心
    for(size_t i=0; i<local_indices.size();i++){
        object objectone;
        float minx = 0;
        float maxx = 0;
        float miny = 0;
        float maxy = 0;
        
        for(auto index=local_indices[i].indices.begin();index!=local_indices[i].indices.end();index++){
            pcl::PointXY point;
            point.x =point_cloud->points[*index].x;
            point.y = point_cloud->points[*index].y; 
            if(point.x>maxx)  maxx=point.x;
            if(point.x<minx)  minx=point.x;
            if(point.y>maxy)  maxy=point.y;
            if(point.x<miny)  miny=point.y;
            objectone.centroid.x += point.x;
            objectone.centroid.y += point.y; 
        }

        objectone.maxpoint.x = maxx;
        objectone.maxpoint.y = maxy;
        objectone.minpoint.x = minx;
        objectone.minpoint.y = miny;
        if(local_indices[i].indices.size()>0){
            objectone.centroid.x /= local_indices[i].indices.size();
            objectone.centroid.y /= local_indices[i].indices.size();
        }
        objectone.bounding.pose.position.x = objectone.centroid.x;
        objectone.bounding.pose.position.y = objectone.centroid.y;
        objectone.bounding.pose.position.z = 1.5;
        objectone.bounding.dimensions.x = (objectone.maxpoint.x-objectone.minpoint.x)/2;
        objectone.bounding.dimensions.y = (objectone.maxpoint.y-objectone.minpoint.y)/2;
        objectone.bounding.dimensions.z = 1.5;
        ROS_INFO("%s",objectone.bounding.header.frame_id);
        pub_box1.publish(objectone.bounding);
        object_list.push_back(objectone);
    }




}


void scan2points::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{ 
    sensor_msgs::PointCloud2 cloud;

    //等待坐标转换
    if(!listener.waitForTransform(
        map_frame,
        scan_in->header.frame_id,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
        return;
    }

    //雷达转化成点云并去除距离大于20的点，并发布
    projector.transformLaserScanToPointCloud(map_frame,*scan_in,cloud,listener,30.0,3);
    ROS_INFO("PUSI");
    point2_pub.publish(cloud);


    //pc2 - pclt
    pcl::PointCloud<pcl::PointXY> points;
    pcl::fromROSMsg(cloud,points);
    //分区聚类，阈值不同
    std::vector<pcl::PointCloud<pcl::PointXY>::Ptr> segment_pcl_arry(2);
    float threshold[2]={0.5,10};
    ROS_INFO("0");

    for(size_t i = 0;i<segment_pcl_arry.size();i++){
        pcl::PointCloud<pcl::PointXY>::Ptr tmp(new pcl::PointCloud<pcl::PointXY>);
        segment_pcl_arry[i] = tmp;
    }

    ROS_INFO("1");

    for(size_t i= 0;i<points.size();i++){
        pcl::PointXY point;
        point.x = points[i].x;
        point.y = points[i].y;

        float distance = sqrt(pow(point.x,2)+pow(point.y,2));
        if(distance>4){
            segment_pcl_arry[0]->points.push_back(point);
        }
        else{
            segment_pcl_arry[1]->points.push_back(point);
        }

    }

    std::vector<object> object_list;

    for(size_t i = 0;i<segment_pcl_arry.size();i++){
        cluster_segment(segment_pcl_arry[i],threshold[i],object_list);
    }

    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = map_frame;
    bbox_array.header.stamp = cloud.header.stamp;
    for(size_t i=0;i<object_list.size();i++){
        bbox_array.boxes.push_back(object_list[i].bounding);
    }
    pub_bounding.publish(bbox_array);
}


int main(int argc,char**argv){
    
    ros::init(argc, argv, "perception1");
    ros::NodeHandle nh;
    
    std::string map_frame;
    ros::param::get("~/map_frame",map_frame);
    cout<<map_frame<<endl;

    // ros::param::get("~base_frame",base_frame);
    // nh.getParam("map_frame",map_frame);

    scan2points Scan(nh,map_frame);

    // ros::spinOnce();//调用一次scancallback
    // ros::spin(); //死循环一直调用直到退出程序
    return 0;
}