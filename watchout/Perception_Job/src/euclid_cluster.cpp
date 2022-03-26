#include<ros/ros.h>
#include<string>
#include<iostream>
#include<vector>

#include<sensor_msgs/LaserScan.h>
#include<laser_geometry/laser_geometry.h>


#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/Transform.h>

#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include<pcl_ros/transforms.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>

/*  聚类头文件  */
#include<pcl/search/kdtree.h>
#include<pcl/segmentation/euclidean_cluster_comparator.h>
#include<pcl/segmentation/extract_clusters.h>

/*  可视化头文件  */
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>

using namespace std;

/*  此程序实现 pcl的欧几里得聚类算法并完成可视化

*/


//物体
struct object{
    jsk_recognition_msgs::BoundingBox bounding;
    pcl::PointXYZ minpoint;
    pcl::PointXYZ maxpoint;
    // pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointXYZ centroid;
    object(){
        bounding.header.frame_id = "/map";
        centroid.x=0;
        centroid.y=0;
        centroid.z=0;
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
    void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr point_in ,float max_distance,std::vector<object> &object_list);

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



void scan2points::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr point_in ,float max_distance,std::vector<object> &object_list){
    
    //建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //复制点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*point_in,*point_cloud );
    std::vector<pcl::PointIndices> local_indices;
    
    if(point_cloud->size()>0)
        tree->setInputCloud(point_cloud);

    //欧几里德聚类
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(point_cloud);
    euclid.setClusterTolerance(max_distance);
    euclid.setMaxClusterSize(100000000);
    euclid.setMinClusterSize(10);
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
        objectone.maxpoint.z = 3;
        objectone.minpoint.z = 0;
        objectone.centroid.z = 1.5;
        objectone.bounding.pose.position.x = objectone.centroid.x;
        objectone.bounding.pose.position.y = objectone.centroid.y;
        objectone.bounding.pose.position.z = objectone.centroid.z;
        objectone.bounding.dimensions.x = (objectone.maxpoint.x-objectone.minpoint.x)/2;
        objectone.bounding.dimensions.y = (objectone.maxpoint.y-objectone.minpoint.y)/2;
        objectone.bounding.dimensions.z = objectone.centroid.z;
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

    //雷达转化成点云并去除距离大于30的点，并发布
    projector.transformLaserScanToPointCloud(map_frame,*scan_in,cloud,listener,30.0,3);
    point2_pub.publish(cloud);


    //pc2 - pclt
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::fromROSMsg(cloud,points);
    
    //分区聚类，阈值不同
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pcl_arry(2);
    float threshold[2]={0.5,1};

    //初始化指针空间
    for(size_t i = 0;i<segment_pcl_arry.size();i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pcl_arry[i] = tmp;
    }

    for(size_t i= 0;i<points.size();i++){
        pcl::PointXYZ point;
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = 0;

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
    
    scan2points Scan(nh,map_frame);

    // ros::spinOnce();//调用一次scancallback
    // ros::spin(); //死循环一直调用直到退出程序
    return 0;
}