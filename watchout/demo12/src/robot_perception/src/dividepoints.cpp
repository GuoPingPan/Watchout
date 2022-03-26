#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<sensor_msgs/point_cloud_conversion.h>
#include<iostream>
#include<string>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Transform.h>
#include<pcl_ros/transforms.h>


/*
    这个程序用于 将里程计转化为位姿信息，且将雷达数据转化为地图点云
    
    对应的话题名称为：   位置 /Pose  障碍物点云 /ObstaclePoints 预测点云 /PerceptPoints
                      世界坐标系  默认：/map
                      如果要修改则在 launch 文件中加入    <param name="/base_frame" value="你想要的frame"/>
*/


class MessageManager{
public:
    MessageManager(ros::NodeHandle &n,ros::Rate &r,std::string &base_name):nh(n),rate(r),base_frame(base_name){
        obstaclepub = nh.advertise<sensor_msgs::PointCloud2>("/ObstaclePoints",10);
        perceptpub = nh.advertise<sensor_msgs::PointCloud2>("/PerceptPoints",10);
        lasersub = nh.subscribe("/scan",10,&MessageManager::callback,this);
        posepub = nh.advertise<geometry_msgs::Transform>("/Pose",10);
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
    ros::Publisher posepub;
    tf::TransformListener listener;
};


void MessageManager::callback(const sensor_msgs::LaserScan::ConstPtr &scan){

    //雷达数据点
    int count = scan->scan_time/ scan->time_increment;
	
    //障碍物点云和预测点云
    sensor_msgs::PointCloud Obstaclepoints,Predictpoints;
    sensor_msgs::PointCloud Obstacleworld,Predictworld;
    Obstaclepoints.header.frame_id = scan->header.frame_id;
    Predictpoints.header.frame_id = scan->header.frame_id;
    Obstaclepoints.points.resize(count);    //这里是直接加入点而不是push_back，所以需要设置大小
    Predictpoints.points.resize(count);

    int p = 0;
    int o = 0;
    ROS_INFO("the num of laser points = %d",count );
    for(int i=0;i<count;i++){
        
        if(scan->ranges[i]==std::numeric_limits<float>::infinity()){
            continue;
        }                                                                //排除深度为无穷的数据


        else if(scan->ranges[i]>4){
            float angle = scan->angle_min+scan->angle_increment*i;
            Predictpoints.points[p].x = scan->ranges[i]*cosf(angle);    //深度 >4 的数据用于预测
            Predictpoints.points[p].y = scan->ranges[i]*sinf(angle);
            Predictpoints.points[p].z = 0;
            p++;
        }
        
        else{
            float angle = scan->angle_min+scan->angle_increment*i;
            Obstaclepoints.points[o].x = scan->ranges[i]*cosf(angle);   //深度 <4 的数据用于避障
            Obstaclepoints.points[o].y = scan->ranges[i]*sinf(angle);
            Obstaclepoints.points[o].z = 0;
            o++;
        }
        
    }

    //获取 位姿
    Eigen::Matrix4f trans1;
    tf::StampedTransform transform;
    try{

        listener.waitForTransform(base_frame,scan->header.frame_id,scan->header.stamp,ros::Duration(3.0));
        listener.lookupTransform(base_frame,scan->header.frame_id,scan->header.stamp,transform);
        
        geometry_msgs::Transform trans;
        trans.rotation.x = transform.getRotation().getX();
        trans.rotation.y = transform.getRotation().getY();
        trans.rotation.z = transform.getRotation().getZ();
        trans.rotation.w = transform.getRotation().getW();
        trans.translation.x = transform.getOrigin().getX();
        trans.translation.y = transform.getOrigin().getY();
        trans.translation.z = transform.getOrigin().getZ();
        
        posepub.publish(trans);

        listener.transformPointCloud(base_frame,Obstaclepoints,Obstacleworld);
        listener.transformPointCloud(base_frame,Predictpoints,Predictworld);
    
    }


    catch(tf::TransformException &ex){
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }
    
    // ROS_INFO("yes");
    
    sensor_msgs::PointCloud2 Obstaclepoint2,Perceptpoint2;
    
    sensor_msgs::convertPointCloudToPointCloud2(Obstacleworld,Obstaclepoint2);
    sensor_msgs::convertPointCloudToPointCloud2(Predictworld,Perceptpoint2);

    Obstaclepoint2.header.stamp = scan->header.stamp;
    // Obstaclepoint2.header.frame_id = "/base_footprint";
    Perceptpoint2.header.stamp = scan->header.stamp;
    // Perceptpoint2.header.frame_id = "/base_footprint";
    
    obstaclepub.publish(Obstaclepoint2);
    perceptpub.publish(Perceptpoint2);
}



int main(int argc,char **argv){
    ros::init(argc,argv,"percept2");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
	std::string base_frame;
    nh.param<std::string>("base_frame",base_frame,"/map");
    MessageManager manager(nh,rate,base_frame);
    return 0;
}
