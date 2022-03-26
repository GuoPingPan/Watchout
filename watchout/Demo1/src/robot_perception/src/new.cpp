#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>


class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Publisher m_globalcloudPub;  //发布局部地图点云
  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;  //接收点云
  tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
  tf::TransformListener m_tfListener;  // 转化坐标系

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
      
      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh,"/seg_kitti_velo/pointcloud", 100);    //接收rosbag中的点云消息
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, "/world", 100);  //接收tf和点云之后触发接收  world是frameid
      m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));   //回调函数
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/global_map", 2, true);   //发布全局地图，用于rviz展示
  }

  ~SegMapROSWraper()
  {
      delete m_pointCloudSub;
      delete m_tfPointCloudSub;
  }
  


  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
  {
      pcl::PointCloud<pcl::PointXYZL> pc;
      pcl::PointCloud<pcl::PointXYZL> pc_global;
      pcl::fromROSMsg(*cloud, pc);

      tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
      try
      {
          // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
          m_tfListener.lookupTransform("/world", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
      }

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
      pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = "/map"; 
      m_globalcloudPub .publish(map_cloud);  //加上时间戳和frameid发布出来
  }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "colored"); 

  SegMapROSWraper  SM;

  ros::spin();
  return 0;
}

