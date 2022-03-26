#include<ros/ros.h>
#include<std_msgs/Time.h>
#include<vector>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Twist.h>


namespace Percept{


    // struct Point2d
    // {
    //     float x;
    //     float y;
    //     float z;
    //     Point2d{
    //         x = 0;
    //         y = 0;
    //         z = 0;
    //     }
    // };
    
    struct LaserData{
        int count;  
        float   angles[360];
        float   ranges[360];
        LaserData(){
            count = 0;
        }
    }; 

    typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;
    typedef vector<myPointCloud> Cluster;


    class Frame
    {
    private:
        ros::NodeHandle nh;
        ros::Publisher pointpub;
        ros::Subscriber lasersub;
        ros::Publisher centroidpub;
        ros::Publisher velspub;
        myPointCloud centroids;
        myPointCloud thetas;
    public:
        int init;
        ros::Time lasttime;
        ros::Time thistime;
        ros::Rate rate;
        Frame(ros::NodeHandle &n,ros::Rate &r);
        void Init_frame();
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void Compare(myPointCloud &centroids_,myPointCloud &thetas_);
        Cluster Clusting(LaserData* laserdata);
    };



   
}

