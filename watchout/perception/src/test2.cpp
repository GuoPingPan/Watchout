#include"ros/ros.h"
#include"sensor_msgs/LaserScan.h"
#include"std_msgs/Time.h"
#include"std_msgs/Header.h"
#include"std_msgs/String.h"
#include"Eigen/Core"
#include<vector>
#include<cmath>
#include"pcl/point_cloud.h"
#include"pcl/point_types.h"


pcl::PointCloud<pcl::PointXYZ> cloud;



#define RAD2DEG(x) ((x)*180.0/M_PI)

#define num_of_laser    360*8
#define depth_max 4
#define depth_change 1
#define succession 20   //连续30个点
#define za_2 1.96





namespace My
{
    typedef struct Point2d
    {
        float x;
        float y;
        Point2d()
        {
            x = 0;
            y = 0;
        }
    };

    typedef std::vector<Point2d>    Point2ds;
    typedef std::vector<Point2ds>    Cluster;
    
    typedef struct TwoFrame{
        ros::Time lastime;
        ros::Time thistime;
        std::string frame_id;
        Point2ds lastcentroids;
        Point2ds thiscentroids;
        Point2ds lastthetas;
        Point2ds thisthetas;
        Cluster lastcluster;
        Cluster thiscluster;
        int lastclasses;
        int thisclasses;
        void update();
    };

    typedef struct LaserData
    {
        int count;  
        float   scan_time;
        float   angle_min;
        float   angle_max;
        float   range_min;
        float   range_max;
        float   angles[num_of_laser];
        float   ranges[num_of_laser];
    };
        
}



// typedef struct StaticObsticle
// {
//     float x_r;
//     float y_r;
// };

// typedef struct DynamicObsticle
// {
//     float x_r;
//     float y_r;
//     float v_x;
//     float v_y;
// };

static bool first = 1;


My::TwoFrame Frame;

void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan){
    if(first){
        first = 0;
        My::LaserData* laserdata;
        laserdata = dataloader(scan);
        ros::
        Frame.lastime = scan->header.stamp;
        Frame.frame_id  = scan->header.frame_id;
        
        Frame.lastcluster = clusting(laserdata);
        Frame.lastclasses = Frame.lastcluster.size();

        for (My::Cluster::iterator it = Frame.lastcluster.begin(); it != Frame.lastcluster.end(); it++)
        {
            My::Point2d centroid;
            My::Point2d theta;
            GetCentroid_and_Theta(it,centroid,theta);
            Frame.lastcentroids.push_back(centroid);
            Frame.lastthetas.push_back(theta);
        }
    }
    else{
        My::LaserData* laserdata;
        laserdata = dataloader(scan);
        
        Frame.thistime = scan->header.stamp;

        Frame.thiscluster = clusting(laserdata);
        Frame.thisclasses = Frame.thiscluster.size();

        for (My::Cluster::iterator it = Frame.thiscluster.begin(); it!= Frame.thiscluster.end(); it++)
        {
            My::Point2d centroid;
            My::Point2d theta;
            GetCentroid_and_Theta(it,centroid,theta);
            Frame.thiscentroids.push_back(centroid);
            Frame.thisthetas.push_back(theta);
        }

        // void Compare();
        // Frame.update();

    }

}

void GetCentroid_and_Theta(My::Cluster::iterator & it,My::Point2d & centroid,My::Point2d & theta){
    float sum_x = 0.0, sum_y = 0.0;
    float num = 0.0;
    for(My::Point2ds::iterator vit = (*it).begin();vit! = (*it).end();it++ ){
        sum_x+=(*vit).x;
        sum_y+=(*vit).y;
        num++;
    }

    centroid.x = sum_x/num;
    centroid.y = sum_y/num;
    
    float sum_x2 = 0.0, sum_y2 = 0.0;
    for(My::Point2ds::iterator vit = (*it).begin();vit! = (*it).end();it++ ){
        sum_x2+=std::pow(((*vit).x-centroid.x),2);
        sum_y2+=std::pow(((*vit).y-centroid.y),2);
    }   
    theta.x = sqrt(sum_x2/((num-1)*num));
    theta.y = sqrt(sum_y2/((num-1)*num));
}

// sensor_msgs::LaserScan laserdata;

//数据清理
My::LaserData* dataloader(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("Loading the scan data");
    My::LaserData* laserdata = (My::LaserData*)malloc(sizeof(My::LaserData)); 
    laserdata->angle_min=scan->angle_min;
    laserdata->angle_max=scan->angle_max;
    laserdata->scan_time=scan->scan_time;
    laserdata->range_min=scan->range_min;
    laserdata->range_max=scan->range_max;
    int i=0,j=0;
    while(scan->ranges[i]==std::numeric_limits<float>::infinity()) i++;
    for (; i < num_of_laser; i++)
    {
        if (scan->ranges[i]>depth_max||scan->ranges[i]==std::numeric_limits<float>::infinity())
            continue;
        laserdata->angles[j] = laserdata->angle_min+scan->angle_increment*i;
        laserdata->ranges[j] = scan->ranges[i];
        j++;        
    }
    laserdata->count  = j;
    return laserdata;
}



//聚类
My::Cluster clusting(My::LaserData* laserdata)
{
    bool flag;
    My::Cluster cluster;
    My::Point2ds point2ds;
    My::Point2d point2d;
    point2d.x =laserdata->ranges[0]*sin(RAD2DEG(laserdater->angle[0]));
    point2d.y =laserdata->ranges[0]*sin(RAD2DEG(laserdater->angle[0])); 
    point2ds.push_back(point2d);
    for (int i = 1; i < laserdata->count; i++)
    {
        flag = 1;
        if (abs(laserdata->ranges[i]-laserdata->ranges[i-1])>depth_change)
        {
            flag=0;
            for (int j = i+1; j < i+n; j++)
                if (abs(laserdata->ranges[j]-laserdata->ranges[i-1])<=depth_change)
                {
                    flag = 1;
                    break;
                } 
        }
        if(flag){
            point2d.x = laserdata->ranges[0]*sin(RAD2DEG(laserdater->angle[i]));
            point2d.y = laserdata->ranges[0]*sin(RAD2DEG(laserdater->angle[i])); 
            point2ds.push_back(point2d);
        }
        else{
            cluster.push_back(point2ds);
            My::Point2Ds().swap(point2ds);
        }
    }
    return cluster; 
}

//计算质心，置信区间，生成障碍物


//帧间匹配




int main(int argc,char **argv){
    ros::init(argc,argv,"laser_test_dynamic");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,dataloader);

    ros::spin();

    return 0;
}