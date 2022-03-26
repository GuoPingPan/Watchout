#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include<vector>

int main(int argc,char **argv){
    ros::init(argc,argv,"mapshow");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid",1,true);
    
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "grid";
    map.info.height = 5;
    map.info.width = 5;
    map.info.resolution = 0.05;
    
    int p[map.info.height*map.info.width] ={-1};
    p[(10-1)*map.info.width+10-1] = 100;
    std::vector<signed char> a(p,p+400);
    map.data = a;
    map.info.origin.position.x=10;
    while (ros::ok())
    {
        map.header.stamp = ros::Time::now();
        map_pub.publish(map);
        rate.sleep();
    }
    
    return 0;
}