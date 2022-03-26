#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

using namespace std;

int main(int argc,char **argv){
    ros::init(argc,argv,"marker");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("markerarray",10);
    ros::Rate rate(10);
    while(ros::ok()){
        visualization_msgs::MarkerArray makerarrary;
        int k =0;
        while(k<1){
            visualization_msgs::Marker marker;
            marker.header.frame_id="/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id =k;

            marker.scale.z = 0.5;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.color.b = k*2;
            marker.color.g = k*2;
            marker.color.r = k*2;
            marker.color.a = 1;

            geometry_msgs::Pose pose;
            pose.position.x =  (float)(k)/10+0.5;
            pose.position.y =  0.5;
            pose.position.z =  0.5;
            marker.pose=pose;

            makerarrary.markers.push_back(marker);
            k++;
        }
        cout<<"markerArray.markers.size()"<<makerarrary.markers.size()<<endl;
        pub.publish(makerarrary);
        rate.sleep();
            
    }
}