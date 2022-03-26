#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<tf/transform_listener.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<string>

#define depth_change 0.3    //深度变化阀值
#define depth_max 30.0      //最大深度探测范围
#define succession 10       //连续点数目
#define kf 1.0              //距离权重
#define kg 1.0              //交集权重
#define minsize 5           //一类物体最小包含点云数


/*
    这个程序用于 将里程计转化为位姿信息，且将雷达数据转化为地图点云
    
    对应的话题名称为：   位置 /Pose  障碍物点云 /ObstaclePoints 预测点云 /PerceptPoints
                      世界坐标系  默认：/map
                      如果要修改则在 launch 文件中加入    <param name="/base_frame" value="你想要的frame"/>
*/



namespace Percept{

    typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;
    typedef std::vector<myPointCloud> Cluster;

    struct LaserData{
        int count;
        float   angles[360];
        float   ranges[360];
        LaserData(){
            count = 0;
        }
    };

    struct Objects{
        myPointCloud centroidcloud;
        myPointCloud maxpointcloud;
        myPointCloud minpointcloud;
        myPointCloud thetascloud;
        myPointCloud vel;
        std::vector<int> label;
        int maxlabel;
    };
    

    class Frame
    {
    private:
        ros::NodeHandle nh;
        ros::Subscriber lasersub;
        ros::Publisher velspub;
        ros::Publisher objectpub;
        tf::TransformListener tf_listener;
    public:
        int init;
        ros::Time lasttime;
        ros::Time thistime;
        std::string base_frame;
        Objects objects;
        ros::Rate rate;

        //初始化
        Frame(ros::NodeHandle &n,ros::Rate &r,std::string &base_f);
        void Init_Frame();
        
        //回调
        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void Testcallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        //数据清洗
        LaserData* Dataloader(const sensor_msgs::LaserScan::ConstPtr& scan);

        //聚类
        Cluster Clusting(Percept::LaserData* laserdata);
        
        //获得质心
        void GetCentroid_and_Theta(Percept::Cluster & cluster,Percept::Objects &objects);
        
        //匹配
        void Compare(Percept::Objects& objectstmp);
        
        //可视化
        void Visualization();
    };


}


Percept::Frame::Frame(ros::NodeHandle &n,ros::Rate &r,std::string &base_f):nh(n),rate(r),base_frame(base_f)
{
    Init_Frame();
}


void Percept::Frame::Init_Frame(){
    
    init=1;
    lasersub   = this->nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&Percept::Frame::ScanCallback,this);
    
    //后面把物体和速度整合成一个数据包
    objectpub  = this->nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/Objects",10);
    velspub    = this->nh.advertise<sensor_msgs::PointCloud2>("ObjectsVel",10);
    
    ros::spin();
}

// void Percept::Frame::Testcallback(const sensor_msgs::LaserScan::ConstPtr& scan){

//     Percept::LaserData* laserdata = Dataloader(scan);
//     //聚类并转化为点云发送
//     Percept::Cluster cluster= this->Clusting(laserdata);
//     //计算质心
//     ROS_INFO("the num of classes = %d", cluster.size());
//     Percept::myPointCloud centroids;
//     Percept::myPointCloud thetas;
//     GetCentroid_and_Theta(cluster,centroids,thetas);
//     //匹配并发送速度
//     //更新数据
//     this->centroids = centroids;
//     this->thetas = thetas;
//     jsk_recognition_msgs::BoundingBoxArray boxarray;
//     for(size_t i = 0;i<cluster.size();i++){
//         jsk_recognition_msgs::BoundingBox box;
//         box.header.frame_id = scan->header.frame_id;
//         box.pose.position.x = centroids[i].x;
//         box.pose.position.y = centroids[i].y;
//         box.pose.position.z = 1.5;
//         box.dimensions.x = maxpoint[i].x - minpoint[i].x;
//         box.dimensions.y = maxpoint[i].y - minpoint[i].y;
//         box.dimensions.z = 1.5; 
//         boxarray.boxes.push_back(box);
//     }
//     boxarray.header.frame_id = scan->header.frame_id;
//     boxarray.header.stamp = scan->header.stamp;
//     boxpub.publish(boxarray);
//     rate.sleep();
//     this->init=0;
// }

void Percept::Frame::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    if(init){
        this->lasttime=this->thistime=scan->header.stamp;

        LaserData* laserdata = Dataloader(scan);
        //聚类并转化为点云发送
        Cluster cluster= Clusting(laserdata);
        //计算质心

        GetCentroid_and_Theta(cluster,objects);
        int i;
        for(i =1;i<objects.centroidcloud.size();i++){
            objects.label.push_back(i);
        }
        objects.maxlabel = i-1;
        //更新数据
        init=0;
        return;
    }
    
    thistime=scan->header.stamp;
    
    //数据清洗
    Percept::LaserData* laserdata = Dataloader(scan);
    
    //聚类并转化为点云发送
    Percept::Cluster cluster= Clusting(laserdata);
    
    //计算质心
    Percept::Objects objectstmp;
    GetCentroid_and_Theta(cluster,objectstmp);
    
    //匹配并发送速度
    Compare(objectstmp);
    
    //更新数据
    objects = objectstmp;
    lasttime = thistime;

    //可视化
    Visualization();

}

//数据清理
Percept::LaserData* Percept::Frame::Dataloader(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("Loading the scan data");
    Percept::LaserData* laserdata = (Percept::LaserData*)malloc(sizeof(Percept::LaserData));
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("the num of laserscans = %d",count);
    int j=0;
    for (int i =0; i < count; i++)
    {
        if (scan->ranges[i]>depth_max||scan->ranges[i]==std::numeric_limits<float>::infinity()){
            continue;
        }
        laserdata->angles[j] = scan->angle_min+scan->angle_increment*i;
        laserdata->ranges[j] = scan->ranges[i];
        j++;
    }
    laserdata->count  = j;
    ROS_INFO("After shower the num of scan = %d",j);
    return laserdata;
}

//聚类
Percept::Cluster Percept::Frame::Clusting(Percept::LaserData* laserdata)
{
    bool flag;
    Percept::Cluster cluster;
    Percept::myPointCloud point2ds;   
    pcl::PointXYZ point2d;
    point2d.x =  laserdata->ranges[0]*cosf(laserdata->angles[0]);
    point2d.y =  laserdata->ranges[0]*sinf(laserdata->angles[0]); //以rad输入
    point2d.z = 0;
    point2ds.push_back(point2d);
    for (int i = 1; i < laserdata->count; i++)
    {
        flag = 1;
        if (abs(laserdata->ranges[i]-laserdata->ranges[i-1])>depth_change)
        {
            flag=0;
            for (int j = i+1; j < i+succession; j++)
                if (abs(laserdata->ranges[j]-laserdata->ranges[i-1])<=depth_change)
                {
                    flag = 1;
                    break;
                }
        }
        point2d.x =  laserdata->ranges[i]*cosf(laserdata->angles[i]);
        point2d.y =  laserdata->ranges[i]*sinf(laserdata->angles[i]);
        if(flag){
            point2ds.push_back(point2d);
        }
        else{
            if(point2ds.size()<=minsize){
                point2ds.clear();
            }
            else{
                cluster.push_back(point2ds);
                point2ds.clear();
                point2ds.push_back(point2d);
            }
        }
    }

    return cluster;
}

//计算质心，置信区间，生成障碍物
void Percept::Frame::GetCentroid_and_Theta(Percept::Cluster & cluster,Percept::Objects &objects){

    for(auto pointcloud:cluster){

        pcl::PointXYZ centroid,theta;
        pcl::PointXYZ maxp,minp;
        float maxx = 0,minx = 0;
        float maxy = 0,miny = 0;
        float sum_x = 0.0, sum_y = 0.0;
        float num = 0.0;
        for(auto point:pointcloud){
            if(point.x>maxx) maxx = point.x;
            if(point.x<minx) minx = point.x;
            if(point.y>maxy) maxy = point.y;
            if(point.y<miny) miny = point.y;
            sum_x+=point.x;
            sum_y+=point.y;
            num++;
        }

        maxp.x = maxx;
        maxp.y = maxy;
        minp.x = minx;
        minp.y = miny;
        objects.maxpointcloud.push_back(maxp);
        objects.minpointcloud.push_back(minp);
        
        centroid.x=sum_x/num;
        centroid.y=sum_y/num;
        centroid.z=0;

        objects.centroidcloud.push_back(centroid);
        
        float sum_x2 = 0.0, sum_y2 = 0.0;
        for(auto point:pointcloud){
            sum_x2+=pow((point.x-centroid.x),2);
            sum_y2+=pow((point.y-centroid.y),2);
        }
        theta.x=sqrt(sum_x2/(num*(num-1)));
        theta.y=sqrt(sum_y2/(num*(num-1)));
        theta.z=0;
        objects.thetascloud.push_back(theta);
    }
}


//求交集边界
template<typename T>
void MaxMin(double &xb,T x1 ,T x2 , T t1,T t2)
{
    double s1,s2,s3,s4;
    s1 = x1+t1;
    s2 = x2+t2;
    if (s2>s1){
        s4 = s2;
        s2 = x2-t2;
        if (s2>=s1){
            xb =0;
            return;
        }
        else{
            s3=s1;
            s1 = x1 - t1;
            if (s1>s2)
            {
                s2 = s1;
                s1 = x2-t2;
            }
            xb = s3-s2;
        }
    }
    else{
        s4 = s1;
        s1 = x1-t1;
        if (s1>=s2) {
            xb =0;
            return;
        }
        else{
            s3=s2;
            s2 = x2 - t2;
            if (s1>s2)
            {
                s2 = s1;
                s1 = x2-t2;
            }
            xb = s3-s2;
        }
    }
}

//IOU
template<typename T>
double IoU(T &c1 ,T &t1, T &c2, T &t2){
    double xb,yb;
    MaxMin(xb,c1.x,c2.x,t1.x,t2.x);
    MaxMin(yb,c1.y,c2.y,t1.y,t2.y);

    double I = xb * yb;
    double U = (2*t1.x*2*t1.y+2*t2.x*2*t2.y)-xb*yb;
    ROS_INFO("x1 = %f , x2 = %f , t1 = %f , t2 = %f",c1.x,c2.x,t1.x,t2.x);
    ROS_INFO("xb = %lf, yb = %lf",xb,yb);
    ROS_INFO("I = %lf , U = %lf , I/U = %lf",I,U,I/U);
    return I/U;
}

//匹配发送速度
void Percept::Frame::Compare(Percept::Objects &objectstmp){
    
    double dt = (thistime-lasttime).toSec();
    objectstmp.maxlabel = objects.maxlabel;
    for(int i=0;i<objectstmp.centroidcloud.size();i++){

        double compare = -1;
        double F=0,G=0;
        int index = 0,label =0;
        for(int j=0;j<objects.centroidcloud.size();j++){
            double x2 = std::pow((objectstmp.centroidcloud.points[i].x-objects.centroidcloud.points[j].x),2);
            double y2 = std::pow((objectstmp.centroidcloud.points[i].y-objects.centroidcloud.points[j].y),2);
            F = sqrt(x2+y2);
            G = 1.0 - IoU(objectstmp.centroidcloud.points[i],
                                objectstmp.thetascloud.points[i],
                                objects.centroidcloud.points[j],
                                objects.thetascloud.points[j]);
            ROS_INFO("F = %lf , G = %lf",F,G);
            double C = kf/F + kg/G;
            ROS_INFO("C = %lf",C);
            if(C>compare){
                compare=C;
                index = j;
                ROS_INFO("%d",j);

                label=objects.label[j];
                ROS_INFO("%d",label);
            }
        }
        if(compare>100){
            pcl::PointXYZ vel;
            vel.x = 0;
            vel.y = 0;
            objectstmp.vel.push_back(vel);
            objectstmp.label.push_back(label);
            ROS_INFO("static");
        
        }
        else if(G<0.6){
            pcl::PointXYZ vel;
            vel.x = (objectstmp.centroidcloud.points[i].x - objects.centroidcloud.points[index].x)/dt;
            vel.y = (objectstmp.centroidcloud.points[i].y - objects.centroidcloud.points[index].y)/dt; 
            objectstmp.vel.push_back(vel);
            objectstmp.label.push_back(label);
            ROS_INFO("move");

        }
        else{
            objectstmp.maxlabel++;
            objectstmp.label.push_back(objectstmp.maxlabel);
        }
    }

}


void Percept::Frame::Visualization(){
    
    jsk_recognition_msgs::BoundingBoxArray boxarray;
    for (size_t i = 0; i < objects.centroidcloud.size(); i++)
    {
        jsk_recognition_msgs::BoundingBox box;
        box.header.frame_id = base_frame;
        box.header.stamp = thistime;
        box.label = objects.label[i];
        box.pose.position.x = objects.centroidcloud.points[i].x;
        box.pose.position.y = objects.centroidcloud.points[i].y;
        box.pose.position.z = 1;

        box.dimensions.x = objects.maxpointcloud.points[i].x - objects.minpointcloud.points[i].x;
        box.dimensions.y = objects.maxpointcloud.points[i].y - objects.minpointcloud.points[i].y;
        box.dimensions.z = 0.5;

        boxarray.boxes.push_back(box);
    }
    boxarray.header.stamp = thistime;
    boxarray.header.frame_id = base_frame;
    objectpub.publish(boxarray);
    rate.sleep();
    
}

int main(int argc , char **argv){
    ros::init(argc,argv,"Frame");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    std::string base_frame;
    nh.param<std::string>("base_frame",base_frame,"/map");
    ROS_INFO("%s",base_frame.c_str());
    Percept::Frame frame(nh,rate,base_frame);

    return 0;
}