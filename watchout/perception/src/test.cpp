#include"ros/ros.h"
#include"sensor_msgs/LaserScan.h"
#include"std_msgs/Time.h"
#include"std_msgs/Header.h"
#include"std_msgs/String.h"
#include"Eigen/Core"
#include<vector>
#include<cmath>
#include<pangolin/pangolin.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>

#define depth_max 4     //最大深度
#define depth_change 0.20   //20cm深度差
#define succession 10   //连续10个点
#define za_2 2.447   //上2a分位点    95%置信
#define kf 2
#define kg 2
// #define RAD2DEG(x) ((x)*180./M_PI)

namespace My
{
    struct Point2d{
        float x;
        float y;
        Point2d(){
            x = 0;
            y = 0;
        }
    };

    struct DynamicObstacle{
        float x;
        float y;
        double vx;
        double vy;
        DynamicObstacle(){
            x = 0;
            y = 0;
            vx = 0;
            vy = 0;
        }
    };

    typedef std::vector<Point2d>    Point2ds;
    typedef std::vector<Point2ds>    Cluster;
    typedef std::vector<DynamicObstacle> Dynamics;
    typedef std::vector<Point2d> Statics;

    struct TwoFrame{
        ros::Time lastime;
        ros::Time thistime;
        std::string frame_id;
        Point2ds lastcentroids;
        Point2ds thiscentroids;
        Point2ds lastthetas;
        Point2ds thisthetas;
        Cluster lastcluster;
        Cluster thiscluster;
        Dynamics dynamics;
        Statics statics;
        int lastclasses;
        int thisclasses;
    };


        
}

//数据清洗
My::LaserData* dataloader(const sensor_msgs::LaserScan::ConstPtr& scan);

//计算质心
void GetCentroid_and_Theta(My::Cluster & cluster,My::Point2ds & centroids,My::Point2ds & thetas);

//聚类
My::Cluster clusting(My::LaserData* laserdata);

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr GenerationCloud(My::Cluster::iterator &it,int color);

// void Visualization(My::Cluster &cluster ,My::Point2ds &centroids);

void showPointCloud(My::Point2ds &points);


template<typename T>
void MaxMin(auto &xb,T x1 ,T x2 , T t1,T t2);

template<typename T>
double IoU(T &c1 ,T &c2, T &t1, T &t2);

//帧间匹配
void Compare(My::TwoFrame &frame);

void Updata(My::TwoFrame &frame);

//初始状态
static bool first = 1;

//帧
My::TwoFrame Frame;

void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan){
    if(first){
        first = 0;
        //加载数据
        My::LaserData* laserdata;
        laserdata = dataloader(scan);
        
        //时间和id
        Frame.lastime = scan->header.stamp;
        Frame.frame_id  = scan->header.frame_id;
        
        //聚类
        Frame.lastcluster = clusting(laserdata);
        Frame.lastclasses = Frame.lastcluster.size();

        //计算质心
        GetCentroid_and_Theta(Frame.lastcluster,Frame.lastcentroids,Frame.lastthetas);

    }
    else{
        //加载数据
        My::LaserData* laserdata;
        laserdata = dataloader(scan);
        
        //时间
        Frame.thistime = scan->header.stamp;

        //当前帧聚类
        Frame.thiscluster = clusting(laserdata);
        Frame.thisclasses = Frame.thiscluster.size();

        //当前帧质心
        GetCentroid_and_Theta(Frame.thiscluster,Frame.thiscentroids,Frame.thisthetas);
        
        ROS_INFO("There are %d classes.",Frame.thisclasses);
        for (My::Point2ds::iterator it =Frame.thiscentroids.begin(),tit = Frame.thisthetas.begin(); it!=Frame.thiscentroids.end() ;it++,tit++)
        {
            ROS_INFO("the centroid is [%.4f,%.4f]",(*it).x,(*it).y);
            ROS_INFO("the   theta  is [%.4f,%.4f]",(*tit).x,(*tit).y);
        }
        // for(auto it:Frame.thiscluster){
        //     ROS_INFO("the centroid is [%.4f,%.4f]",(*it).x,(*it).y);
        //     ROS_INFO("the   theta  is [%.4f,%.4f]",(*tit).x,(*tit).y);
        // }
        //匹配计算速度
        Compare(Frame);
        ROS_INFO("There are %d dynamic obstacle!!!",Frame.dynamics.size());
        for (My::Dynamics::iterator it = Frame.dynamics.begin(); it!= Frame.dynamics.end(); it++)
        {
            ROS_INFO("the centroid of dynamic is [%.4f,%.4f]",(*it).x,(*it).y);
            ROS_INFO("the   speed  of dynamic is [%.4f,%.4f]",(*it).vx,(*it).vy);

        }
        
        // for (My::Cluster::iterator it = Frame.thiscluster.begin(); it!=Frame.thiscluster.end();it++)
        // {
        //     showPointCloud((*it));
        // }
        

        Updata(Frame);
        


        //更新帧数据
        // Frame.update();

    }

}

void TestCallback(const sensor_msgs::LaserScanConstPtr &scan){
    
    My::LaserData* laserdata;
    laserdata = dataloader(scan);
    ROS_INFO(" the angle_min: %.4f",laserdata->angle_min);
    ROS_INFO(" the angle_max: %.4f",laserdata->angle_max);
    ROS_INFO(" the range_max: %.4f",laserdata->range_max);
    ROS_INFO(" the range_min: %.4f",laserdata->range_min);
    for (size_t i = 0; i < laserdata->count; i++)
    {
        ROS_INFO(" [%.4f,%.4f] ",laserdata->ranges[i],laserdata->angles[i]);

    }
    ROS_INFO(" there are %d points after showerdata!",laserdata->count);
    My::Cluster cluster = clusting(laserdata);
    ROS_INFO("\n There are %d classes",cluster.size());
    My::Point2ds centroids;
    My::Point2ds theta;
    GetCentroid_and_Theta(cluster,centroids,theta);
    for (My::Point2ds::iterator it  = centroids.begin(),it2 = theta.begin();it!=centroids.end(); it++,it2++)
    {
        ROS_INFO("\n the average x = %f , the theta x = %f \
        \n the average y = %f , the theta y = %f",(*it).x,(*it2).x,(*it).y,(*it2).y);
        
    }
    // ROS_INFO(" the num of averages = %d",centroids.size());
    
}

//帧间匹配

int main(int argc,char **argv){
    ros::init(argc,argv,"laser_test_dynamic");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,ScanCallback);

    ros::spin();

    return 0;
}



template<typename T>
void MaxMin(auto &xb,T x1 ,T x2 , T t1,T t2)
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

template<typename T>
double IoU(T &c1 ,T &c2, T &t1, T &t2){
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

// #define FPS2Sec 0.1

void Compare(My::TwoFrame & frame){
    for (My::Point2ds::iterator lastc = frame.lastcentroids.begin(),lastt = frame.lastthetas.begin(); lastc != frame.lastcentroids.end();lastc++,lastt++)
    {
        for (My::Point2ds::iterator thisc = frame.thiscentroids.begin(),thist = frame.thisthetas.begin(); thisc != frame.thiscentroids.end(); thisc++,thist++)
        {
            double x2 = std::pow(((*lastc).x-(*thisc).x),2);
            double y2 = std::pow(((*lastc).y-(*thisc).y),2);
            double F = sqrt(x2+y2);
            double G = 1.0 - IoU((*lastc),(*thisc),(*lastt),(*thist));
            ROS_INFO("F = %lf , G = %lf",F,G);
            // if(F<0.0001) F = 0.0001;
            // if(G<0.0001) G = 0.0001;
            double C = kf/F + kg/G;
            ROS_INFO("C = %lf",C);
            if (C>100)
            {
                My::Point2d static_one;
                static_one.x = (*thisc).x;
                static_one.y = (*thisc).y;
                frame.statics.push_back(static_one);
            }
            else if(C>10)
            {
                My::DynamicObstacle dynamic;
                double time=frame.thistime.toSec() - frame.lastime.toSec();     //伏笔
                ROS_INFO("time = %lf",time);
                dynamic.x = (*thisc).x;
                dynamic.y = (*thisc).y;
                dynamic.vx = ((*thisc).x - (*lastc).x)/time;
                dynamic.vy = ((*thisc).y - (*lastc).y)/time;
                frame.dynamics.push_back(dynamic);
            }
            
        }
        
    }
    
}

void Updata(My::TwoFrame &frame){
    frame.lastthetas.swap(frame.thisthetas);
    frame.lastcentroids.swap(frame.thiscentroids);
    frame.lastclasses = frame.thisclasses;
    frame.lastime = frame.thistime;
    frame.lastcluster.swap(frame.thiscluster);
    My::Point2ds().swap(frame.thisthetas);
    My::Point2ds().swap(frame.thiscentroids);
    My::Cluster().swap(frame.thiscluster);
    My::Dynamics().swap(frame.dynamics);
    My::Statics().swap(frame.statics);
}

// void Visualization(My::Cluster &cluster ,My::Point2ds &centroids){
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds;
//     pcl::visualization::CloudViewer viewer ("viewer");
//     // viewer.setBackgroundColor(0.05,0.05,0.05,0);
//     // viewer.addCoordinateSystem(1.0,"cloud",0);
//     int color = 255;
//     for(My::Cluster::iterator it=cluster.begin();it!=cluster.end();it++){
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud1=GenerationCloud(it, color);
//         *clouds+=*subcloud1;
//         color-=10;
//     }
//     // pcl::io::savePCDFileBinary("pcl/navigation.pcd",*clouds); // save the map
//     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new  pcl::PointCloud<pcl::PointXYZRGB>());
//     // pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
//     // voxel.setInputCloud(clouds);
//     // voxel.filter(*tmp);
//     // clouds->swap(*tmp);
//     viewer.showCloud(clouds);
//     std::cout<<"show global map, size=" << clouds->size()<<endl;
// }

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr GenerationCloud(My::Cluster::iterator &it,int color){
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     for(My::Point2ds::iterator vit = (*it).begin();vit != (*it).end();vit++ ){
//        pcl::PointXYZRGB point;
//        point.x = (*vit).x; point.y = (*vit).y ;point.z = 1;
//        point.r = color; point.g = color; point.b = color;
//        subcloud->points.push_back(point);
//    }
//    return subcloud;
// }

void showPointCloud(My::Point2ds &points) {

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: points) {
            // glColor3f(p[3], p[3], p[3]);
            glVertex3d(p.x, p.y, 1);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

/**************************************************/

//数据清理
My::LaserData* dataloader(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("Loading the scan data");
    My::LaserData* laserdata = (My::LaserData*)malloc(sizeof(My::LaserData));
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("COUNT = %d",count);
    laserdata->angle_min=scan->angle_min;
    laserdata->angle_max=scan->angle_max;
    laserdata->scan_time=scan->scan_time;
    laserdata->range_min=scan->range_min;
    laserdata->range_max=scan->range_max;
    int j=0;
    for (int i =0; i < count; i++)
    {
        if (scan->ranges[i]>depth_max||scan->ranges[i]==std::numeric_limits<float>::infinity()){
            continue;
        }
        laserdata->angles[j] = laserdata->angle_min+scan->angle_increment*i;
        laserdata->ranges[j] = scan->ranges[i];
        j++;
    }
    laserdata->count  = j;
    ROS_INFO("After Shower COUNT = %d",j);
    return laserdata;
}

//聚类
My::Cluster clusting(My::LaserData* laserdata)
{
    bool flag;
    My::Cluster cluster;
    My::Point2ds point2ds;
    My::Point2d point2d;
    point2d.x =  laserdata->ranges[0]*cosf(laserdata->angles[0]);
    point2d.y = -laserdata->ranges[0]*sinf(laserdata->angles[0]); //以rad输入
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
        point2d.y = -laserdata->ranges[i]*sinf(laserdata->angles[i]); 
        if(flag){
            point2ds.push_back(point2d);
        }
        else{
            if(point2ds.size()<=1){
                My::Point2ds().swap(point2ds);
            }
            else{
                cluster.push_back(point2ds);
                My::Point2ds().swap(point2ds);
                point2ds.push_back(point2d);
            }
        }
    }
    return cluster;
}

//计算质心，置信区间，生成障碍物
void GetCentroid_and_Theta(My::Cluster & cluster,My::Point2ds & centroids,My::Point2ds & thetas){
    for(My::Cluster::iterator it=cluster.begin();it!=cluster.end();it++){
        float sum_x = 0.0, sum_y = 0.0;
        float num = 0.0;
        My::Point2d centroid,theta;
        for(My::Point2ds::iterator vit = (*it).begin();vit != (*it).end();vit++ ){
            sum_x+=(*vit).x;
            sum_y+=(*vit).y;
            num++;
        }

        centroid.x = sum_x/num;
        centroid.y = sum_y/num;
        centroids.push_back(centroid);

        float sum_x2 = 0.0, sum_y2 = 0.0;
        for(My::Point2ds::iterator vit = (*it).begin();vit!= (*it).end();vit++ ){
            sum_x2+=(float)pow(((*vit).x-centroid.x),2);
            sum_y2+=(float)pow(((*vit).y-centroid.y),2);
        }
        // if(sum_x2<=0.01) sum_x2 = 0.01*num*(num-1);
        // if(sum_y2<=0.01) sum_y2 = 0.01*num*(num-1);
        ROS_INFO("sum_x2 = %f,sum_y2 = %f, num = %f",sum_x2,sum_y2,num);
        ROS_INFO("tx = %f , ty = %f", sqrt(sum_x2/((num-1)*num)),sqrt(sum_y2/((num-1)*num)));
        theta.x = sqrt(sum_x2/((num-1)*num));
        theta.y = sqrt(sum_y2/((num-1)*num));
        thetas.push_back(theta);
    }
}

