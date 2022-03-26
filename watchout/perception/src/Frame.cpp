#include"demo1/Frame.h"

Percept::Frame::Frame(ros::NodeHandle &n,ros::Rate &r):nh(n),rate(r)
{
    Init_frame();
}

void Percept::Frame::Init_frame(){
    this->init=1;
    this->pointpub = this->nh.advertise<sensor_msgs::myPointCloud>("/myPointCloud",10);
    this->centroidpub=this->nh.advertise<sensor_msgs::myPointCloud>("/Centroid",10);
    this->velspub= this->nh.advertise<geometry_msgs::Twist>("Objectvel",10);
    this->lasersub=this->nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&Percept::Frame::ScanCallback,this);
    ros::spin();
}

void Percept::Frame::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   
    if(init){
        this->lasttime=this->thistime=scan->header.stamp;

        Percept::LaserData* laserdata = Dataloader(scan);
        //聚类并转化为点云发送
        Percept::Cluster cluster= this->Clustering(laserdata); 
        //计算质心

        Percept::myPointCloud centroids;
        Percept::myPointCloud thetas;
        GetCentroid_and_Theta(cluster,centroids,thetas);
        //匹配并发送速度
        //更新数据
        this->centroids = centroids;
        this->thetas = thetas;
        this->init=0;
    }
    this->thistime=scan->header.stamp;
    //数据清洗
    Percept::LaserData* laserdata = Dataloader(scan);
    //聚类并转化为点云发送
    Percept::Cluster cluster= this->Clustering(laserdata); 
    //计算质心
    Percept::myPointCloud centroids;
    Percept::myPointCloud thetas;
    GetCentroid_and_Theta(cluster,centroids,thetas);
    //匹配并发送速度
    this->Compare(centroids,thetas);
    //更新数据
    this->centroids = centroids;
    this->thetas = thetas;
    this->lasttime=this->thistime;


}

//数据清理
Percept::LaserData* Dataloader(const sensor_msgs::LaserScan::ConstPtr& scan)
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
    pcl::PointCloud<pcl::PointXYZ> pointclouds;
    Percept::myPointCloud point2ds;   //自己的
    pcl::PointXYZ point2d;
    point2d.x =  laserdata->ranges[0]*cosf(laserdata->angles[0]);
    point2d.y = -laserdata->ranges[0]*sinf(laserdata->angles[0]); //以rad输入
    point2d.z = 0;
    point2ds.push_back(point2d);
    pointclouds.push_back(point2d);
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
            pointclouds.push_back(point2d);
        }
        else{
            if(point2ds.size()<=1){
                point2ds.clear();
                pointclouds.push_back(point2d);
            }
            else{
                cluster.push_back(point2ds);
                point2ds.clear();
                point2ds.push_back(point2d);
                pointclouds.push_back(point2d);
            }
        }
    }

    sensor_msgs::myPointCloud pointcloud;
    pcl::toROSMsg (pointclouds,pointcloud);
    this->pointpub.publish(pointcloud);

    return cluster;
}

//计算质心，置信区间，生成障碍物
void GetCentroid_and_Theta(Percept::Cluster & cluster,Percept::myPointCloud & centroids,Percept::myPointCloud & thetas){

    for(auto points:cluster){

        pcl::PointXYZ centroid,theta;
        float sum_x = 0.0, sum_y = 0.0;
        float num = 0.0;
        for(auto point:points){
            sum_x+=point.x;
            sum_y+=point_y;
            num++;
        }
        centroid.x=sum_x/num;
        centroid.y=sum_y/num;
        centroid.z=0;
        centroids.push_back(centroid);
        
        float sum_x2 = 0.0, sum_y2 = 0.0;
        for(auto point:points){
            sum_x2+=pow((point.x-centroid.x),2);
            sum_y2+=pow((point.y-centroid.y),2);
        }
        theta.x=sqrt(sum_x2/(num*(num-1)));
        theta.y=sqrt(sum_y2/(num*(num-1)));
        theta.z=0;
        thetas.push_back(theta);
    }
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
        
void Percept::Frame::Compare(myPointCloud &centroids_,myPointCloud &thetas_){
    for(int i=0;i<centroids_.size();i++){
        for(int j=0;j<this->centroids.size();j++){
            double x2 = std::pow((centroids_.points[i].x-this->centroids.points[j].x),2);
            double y2 = std::pow((centroids_.points[i].y-this->centroids.points[j].y),2);
            double F = sqrt(x2+y2);
            double G = 1.0 - IoU(centroids_.points[i],this->centroids.points[j],thetas_.points[i],this->thetas.points[j]);
            ROS_INFO("F = %lf , G = %lf",F,G);
            // if(F<0.0001) F = 0.0001;
            // if(G<0.0001) G = 0.0001;
            double C = kf/F + kg/G;
            ROS_INFO("C = %lf",C);
            //下面是计算并发布速度
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



