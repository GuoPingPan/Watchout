## 感知Demo1

### 下载依赖

>1.ROS操作系统，[点击链接安装](https://www.it610.com/article/1288233309515591680.htm)

>2.pcl库（含pcl_ros）, 一般在安装ros-$ROS_DISTRO-desktop-full的时候自带，不建议自己再装一个pcl，寻找路径会出错

>3.安装jsg_recognition_msgs (用于可视化) 
`sudo apt-get install ros-melodic-jsk-recognition-msgs`
    
    
### 代码
 - 将雷达数据转化为世界坐标系 Scan2point.cpp
 - 将雷达数据聚类并发布boundingboxs Frame.cpp
 - 欧几里得聚类（注意修改 `euclid.setMaxClusterSize` 和 `euclid.setMinClusterSize`）euclid_cluster.cpp
    
### 使用
    
1.创建ros工作空间
```    
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```
  
2.创建功能包(直接用现成的包时不需要此操作)
```
cd catkin_ws/src
示例：catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg robot_perception pcl_ros pcl_conversions tf rospy roscpp sensor_msgs laser_geometry
(后续可在CMakelists.txt 和 package.xml 中 添加/修改 依赖)
```    
3.将下载的包放到src中
```
cd catkin_ws/src
git clone 
cd ..
catkin_make --pkg package_name
```
4.使用（**注意roslaunch顺序**）
    
```
开启小车底盘(方法私戳)
roslaunch base_control base_control.launch

roslaunch robot_perception robot_nav.launch
roslaunch rplidar_ros rplidar.launch
```

    
    