#include <stdio.h>
#include <string.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>	//数据格式
#include <serial/serial.h>		//ros串口

#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <fstream>
#include <iostream>
using namespace std;

serial::Serial ser;			//定义一个串口对象
ros::Subscriber position_sub;		//订阅位姿节点

typedef union{				//定义一个联合体，用于double数据与16进制的转换
unsigned char cvalue[12];
float vel[3];
}vel_xyz;




static uint8_t s_buffer[14];	//分配静态存储空间

void position_world_callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO_STREAM("----------------------------------------------");

	vel_xyz vel;
	memset(s_buffer,0,sizeof(s_buffer));		//内存空间初始化为0
	
	vel.vel[0]  =   0.1;
	vel.vel[1]  =   0;
	vel.vel[2]  =   0;
	
	// vel.vel[0]  =   cmd_vel.linear.x;
	// vel.vel[1]  =   cmd_vel.linear.y;
	// vel.vel[2]  =   cmd_vel.linear.z;

	//数据打包
	s_buffer[0] = 0x0f;		//数据的帧头
	s_buffer[1] =  vel.cvalue[0];
	s_buffer[2] =  vel.cvalue[1];
	s_buffer[3] =  vel.cvalue[2];
	s_buffer[4] =  vel.cvalue[3];
	 
	s_buffer[5] =  vel.cvalue[4];
	s_buffer[6] =  vel.cvalue[5];
	s_buffer[7] =  vel.cvalue[6];
	s_buffer[8] =  vel.cvalue[7];
	 
	s_buffer[9]  =  vel.cvalue[8];
	s_buffer[10] =  vel.cvalue[9];
	s_buffer[11] =  vel.cvalue[10];
	s_buffer[12] =  vel.cvalue[11];
	s_buffer[13] =  0xff;
    
	// //CRC		校验和从有效数据开始取异或即可
	// s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^\
	// 				s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];
	ser.write(s_buffer,14);
	
	ROS_INFO("data send successful");
	
}





int main(int argc, char** argv)
{

	
	ros::init(argc, argv, "serial_node");		//ROS串口节点名称
	ros::NodeHandle my_node;
	
	position_sub = my_node.subscribe("/cmd_vel", 1, position_world_callback);
	
	//读取串口名称
	// string filename = "/home/lmf/WORK_SPACE/catkin_ws/src/realsense-ros/realsense2_camera/parameter.txt";
	// ifstream fin( filename.c_str() );
	// if (!fin)
    //     {
    //         cout<<"parameter file does not exist."<<endl;
    //         return 0;
    //     }
	// string serialPort;
	// getline( fin, serialPort );
	// fin.close();
	// cout<<serialPort<<endl;
	try
	{
	//设置串口属性，并打开串口
        string serialPort="serial";
    	ser.setPort(serialPort);	//设置对应串口的名称
		ser.setBaudrate(115200);		//波特率115200
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
	}
	catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
	
	
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }
    else
    {
        return -1;
    }
	
	
	//指定循环的速率 
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		

		ros::spinOnce();
		
		loop_rate.sleep();
	}
    ser.close();
	return 0;
}


