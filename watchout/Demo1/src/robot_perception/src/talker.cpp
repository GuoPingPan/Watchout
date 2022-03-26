#include<ros/ros.h>
#include<std_msgs/String.h>
#include<string>

using namespace std;

int main(int argc,char ** argv){
    ros::init(argc,argv,"talker");
    ros::NodeHandle n1("n1");
    ros::NodeHandle n2("n2");
    ros::Publisher pub1 = n1.advertise<std_msgs::String>("/helloworld1",10);
    ros::Publisher pub2 = n2.advertise<std_msgs::String>("/helloworld2",10);
    ros::Rate rate(10);
    int i = 0;
    while (ros::ok())
    {
        std_msgs::String msg1,msg2;

        std::stringstream s1,s2;
        s1 << "hello world " << ros::Time::now();
        msg1.data = s1.str();
        s2 << "hello world " << "2";
        // msg2.data = s2.str(  
        pub1.publish(msg1);
        pub2.publish(msg2);
        std::cout<< msg1.data.c_str()<<std::endl;
        std::cout<< msg2.data.c_str()<<std::endl;
        rate.sleep();
        i++;
    }
    
}
