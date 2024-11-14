//1.包含ros的头文件
#include "ros/ros.h"

//2.编写main函数
int main(int argc,char *argv[]){
//3.初始化ros节点
ros::init(argc,argv,"hello_ws2");
//4.输出日志
ROS_INFO("demo02_ws");
return 0;
}
