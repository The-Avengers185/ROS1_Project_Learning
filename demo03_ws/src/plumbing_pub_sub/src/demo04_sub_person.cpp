#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"
/*
    订阅方：订阅人的信息
        1.包含头文件；
            #include "plumbing_pub_sub/Person.h"
        2.初始化ROS节点；
        3.创建节点句柄；
        4.创建订阅者对象；
        5.处理订阅数据；
        6.调用spin（）函数。

*/
void doPerson(const plumbing_pub_sub::Person::ConstPtr& person)
{
    ROS_INFO("订阅人的信息：%s,%d,%.2f",person->name.c_str(),person->age,person->height);
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ROS_INFO("订阅方实现");
    // 2.初始化ROS节点；
    ros::init(argc,argv,"jiazhang");
    // 3.创建节点句柄；
    ros::NodeHandle nh;
    // 4.创建订阅者对象；
    ros::Subscriber sub=nh.subscribe("liaotian",10,doPerson);
    // 5.处理订阅数据；

    // 6.调用spin（）函数。
    ros::spin();
    
    return 0; 
}
