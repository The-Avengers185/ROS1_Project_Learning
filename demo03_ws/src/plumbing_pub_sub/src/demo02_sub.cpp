#include "ros/ros.h"
#include "std_msgs/String.h"

/*
    订阅方实现：
        1.包含头文件；
          ROS中文本类型 ---> std_msgs/String.h
        2.初始化ROS节点；
        3.创建节点句柄；
        4.创建订阅者对象；
        5.处理订阅到的数据。
        6.spin()函数
*/

//传入消息：：string类型：：常量指针引用
void doMsg(const std_msgs::String::ConstPtr &msg){
    //通过msg获取并操作订阅到的数据
     ROS_INFO("翠花订阅的数据：%s",msg->data.c_str());
}



// 什么是回调函数？
// 每订阅一条数据都要执行他，与一般函数最大区别是：
// 一般函数定义了之后，什么时候被调用由自己掌控，需要就用；
// 而回调函数不一定被调用，等到外部控制在使用。

// 以打鬼子为例
// 即一般函数就像子弹，打鬼子的时候等鬼子来了调用这个函数，出发这个子弹打鬼子，时机可控制；
// 回调函数则像地雷，等小鬼子踩到地雷上才能爆炸，时机不能控制。

int main(int argc, char  *argv[])
{
    //设置编码，没有的话中文会乱码
    setlocale(LC_ALL,"");

    // 2.初始化ROS节点；
    ros::init(argc,argv,"cuiHua");

    // 3.创建节点句柄；
    ros::NodeHandle nh;

    // 4.创建订阅者对象；
    // 泛型不用添加，会自动根据回调函数推
    // 话题名称+队列长度+回调函数
    ros::Subscriber sub = nh.subscribe("fang",10,doMsg);

    // 5.处理订阅到的数据。

    // 6.设置循环调用回调函数
    // 循环读取接受数据，并调用回调函数处理
    ros::spin();

    return 0;
}
