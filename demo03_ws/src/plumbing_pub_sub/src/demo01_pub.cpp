#include "ros/ros.h"
#include "std_msgs/String.h"//发布数据的头文件
#include  <sstream>//组织字符串

/*
    发布方实现：
        1.包含头文件；
          ROS中文本类型 ---> std_msgs/String.h
        2.初始化ROS节点；
        3.创建节点句柄；
        4.创建发布者对象；
        5.编写发布逻辑并发布数据。
*/

int main(int argc, char  *argv[])
{
    //输出有中文，避免乱码
    setlocale(LC_ALL,"");

    // 2.初始化ROS节点；
    //前两个是main里的参数，后面是节点名称，里面不能有空格，且不能重名
    ros::init(argc,argv,"erGouZi");

    // 3.创建节点句柄；
    ros::NodeHandle nh;

    // 4.创建发布者对象；
    //调用advertise函数，需要设置泛型，泛型即发布数据的类型
    //参数1是话题
    //参数2是消息队列，消息如果遇到网络阻塞没有发出去，就会放到队列里面，若超过阈值（如这里为10）
    //则先进的会销毁
    ros::Publisher pub = nh.advertise<std_msgs::String>("fang",10);

    // 5.编写发布逻辑并发布数据。
    //要求以10HZ的频率发布数据，并且文本后添加编号
    //先创建被发布消息
    std_msgs::String msg;

    //发布频率
    ros::Rate rate(1);

    //设置编号
    int count=0;

    //休眠功能，延迟第一条数据的发送
    ros::Duration(3).sleep();

    //编写循环，循环中发布数据
    while (ros::ok())//节点不死（死就是ctrl+c），条件即成立
    {
        count++;
        //实现字符串拼接数字
        //创建stringstream流，使用它拼接字符串和编号
        std::stringstream ss;
        ss <<"hello--->"<< count;

        //msg.data = "hello";
        //把data设置成流里的字符串
        msg.data = ss.str();
        pub.publish(msg);

        //添加日志输出，相当于printf
        ROS_INFO("发布的数据：%s",ss.str().c_str());

        //调用一下休眠，保证发布频率
        rate.sleep();

        //官方建议，处理回调函数
        ros::spinOnce();
    }
    
    return 0;
}
