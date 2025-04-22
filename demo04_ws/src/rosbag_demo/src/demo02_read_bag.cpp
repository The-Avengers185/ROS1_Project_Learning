#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
/*
    需求: 使用 rosbag 读取磁盘上的文件
    流程:
        1.导包
        2.初始化
        3.创建 rosbag 对象
        4.打开文件流（以读得方式打开）
        5.读数据
        6.关闭文件流
*/

int main(int argc, char *argv[])
{
    // 2.初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;
    // 3.创建 rosbag 对象
    rosbag::Bag bag;
    // 4.打开文件流（以读得方式打开）
    bag.open("hello.bag",rosbag::BagMode::Read);
    // 5.读数据
    // 取出话题 时间戳和消息内容
    // 可以先获取消息集合，再迭代取出消息的字段
    for (auto &&m : rosbag::View(bag))
    {
        // 解析
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();
        std_msgs::StringConstPtr p = m.instantiate<std_msgs::String>();
        ROS_INFO("解析的内容,话题:%s,时间戳:%.2f,消息值:%s",
                topic.c_str(),
                time.toSec(),
                p->data.c_str());
    }
    
    

    // 6.关闭文件流
    bag.close();
    return 0;
}

