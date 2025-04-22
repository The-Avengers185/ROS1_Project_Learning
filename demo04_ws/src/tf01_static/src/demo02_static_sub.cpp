/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.编码、初始化、NodeHandle
        3.创建订阅对象 ---> 订阅坐标系相对关系
        4.组织一个坐标点数据(相对于子级坐标系)
        5.转换坐标点，需要调用TF内置实现(相对于父级坐标系)
        6.spin()
*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;
    // 3.创建订阅对象 ---> 订阅坐标系相对关系
    // 3.1. 创建 Buffer 缓存
    tf2_ros::Buffer buffer;
    // 3.2. 再创建监听对象（监听对象可以将订阅数据存入buffer）
    tf2_ros::TransformListener listener(buffer);
    
    // 4.生成一个坐标点(相对于子级坐标系)
    geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "laser";
    point_laser.header.stamp = ros::Time::now();
    point_laser.point.x = 2.0;
    point_laser.point.y = 3.0;
    point_laser.point.z = 5.0;
    // 添加休眠
    ros::Duration(2).sleep();
    // 5.转换坐标点(相对于父级坐标系)
    ros::Rate rate(10);
    while (ros::ok())
    {
        // 核心代码 --- 将 point_laser 转换成相对于 base_link 的坐标点

        // 新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            /*
                调用了buffer的转换函数transform
                参数1: 被转换的目标点
                参数2: 目标坐标系
                返回值： 输出坐标点

                PS1: 调用时必须要用头文件 tf2_geometry_msg/tf2_geometry_msgs.h
                PS2: 运行时存在问题，抛出异常 base_link 不存在
                     原因: 订阅数据是一个耗时操作，可能在调用 transform 转换函数时，
                           坐标系的相对关系还没有订阅到，因此出现异常
                     解决: 
                        方案1: 在调用转换函数前执行休眠
                        方案2: 进行异常处理 (建议)
            */
            point_base = buffer.transform(point_laser,"base_link");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:",point_base.point.x,point_base.point.y,point_base.point.z,point_base.header.frame_id.c_str());

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }

        rate.sleep();  
        ros::spinOnce();
    }


    return 0;
}