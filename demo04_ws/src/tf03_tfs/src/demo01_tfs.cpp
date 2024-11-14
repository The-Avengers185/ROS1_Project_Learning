#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
/* 
    订阅方实现: 1. 计算son1和son2的相对关系 2.计算son1某个坐标在son2中的坐标值

    流程:
        1.包含头文件;
        2.编码、初始化、NodeHandle;
        3.创建订阅对象;
        4.编写解析逻辑;
        5.spinOnce();
*/

int main(int argc, char *argv[])
{
    // 2.编码、初始化、NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub");
    ros::NodeHandle nh;
    // 3.创建订阅对象;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    
    // 4.编写解析逻辑;
    ros::Rate rate(10);
    while (ros::ok())
    {
        // 核心   
        try
        {
            //1.计算son1和son2的相对关系

            //2.计算son1的中某个坐标点在son2中的坐标值
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系:父坐标系ID=%s",tfs.header.frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:子坐标系ID=%s",tfs.child_frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z
                    );

            // 坐标点解析
            geometry_msgs::PointStamped ps;
            ps.header.frame_id = "son1";
            ps.header.stamp = ros::Time::now();
            ps.point.x = 1.0;
            ps.point.y = 2.0;
            ps.point.z = 3.0;

            geometry_msgs::PointStamped psAtSon2;
            psAtSon2 = buffer.transform(ps,"son2");
            ROS_INFO("在 Son2 中的坐标:x=%.2f,y=%.2f,z=%.2f",
                    psAtSon2.point.x,
                    psAtSon2.point.y,
                    psAtSon2.point.z
                    );
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());
        }

        rate.sleep();
        ros::spinOnce();
    }
    // 5.spinOnce();
    return 0;
}
