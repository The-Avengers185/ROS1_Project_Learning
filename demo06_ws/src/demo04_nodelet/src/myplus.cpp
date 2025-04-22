#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"

/*
    需求:首先,需要订阅一个浮点数据;然后，将订阅数据与参数服务器的指定参数相加;最后，将最终结果发布。
    流程:
        1.先确定需要的变量: 订阅对象,发布对象,存储参数的变量;
        2.获取 NodeHandle;
        3.通过 NodeHandle 创建订阅对象和发布对象,解析参数;
        4.回调函数需要处理数据，并通过发布对象发布。
*/
namespace my_nodelet{
    class MyPlus: public nodelet::Nodelet {
        
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            double value;

        public: 
            MyPlus(){
                value = 0.0;
            };
            void onInit(){
                ROS_INFO("hello nodelet -----------------------------------------");
                // 2.获取 NodeHandle; 获取私有的
                ros::NodeHandle nh = getPrivateNodeHandle();
                // 3.通过 NodeHandle 创建订阅对象和发布对象,解析参数;
                nh.getParam("value",value);
                pub = nh.advertise<std_msgs::Float64>("out",100); //话题名称: /节点名/in
                sub = nh.subscribe<std_msgs::Float64>("in",100,&MyPlus::cb,this);
                // 4.回调函数需要处理数据，并通过发布对象发布。
            }

            //处理订阅的回调函数
            void cb(const std_msgs::Float64::ConstPtr& p){
                //4-1.解析订阅数据
                double in = p->data;
                //4-2.和参数相加
                double out = in + value;
                //4-3.发布
                std_msgs::Float64 out_msgs;
                out_msgs.data = out;
                pub.publish(out_msgs);
            }
    };
};

PLUGINLIB_EXPORT_CLASS(my_nodelet::MyPlus,nodelet::Nodelet)