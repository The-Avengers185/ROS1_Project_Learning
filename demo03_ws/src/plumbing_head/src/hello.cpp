#include "ros/ros.h"
#include "/home/alan/demo03_ws/src/plumbing_head/include/hello.h"

namespace hello_ns
{
    void MyHello::run()
    {
        ROS_INFO("run函数执行.....");
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"hello_head");
    hello_ns::MyHello myHello;
    myHello.run();
    
    return 0;
}
