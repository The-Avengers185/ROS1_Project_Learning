#include "ros/ros.h"

int main(int argc, char *argv[])
{
    //解决乱码问题
    // setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"hello_c");
    ROS_INFO("HELLO，哈哈哈");

    return 0;
}
