#include "ros/ros.h"

/*
    演示参数查询
    实现：
        ros::NodeHandle --------------------------------------------------------
            param(键,默认值) 
                存在，返回对应结果，否则返回默认值

            getParam(键,存储结果的变量)
                存在,返回 true,且将值赋值给参数2
                若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamCached键,存储结果的变量)--提高变量获取效率
                存在,返回 true,且将值赋值给参数2
                若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamNames(std::vector<std::string>)
                获取所有的键,并存储在参数 vector 中 

            hasParam(键)
                是否包含某个键，存在返回 true，否则返回 false

            searchParam(参数1，参数2)
                搜索键，参数1是被搜索的键，参数2存储搜索结果的变量

        ros::param -------------------------------------------------------------
*/

//如果没有在CmakeList中提前加入demo02，main就会报错
int main(int argc, char *argv[])
{
    //设置编码
    setlocale(LC_ALL,"");

    //初始化ros节点
    ros::init(argc,argv,"get_param_c");

    //创建句柄
    ros::NodeHandle nh;

    //ros::NodeHandle ---------------------------------------------------------
    // 1.param
    double radius = nh.param("radius",0.5);
    ROS_INFO("radius = %.2f", radius);

    // 2.getParam
    double radius2 = 0.0;
    // bool result = nh.getParam("radiusxxxx",radius2);
    // 3.getParamCached 与 getParam 类似，只是性能上有提升，一般测试下看不出
    bool result = nh.getParamCached("radiusxxxx",radius2);
    if (result)
    {
        ROS_INFO("获取的半径是：%.2f",radius2);
    }
    else
    {
        ROS_INFO("被查询的变量不存在");
    }

    // 4.getParamNames
    std::vector<std::string> names;
    // std::vector 看作是一个可以动态增减大小的数组
    // 表示一个存储 std::string 类型元素的动态数组。
    nh.getParamNames(names);
    for (auto &&name : names)
    // for (auto &&变量 : 容器) { /* 使用变量 */ }
    // auto 会自动推导出变量的类型，而 && 是右值引用，通常用于高效地处理元素。
    // for (auto &&name : names) 会遍历 names 容器中的每一个元素，并将当前元素赋给 name。
    {
        ROS_INFO("遍历的元素：%s",name.c_str());
    }

    // 5.hasParam
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("radiusxxx");
    ROS_INFO("radius存在吗? %d",flag1);
    ROS_INFO("radiusxxx 存在吗？%d",flag2);

    // 6.searchParam
    std::string key;
    nh.searchParam("radiusxxx",key);
    ROS_INFO("搜索的结果: %s",key.c_str());

    // ros::param -------------------------------------------------------------
    double radius_param = ros::param::param("radiuss",100.5);
    ROS_INFO("radius_param = %.2f",radius_param);

    std::vector<std::string> names_param;
    ros::param::getParamNames(names_param);
    for (auto &&name : names_param)
    {
        ROS_INFO("键：%s",name.c_str());
    }



    return 0;
}
