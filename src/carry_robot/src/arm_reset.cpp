#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_reset_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    // 只保留需要的服务客户端：关闭气泵 + 机械臂归零
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/control_center/release_service");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/control_center/zero_service");

    std_srvs::Empty empty_srv;

    // 第一步：关闭气泵
    arm_release_client.call(empty_srv);
    sleep(1.0);

    // 第二步：机械臂归零复位
    arm_zero_client.call(empty_srv);

    ros::shutdown();
    return 0;
}