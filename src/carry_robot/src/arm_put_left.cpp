#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "put_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    ros::ServiceClient arm_move_close_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_release_client = nh.serviceClient<std_srvs::Empty>("/control_center/release_service");
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/control_center/zero_service");

    // 放置的目标点，单位毫米，以机械臂基座上自转轴为基点，x左正右负，y前正后负，z上正下负
    int target_put_x = 220;//yyx
    int target_put_y = 130;
    int target_put_z = 0;
    ROS_INFO("target_put_x = %d,target_put_y = %d,target_put_z = %d",target_put_x,target_put_y,target_put_z);
    upros_message::ArmPosition move_srv;
    //第一步，运动到放置点-高
    move_srv.request.x = target_put_x + 50;
    move_srv.request.y = target_put_y;
    move_srv.request.z = target_put_z + 60;
    arm_move_close_client.call(move_srv);
    sleep(4.0);

    ros::shutdown();

    return 0;
}
