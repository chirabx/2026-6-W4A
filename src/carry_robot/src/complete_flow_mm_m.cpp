#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include "std_srvs/Empty.h"

using namespace std;
int tag_id = -1;           // 初始化为 -1，表示未检测到任何 tag
bool tag_received = false; // 标志变量，表示是否接收到 tag_id

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sendGoal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw);
void move_safe(ros::Publisher &pub, double vx, double vy, int max_count);

// 安全移动封装函数
void move_safe(ros::Publisher &pub, double vx, double vy, int max_count)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = vx;
    vel_msg.linear.y = vy;
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok() && count < max_count)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw)
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ROS_INFO("Executing backward retry logic...");
    move_safe(pub, -0.05, 0.0, 30); // Backward 30 steps

    ROS_INFO("Retrying to move to target point (%.3f, %.3f, %.3f)", x, y, yaw);
    sendGoal(ac, x, y, yaw);
}

void tagIdCallback(const std_msgs::Int32::ConstPtr &msg)
{
    // 获取接收到的 AprilTag ID
    if (!tag_received)
    {
        tag_id = msg->data;
        tag_received = true; // 设置标志变量为 true
        ROS_INFO("Received AprilTag ID: %d", tag_id);
    }
}

void sleep(double second)
{
    ros::Duration(second).sleep();
}

void sendGoal(MoveBaseClient &ac, double x, double y, double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    actionlib::SimpleClientGoalState state = ac.getState();
    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Target point (%.3f, %.3f, %.3f) reached successfully!", x, y, yaw);
        break;
    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, x, y, yaw);
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;
    std_srvs::Empty empty_srv;
    ros::ServiceClient arm_zero_client = nh.serviceClient<std_srvs::Empty>("/control_center/zero_service");
    arm_zero_client.call(empty_srv);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // 订阅 /detected_tag_id 主题
    ros::Subscriber sub = nh.subscribe("/detected_tag_id", 1000, tagIdCallback);

    double grab_desk_x = 2.12;  // 抓取的桌子的x坐标
    double grab_desk_y = 0.13; // 抓取的桌子的y坐标

    double tag_1_put_x = 1.95;   // 放置tag1的x坐标
    double tag_1_put_y = 2.20; // 放置tag1的y坐标 2.28

    double tag_2_put_x = 1.06;   // 放置tag2的x坐标1.26
    double tag_2_put_y = 2.20; // 放置tag2的y坐标

    //左移
    move_safe(pub, 0.0, 0.2, 14);
    //前移
    move_safe(pub, 0.3, 0.0, 50);

    //导航到货架
    sendGoal(ac, grab_desk_x, grab_desk_y, 0);

    // 启动 print_id.launch 文件
    system("roslaunch carry_robot print_id.launch");

    // 等待接收到 tag_id
    while (!tag_received && ros::ok())
    {
        ros::spinOnce();
        sleep(0.1);
    }

    // 先后退一小段,大约30cm，防止规划时发生碰撞
    move_safe(pub, -0.1, 0.0, 30);

    // 根据 tag_id 执行不同的任务
    if (tag_id == 1)
    {
        // put TAG1
        sendGoal(ac, tag_1_put_x, tag_1_put_y, 1.57);
        tag_id = 2;

        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
        move_safe(pub, -0.2, 0.0, 25);
    }
    else if (tag_id == 2)
    {
        // 抓取 TAG2
        sendGoal(ac, tag_2_put_x, tag_2_put_y, 1.57);
        tag_id = 1;

        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
        move_safe(pub, -0.2, 0.0, 25);
    }

    // 先右移一小段,大约30cm，防止再规划碰撞
    sendGoal(ac, grab_desk_x, grab_desk_y, 0);
    
    // 前进
    move_safe(pub, 0.1, 0.0, 5);

    // 抓取TAG位1的物块
    system("roslaunch carry_robot print_id.launch");

    move_safe(pub, 0.0, 0.15, 10);
    move_safe(pub, -0.1, 0.0, 20);

    // 发送放置导航点
    if (tag_id == 1)
    {
        sendGoal(ac, tag_1_put_x, tag_1_put_y, 1.57);
        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
        move_safe(pub, -0.2, 0.0, 25);
    }
    else if (tag_id == 2)
    {
        sendGoal(ac, tag_2_put_x, tag_2_put_y, 1.57);
        // 放置物块
        system("roslaunch carry_robot arm_put.launch");
        move_safe(pub, -0.2, 0.0, 25);
    }

    // 发送返回导航点
    sendGoal(ac, 0.0, 0.0, -1.57);
    
    // zhongdian cheshen tiaozheng (终点车身调整)
    move_safe(pub, 0.1, 0.0, 23); // yyx
    move_safe(pub, 0.0, -0.08, 23); // cbx

    return 0;
}