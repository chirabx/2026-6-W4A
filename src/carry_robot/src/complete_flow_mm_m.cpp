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

// 声明新添加的放置函数
void put_where(MoveBaseClient &ac, ros::Publisher &pub, int current_id, 
               double t1_x, double t1_y, 
               double t2_x, double t2_y);

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

// 根据识别到的 Tag ID 决定放到哪里去的封装函数
void put_where(MoveBaseClient &ac, ros::Publisher &pub, int current_id, 
               double t1_x, double t1_y, 
               double t2_x, double t2_y)
{
    if (current_id == 1)
    {
        ROS_INFO("Camera detected TAG 1. Moving to put zone 1...");
        sendGoal(ac, t1_x, t1_y, 1.57);
    }
    else if (current_id == 2)
    {
        ROS_INFO("Camera detected TAG 2. Moving to put zone 2...");
        sendGoal(ac, t2_x, t2_y, 1.57);
    }
    else
    {
        ROS_WARN("Invalid TAG ID detected (%d). Aborting put task!", current_id);
        return; // 如果识别出错，跳过放置
    }

    // 执行放置动作
    system("roslaunch carry_robot arm_put.launch");
    
    // 统一执行放置后的安全后退
    move_safe(pub, -0.2, 0.0, 25);
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

    double grab_desk_x = 2.15;  // 抓取的桌子的x坐标
    double grab_desk_y = 0.01; //抓取的桌子的y坐标0.01

    double tag_1_put_x = 1.97;   // 放置tag1的x坐标 2.00
    double tag_1_put_y = 2.12; // 放置tag1的y坐标 2.07

    double tag_2_put_x = 0.95;   // 放置tag2的x坐标1.00 0.95
    double tag_2_put_y = 2.12; // 放置tag2的y坐标 2.12

    //左移
    move_safe(pub, 0.0, 0.2, 18);
    //前移
    move_safe(pub, 0.3, 0.0, 60);

    //导航到货架
    sendGoal(ac, grab_desk_x, grab_desk_y, 0);

    // ================= 第一次抓取与放置 =================

    // 启动 print_id.launch 文件
    system("roslaunch carry_robot print_id.launch");

    // 等待接收到 tag_id
    while (!tag_received && ros::ok())
    {
        ros::spinOnce();
        sleep(0.1);
    }

    // 先后退一小段,大约30cm，防止规划时发生碰撞
    move_safe(pub, 0.0, 0.15, 10);

    // 调用放置函数，根据真实识别到的 tag_id 去放置
    put_where(ac, pub, tag_id, tag_1_put_x, tag_1_put_y, tag_2_put_x, tag_2_put_y);

    // 【关键】：重置标志位，让系统忘记第一个 ID，为第二次识别做准备
    tag_received = false;
    tag_id = -1;


    // ================= 第二次抓取与放置 =================

    // 先右移一小段,大约30cm，防止再规划碰撞
    sendGoal(ac, grab_desk_x, grab_desk_y, 0);
    
    // 前进
    move_safe(pub, 0.1, 0.0, 5);

    // 抓取TAG位1的物块 (重新启动摄像头识别)
    system("roslaunch carry_robot print_id.launch");

    while (!tag_received && ros::ok())
    {
        ros::spinOnce();
        sleep(0.1);
    }

    // 调整车身姿态
    move_safe(pub, 0.0, 0.15, 10);
    // move_safe(pub, -0.1, 0.0, 20);

    // 再次调用放置函数，小车会根据刚刚【重新识别】到的新 ID 去对应位置
    put_where(ac, pub, tag_id, tag_1_put_x, tag_1_put_y, tag_2_put_x, tag_2_put_y);


    // ================= 任务结束，返回原点 =================

    // 发送返回导航点
    sendGoal(ac, 0.3, 0.0, 0);
    
    // zhongdian cheshen tiaozheng (终点车身调整)
    
    move_safe(pub, 0.0, -0.08, 23); // cbx23
    move_safe(pub, -0.1, 0.0, 50); // yyx
    return 0;
}