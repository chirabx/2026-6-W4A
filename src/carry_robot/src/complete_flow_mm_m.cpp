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
#include <sensor_msgs/LaserScan.h>
// ======== TF 相关头文件 ========
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h> // 包含 getYaw
#include <cmath>
#include <memory>

using namespace std;
int tag_id = -1;           // 初始化为 -1，表示未检测到任何 tag
bool tag_received = false; // 标志变量，表示是否接收到 tag_id
double front_wall_distance = -1.0; 
bool scan_received = false;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 全局 TF 监听器指针
std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

void sendGoal(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw);
void move_safe(ros::Publisher &pub, double vx, double vy, int max_count);

// 声明放置函数
void put_where(MoveBaseClient &ac, ros::Publisher &pub, int current_id,
               double t1_x, double t1_y,
               double t2_x, double t2_y);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void align_with_wall(ros::Publisher &pub, double target_dist);
void align_y_with_tf(ros::Publisher &pub, double target_x, double target_y);
void align_yaw_with_tf(ros::Publisher &pub, double target_yaw);
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
    double scan_dist = 0;
    double target_x = 0; // 记录当前选择的X目标
    double target_y = 0; // 记录当前选择的Y目标
    double target_yaw = 1.5708;
    if (current_id == 1)
    {
        ROS_INFO("Camera detected TAG 1. Moving to put zone 1...");
        target_x = t1_x;
        target_y = t1_y;
        sendGoal(ac, pub, target_x, target_y, target_yaw);
        scan_dist = 0.63;
    }
    else if (current_id == 2)
    {
        ROS_INFO("Camera detected TAG 2. Moving to put zone 2...");
        target_x = t2_x;
        target_y = t2_y;
        sendGoal(ac, pub, target_x, target_y, target_yaw);
        scan_dist = 0.6;
    }
    else
    {
        ROS_WARN("Invalid TAG ID detected (%d). Aborting put task!", current_id);
        return; // 如果识别出错，跳过放置
    }


    align_y_with_tf(pub, target_x, target_y);
    sleep(0.5); // 等待车身稳定

    align_yaw_with_tf(pub, target_yaw);
    sleep(0.5);

    align_with_wall(pub, scan_dist); 
    sleep(0.5); // 等待车身稳定
    // 执行放置动作
    system("roslaunch carry_robot arm_put.launch");

    // 统一执行放置后的安全后退
    move_safe(pub, -0.2, 0.0, 25);
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw)
{
    ROS_INFO("Executing backward retry logic...");
    move_safe(pub, -0.05, 0.0, 30); // Backward 30 steps

    ROS_INFO("Retrying to move to target point (%.3f, %.3f, %.3f)", x, y, yaw);
    sendGoal(ac, pub, x, y, yaw);
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
// 订阅激光雷达的回调函数
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS标准中，0弧度代表车头正前方。我们计算0弧度在数组中的索引
    int center_index = (0.0 - msg->angle_min) / msg->angle_increment;
    
    // 取车头正前方左右各几个点求平均值，过滤噪点
    int window = 5; 
    double sum = 0;
    int count = 0;
    
    for (int i = center_index - window; i <= center_index + window; i++) {
        // 防止索引越界
        if (i >= 0 && i < msg->ranges.size()) {
            double r = msg->ranges[i];
            // 排除无穷大或无效数据
            if (!std::isinf(r) && !std::isnan(r) && r > 0.1) {
                sum += r;
                count++;
            }
        }
    }
    
    if (count > 0) {
        front_wall_distance = sum / count;
        scan_received = true;
    }
}

//基于激光雷达的末端闭环对齐函数
void align_with_wall(ros::Publisher &pub, double target_dist)
{
    ROS_INFO(">>> [LiDAR Alignment] Starting LiDAR alignment to target distance: %.2fm", target_dist);
    ros::Rate rate(10);
    int timeout = 0;
    int max_timeout = 80; // 最大允许调整8秒，防止死循环

    while (ros::ok() && timeout < max_timeout)
    {
        ros::spinOnce(); // 更新雷达数据
        
        if (!scan_received) {
            rate.sleep();
            continue;
        }

        // 计算误差 (当前距离 - 目标距离)
        double error = front_wall_distance - target_dist;

        // 如果误差小于 1cm (0.01m)，则认为对齐成功，退出循环
        if (std::abs(error) <= 0.01) {
            ROS_INFO(">>> [LiDAR Alignment] Success! Current distance: %.3fm", front_wall_distance);
            break;
        }

        geometry_msgs::Twist vel;
        // 如果当前距离大于50cm，说明离墙远了，需要前进；反之需要后退
        if (error > 0) {
            vel.linear.x = 0.04;  // 慢速前进
        } else {
            vel.linear.x = -0.04; // 慢速后退
        }
        
        pub.publish(vel);
        
        rate.sleep();
        timeout++;
    }

    // 刹车停止
    geometry_msgs::Twist stop_vel;
    stop_vel.linear.x = 0.0;
    pub.publish(stop_vel);
    ROS_INFO(">>> [LiDAR Alignment] Completed.");
}

void align_yaw_with_tf(ros::Publisher &pub, double target_yaw)
{
    ROS_INFO(">>> [TF Alignment] Starting Yaw (Angle) alignment to %.3f...", target_yaw);
    ros::Rate rate(10);
    int timeout = 0;
    int max_timeout = 50; // 最大允许调整 5 秒

    while (ros::ok() && timeout < max_timeout)
    {
        try
        {
            geometry_msgs::TransformStamped current_tf = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
            double cur_yaw = tf2::getYaw(current_tf.transform.rotation);

            // 计算角度偏差，并用 atan2 规范化到 [-pi, pi] 之间
            double err_yaw = atan2(sin(target_yaw - cur_yaw), cos(target_yaw - cur_yaw));

            // 如果角度误差小于 0.015 弧度 (约 0.8度)，判定对齐成功
            if (std::abs(err_yaw) <= 0.015)
            {
                ROS_INFO(">>> [TF Alignment] Yaw Success! Error within 0.8 degrees.");
                break;
            }

            geometry_msgs::Twist vel;
            // 顺时针或逆时针自转微调
            vel.angular.z = (err_yaw > 0) ? 0.15 : -0.15;
            pub.publish(vel);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF Error during Yaw adjustment: %s", ex.what());
        }

        rate.sleep();
        timeout++;
    }

    // 刹车
    geometry_msgs::Twist stop_vel;
    stop_vel.angular.z = 0.0;
    pub.publish(stop_vel);
}

// 基于TF的闭环左右横移对齐函数 (左右Y轴)
void align_y_with_tf(ros::Publisher &pub, double target_x, double target_y)
{
    ROS_INFO(">>> [TF Alignment] Starting lateral (Left/Right) alignment...");
    ros::Rate rate(10);
    int timeout = 0;
    int max_timeout = 60; // 最大允许调整6秒

    while (ros::ok() && timeout < max_timeout)
    {
        try
        {
            // 获取当前真实坐标
            geometry_msgs::TransformStamped current_tf = tfBuffer->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
            double cur_x = current_tf.transform.translation.x;
            double cur_y = current_tf.transform.translation.y;
            double cur_yaw = tf2::getYaw(current_tf.transform.rotation);

            // 计算全局偏差
            double dx = target_x - cur_x;
            double dy = target_y - cur_y;

            // 将全局偏差转换到小车局部坐标系，重点提取左右方向的偏差 (local_dy)
            double local_dy = -dx * sin(cur_yaw) + dy * cos(cur_yaw);

            // 如果左右误差小于 1cm (0.01m)，说明对齐成功！
            if (std::abs(local_dy) <= 0.01)
            {
                ROS_INFO(">>> [TF Alignment] Success! Lateral error within 1cm.");
                break;
            }

            // 速度控制
            geometry_msgs::Twist vel;
            // 麦轮横移摩擦力大，速度稍微给大一点点，防止卡死不走
            if (local_dy > 0) {
                vel.linear.y = 0.06; // 向左平移
            } else {
                vel.linear.y = -0.06; // 向右平移
            }
            
            pub.publish(vel);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF Error during Y-axis adjustment: %s", ex.what());
        }

        rate.sleep();
        timeout++;
    }

    // 刹车停止
    geometry_msgs::Twist stop_vel;
    stop_vel.linear.x = 0.0;
    stop_vel.linear.y = 0.0;
    pub.publish(stop_vel);
    ROS_INFO(">>> [TF Alignment] Completed.");
}

void sleep(double second)
{
    ros::Duration(second).sleep();
}

void sendGoal(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw)
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
        performRetryLogic(ac, pub, x, y, yaw);
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;

    // 开启后台异步接收回调
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化全局 TF 监听器
    tfBuffer = std::make_shared<tf2_ros::Buffer>();
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // 订阅 /detected_tag_id 主题
    ros::Subscriber sub = nh.subscribe("/detected_tag_id", 1000, tagIdCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    double grab_desk_x = 2.15; // 抓取的桌子的x坐标
    double grab_desk_y = 0.08; // 抓取的桌子的y坐标0.01

    double tag_1_put_x = 2.02; // 放置tag1的x坐标 1.99
    double tag_1_put_y = 2.09; // 放置tag1的y坐标 2.12

    double tag_2_put_x = 1.055; // 放置tag2的x坐标 1.03
    double tag_2_put_y = 2.09; // 放置tag2的y坐标 2.12

    // 左移
    move_safe(pub, 0.0, 0.2, 18);
    // 前移
    move_safe(pub, 0.3, 0.0, 60);

    // 导航到货架
    sendGoal(ac, pub, grab_desk_x, grab_desk_y, 0);

    // ================= 第一次抓取与放置 =================

    // 启动 print_id.launch 文件
    system("roslaunch carry_robot print_id.launch");

    // 等待接收到 tag_id
    while (!tag_received && ros::ok())
    {
        ros::spinOnce();
        sleep(0.1);
    }

    // 先后退一小段,防止规划时发生碰撞
    move_safe(pub, -0.15, 0, 10);

    // 调用放置函数，根据真实识别到的 tag_id 去放置
    put_where(ac, pub, tag_id, tag_1_put_x, tag_1_put_y, tag_2_put_x, tag_2_put_y);

    // 重置标志位，让系统忘记第一个 ID，为第二次识别做准备
    tag_received = false;
    tag_id = -1;

    // ================= 第二次抓取与放置 =================

    sendGoal(ac, pub, grab_desk_x, grab_desk_y, 0);

    // 抓取TAG位1的物块 (启动摄像头识别)
    system("roslaunch carry_robot print_id.launch");

    while (!tag_received && ros::ok())
    {
        ros::spinOnce();
        sleep(0.1);
    }

    // 调整车身姿态
    // move_safe(pub, 0.0, 0.15, 10);
    move_safe(pub, -0.15, 0.0, 10);

    // 再次调用放置函数，小车会根据刚刚【重新识别】到的新 ID 去对应位置
    put_where(ac, pub, tag_id, tag_1_put_x, tag_1_put_y, tag_2_put_x, tag_2_put_y);

    // ================= 任务结束，返回原点 =================

    // 发送返回导航点
    sendGoal(ac, pub, 0.3, 0.0, 0);

    // zhongdian cheshen tiaozheng (终点车身调整)

    move_safe(pub, 0.0, -0.08, 23); // cbx23
    move_safe(pub, -0.1, 0.0, 50);  // yyx
    return 0;
}