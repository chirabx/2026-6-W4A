#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>
#include <tf/tf.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
}

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
    // 函数自动停止机器人，无需手动编写停止逻辑
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mgrab_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // yyx

    // 在参数中加载要抓取的tag目标
    std::string tag_link;
    private_nh.getParam("tag", tag_link);
    ROS_INFO("The value of tag is %s", tag_link.c_str());

    // 初始化service创建服务客户端
    ros::ServiceClient arm_pose_client = nh.serviceClient<upros_message::ArmPosition>("/control_center/arm_pos_service");
    ros::ServiceClient arm_grab_client = nh.serviceClient<std_srvs::Empty>("/control_center/grab_service");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ROS_INFO("tf coordinate transformaing....");

    // 获取tag到机械臂基坐标的坐标变换
    geometry_msgs::TransformStamped tfs_1 = buffer.lookupTransform("arm_base_link", tag_link, ros::Time(0), ros::Duration(5));

    int bias_x = 0;  // x方向的偏移，增加的机械臂往左多探的毫米数
    int bias_y = 95; // y方向的偏移，增加的机械臂往前多探的毫米数
    int bias_z = 75; // z方向的偏移，增加的机械臂往上多探的毫米数

    // 首先我们要知道ros坐标系和逆运动学坐标系的关系，ros坐标系中x轴向前，y轴向左，z轴向上，
    // 而逆运动学坐标系中x轴向前，y轴向右，z轴向上，所以我们在进行坐标转换时需要进行相应的调整。
    int dist = int(tfs_1.transform.translation.x * 1000.0);           // 前后距离（distance）
    int arm_x = int(tfs_1.transform.translation.y * 1000.0);          // 左右
    int arm_y = dist + bias_y;                                        // 前后距离加上偏移得到机械臂y坐标
    int arm_z = int(tfs_1.transform.translation.z * 1000.0) + bias_z; // 高度加上偏移得到机械臂z坐标

    std::cout << "arm_x: " << arm_x << " arm_y: " << arm_y << " arm_z: " << arm_z << " dist: " << dist << std::endl;

    std_srvs::Empty empty_srv;
    // 1. 距离过近，后退
    if (dist < 135)
    {
        move_safe(pub, -0.08, 0.0, 6);
    }
    // 2. 距离过远，前进
    else if (dist > 145)
    {
        move_safe(pub, 0.06, 0.0, int((dist - 145) / 2));
    }

    // 3. Y坐标分段修正
    if (dist > 160 && dist < 260)
    {
        if (dist < 180)
        {
            arm_y -= 30;
        }
        else if (dist < 200)
        {
            arm_y -= 70;
        }
        else if (dist < 220)
        {
            arm_y -= 90;
        }
        else if (dist < 240)
        {
            arm_y -= 110;
        }
        else
        {
            arm_y -= 130;
        } // dist >= 240 且 dist < 260
    }

    // 4 & 5. X方向底盘平移修正
    if (arm_x >= -8)
    {
        move_safe(pub, 0.0, 0.04, arm_x + 7);
    }
    else if (arm_x <= -22)
    {
        // 左移修正
        move_safe(pub, 0.0, -0.05, -arm_x - 22);
    }

    // 6. 极限值保护 (超出工作空间强制设定安全姿态)
    if (arm_x > 30 || arm_x < -40)
    {
        arm_x = -16;
        arm_y = 228;
        arm_z = 82;
    }
    // 逆运算移动抓取到上方
    upros_message::ArmPosition srv;
    srv.request.x = -9;
    srv.request.y = 188;
    srv.request.z = 182;

    arm_pose_client.call(srv);

    sleep(3.0);

    arm_grab_client.call(empty_srv);

    // 下探抓取
    srv.request.x = arm_x -7; //13
    srv.request.y = arm_y + 25; // 17
    srv.request.z = arm_z - 7;  //-5
    arm_pose_client.call(srv);
    sleep(3.0);

    // 抬起来
    srv.request.x = 0;
    srv.request.y = 188; // 50
    srv.request.z = 182;
    arm_pose_client.call(srv);
    sleep(2.0);

    ros::shutdown();

    return 0;
}