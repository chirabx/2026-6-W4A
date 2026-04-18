#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/Twist.h> //yyx
#include <std_msgs/Int32.h>

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>
#include <tf/tf.h>

void sleep(double second)
{
    ros::Duration(second).sleep();
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

    // 单位转换，ros坐标系到逆运算坐标系
    int y1 = int(tfs_1.transform.translation.x * 1000.0);
    int x = int(tfs_1.transform.translation.y * 1000.0);
    int y = y1 + bias_y;
    int z = int(tfs_1.transform.translation.z * 1000.0) + bias_z;

    std::cout << "x: " << x << " y: " << y << " z: " << z << " y1: " << y1 << std::endl;

    std_srvs::Empty empty_srv;
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);
    int count = 0;
    if (y1 < 135)
    {
        vel_msg.linear.x = -0.08;

        while (ros::ok() && count < 6)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
    }
    if (y1 > 145)
    {

        vel_msg.linear.x = 0.06; // 0.08

        while (ros::ok() && count < int((y1 - 145) / 2))
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);
    }
    if (y1 > 160 && y1 < 180)
    {
        y = y - 30;
    }
    else if (y1 >= 180 && y1 < 200)
    {
        y = y - 70;
    }
    else if (y1 >= 200 && y1 < 220)
    {
        y = y - 90;
    }
    else if (y1 >= 220 && y1 < 240)
    {
        y = y - 110;
    }
    else if (y1 >= 240 && y1 < 260)
    {
        y = y - 130;
    }

    if (x >= -5)
    { //&& x<=30
        if (x < 0)
        {
            x = -x;
        }
        vel_msg.linear.y = 0.04; // 0.06
        count = 0;

        while (ros::ok() && count < x + 2)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.y = 0.0;
        pub.publish(vel_msg);
        x = x + 10;
    }
    else if (x <= -25)
    {
        vel_msg.linear.y = -0.05;
        count = 0;

        while (ros::ok() && count < (-x) - 25 + 3)
        { //+3
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.y = 0.0;
        pub.publish(vel_msg);
    }

    if (x > 30 || x < -40)
    {
        x = -16;
        y = 228;
        z = 82;
    }
    // 逆运算移动抓取到上方
    upros_message::ArmPosition srv;
    srv.request.x = -9;
    srv.request.y = 188;
    srv.request.z = 182;

    arm_pose_client.call(srv);

    sleep(3.0);

    arm_grab_client.call(empty_srv);

    // 下探
    srv.request.x = x + 10;
    srv.request.y = y + 17; // 25
    srv.request.z = z - 7;  //-5
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
