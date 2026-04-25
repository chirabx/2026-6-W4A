#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点，命名为 debug_tf_printer
    ros::init(argc, argv, "debug_tf_printer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 1. 读取参数，拼接出 tag_link
    int tag_id = 1;
    if (!private_nh.getParam("tag", tag_id))
    {
        ROS_WARN("Failed to get param 'tag', using default: %d", tag_id);
    }
    std::string tag_link = "tag_" + std::to_string(tag_id);
    ROS_INFO("Debugging TF coordinates for target: %s", tag_link.c_str());

    // 2. 初始化 TF 监听器
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 设置循环频率为 10Hz (每秒打印10次)
    ros::Rate loop_rate(10);

    // 机械臂抓取设定的偏移量
    int bias_x = 0;
    int bias_y = 95;
    int bias_z = 75;

    ROS_INFO("Start continuous coordinate printing. Move the robot to see changes. Press Ctrl+C to stop.");

    // 3. 持续监听并输出
    while (ros::ok())
    {
        geometry_msgs::TransformStamped tfs_1;
        try
        {
            // 获取最新时刻 (ros::Time(0)) tag_link 到 base_link 的坐标变换
            tfs_1 = buffer.lookupTransform("base_link", tag_link, ros::Time(0), ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            // 调试期间，如果摄像头丢失了 Tag 导致找不到 TF，不要退出程序
            // 而是打印警告，等待一会后继续下一次尝试查找
            ROS_WARN_THROTTLE(1.0, "Waiting for TF transform: %s", ex.what()); // 限制1秒最多打印一次警告，防止刷屏
            loop_rate.sleep();
            continue;
        }

        // 4. 单位转换，ros坐标系到逆运算坐标系
        int dist = int(tfs_1.transform.translation.x * 1000.0);  // 原为以底盘为坐标系的x坐标
        int arm_x = int(tfs_1.transform.translation.y * 1000.0); // 原为以底盘为坐标系的y坐标
        int arm_y = dist + bias_y;
        int arm_z = int(tfs_1.transform.translation.z * 1000.0) + bias_z;

        // 5. 持续打印到终端
        ROS_INFO("dist: %4d | arm_x: %4d | arm_y: %4d | arm_z: %4d", dist, arm_x, arm_y, arm_z);

        // 维持 10Hz 频率
        loop_rate.sleep();
    }

    return 0;
}