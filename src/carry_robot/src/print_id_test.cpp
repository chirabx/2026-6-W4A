#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

// 机械臂抓取设定的偏移量
const int bias_y = 95; 
const int bias_z = 75; 

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // 如果没有检测到标签，安静地等待，每秒只打印一次提示，不疯狂刷屏也不报错
    if (msg->detections.empty())
    {
        ROS_INFO_THROTTLE(1.0, "No AprilTags detected. Waiting for tag to appear...");
        return;
    }

    // 遍历当前画面中检测到的所有 AprilTag
    for (const auto &detection : msg->detections)
    {
        int tag_id = detection.id[0];
        
        // 获取话题中直接输出的位姿坐标 (单位：米)
        const auto &position = detection.pose.pose.pose.position;

        // 【修改点】：将原来的 tfs_1.transform.translation 替换为 position
        // 单位转换，ros坐标系到逆运算坐标系
        int dist = int(position.x * 1000.0);  // 原为以底盘为坐标系的x坐标
        int arm_x = int(position.y * 1000.0); // 原为以底盘为坐标系的y坐标
        int arm_y = dist + bias_y;
        int arm_z = int(position.z * 1000.0) + bias_z;

        // 持续打印到终端
        ROS_INFO("Tag ID: %d | dist: %4d | arm_x: %4d | arm_y: %4d | arm_z: %4d", tag_id, dist, arm_x, arm_y, arm_z);

    }
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "debug_tag_topic_printer");
    ros::NodeHandle nh;

    ROS_INFO("Started AprilTag coordinate debugging node.");
    ROS_INFO("Move the robot or the Tag to see values changing. Press Ctrl+C to stop.");

    // 订阅 /tag_detections 主题
    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, callback);

    // 保持节点运行，有新消息就会自动进入 callback
    ros::spin();

    return 0;
}