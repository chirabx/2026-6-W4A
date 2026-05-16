#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h> // 用于发布整数消息
#include <tf/tf.h>

ros::Publisher pub;
ros::Publisher tag_id_pub;     // 用于发布 tag_id 的发布者
bool has_detected_tag = false; // 标志变量，表示是否检测到过 AprilTag
bool tag_1_detected = false;   // 标志变量，表示是否检测到过 ID 为 1 的 AprilTag
bool tag_2_detected = false;   // 标志变量，表示是否检测到过 ID 为 2 的 AprilTag

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

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    // 检测到的 AprilTag 数量
    size_t num_tags = msg->detections.size();

    if (num_tags == 0)
    {
        // 没有检测到任何 AprilTag
        ROS_INFO("No AprilTags detected.");

        if (!has_detected_tag)
        {
            // 未检测到标签，执行后退
            move_safe(pub, -0.06, 0.0, 3);
        }
    }
    else
    {
        // 检测到 AprilTag
        has_detected_tag = true; // 设置标志变量为 true

        // 遍历检测到的 AprilTag
        for (const auto &detection : msg->detections)
        {
            // 输出 AprilTag 的 ID
            int tag_id = detection.id[0]; // AprilTag 的 ID 是一个整数数组，通常取第一个元素
            ROS_INFO("Detected AprilTag ID: %d", tag_id);

            // 获取 AprilTag 的位姿
            const auto &pose = detection.pose;

            // 输出位姿信息
            ROS_INFO("AprilTag Pose: ");
            ROS_INFO("Position: (%f, %f, %f)", pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

            // 将四元数转换为欧拉角
            tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double a = 180 / 3.14;

            // 输出欧拉角
            ROS_INFO("Orientation: Roll = %f, Pitch = %f, Yaw = %f", roll * a, pitch * a, yaw * a);

            double b = pitch * a;
            double c = roll * a;

            if (b > 60 || b < -60 || (c > 80 && c < 130) || (c < -80 && c > -130))
            {
                move_safe(pub, 0.0, -0.06, 4);
                ROS_INFO("识别为侧面，已右移");
                move_safe(pub, -0.06, 0.0, 8);
                ROS_INFO("已后退");
            }
            int wait_count = 0;
            // 稍作等待，确保主控程序的 Subscriber 已经连接上
            while (tag_id_pub.getNumSubscribers() == 0 && wait_count < 10)
            {
                ros::Duration(0.1).sleep();
                wait_count++;
            }
            // 发布 tag_id
            std_msgs::Int32 tag_id_msg;
            tag_id_msg.data = tag_id;

            for (int i = 1; i <= 5; ++i)
            {
                tag_id_pub.publish(tag_id_msg);
                ros::Duration(0.05).sleep();
            }

            // 根据 AprilTag 的 ID 启动不同的 launch 文件
            if (tag_id == 1 && !tag_1_detected)
            {
                tag_1_detected = true; // 设置标志变量为 true
                system("roslaunch carry_robot arm_grab_1.launch");
                ros::shutdown(); // 关闭当前节点
            }
            else if (tag_id == 2 && !tag_2_detected)
            {
                tag_2_detected = true; // 设置标志变量为 true
                system("roslaunch carry_robot arm_grab_2.launch");
                ros::shutdown(); // 关闭当前节点
            }
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "apriltag_id_listener");
    ros::NodeHandle nh;

    // 创建一个发布者，用于发布机器人速度消息
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 创建一个发布者，用于发布 tag_id 消息
    tag_id_pub = nh.advertise<std_msgs::Int32>("/detected_tag_id", 10);

    // 订阅 /tag_detections 主题
    ros::Subscriber sub = nh.subscribe("/tag_detections", 1000, callback);

    // 保持节点运行
    ros::spin();

    return 0;
}