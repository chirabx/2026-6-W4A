#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

const int bias_y = 95; 
const int bias_z = 75; 

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.empty())
    {
        ROS_INFO_THROTTLE(1.0, "No AprilTags detected. Waiting for tag to appear...");
        return;
    }

    for (const auto &detection : msg->detections)
    {
        int tag_id = detection.id[0];
        const auto &position = detection.pose.pose.pose.position;

        int dist = int(position.x * 1000.0); 
        int arm_x = int(position.y * 1000.0);
        int arm_y = dist + bias_y;
        int arm_z = int(position.z * 1000.0) + bias_z;

        ROS_INFO("Tag ID: %d | dist: %4d | arm_x: %4d | arm_y: %4d | arm_z: %4d", tag_id, dist, arm_x, arm_y, arm_z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug_tag_topic_printer");
    ros::NodeHandle nh;

    // 【注意看这里！如果运行成功，你的终端第一句话必定是下面这句！】
    ROS_INFO("Started AprilTag coordinate debugging node.");
    
    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, callback);
    ros::spin();
    return 0;
}