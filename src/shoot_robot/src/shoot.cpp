#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <serial/serial.h>

serial::Serial ser; // 全局串口对象

typedef std::vector<uint8_t> Buffer;

Buffer close_buffer = {0xA0};

Buffer open_buffer = {0xA1};

// 射击服务回调函数
bool shootCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (ser.isOpen())
    {

        ser.write(open_buffer);
        ROS_INFO("Sent shoot command: 1");
        return true;
    }
    ROS_ERROR("Serial port not open!");
    return false;
}

// 关闭服务回调函数
bool closeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (ser.isOpen())
    {
        ser.write(close_buffer);
        ROS_INFO("Sent close command: 0");
        return true;
    }
    ROS_ERROR("Serial port not open!");
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shooter_controller");
    ros::NodeHandle nh;

    // 初始化串口（参数可通过launch文件配置）
    try
    {
        ser.setPort("/dev/arm"); // 串口设备路径
        ser.setBaudrate(9600);   // 波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Serial init failed: %s", e.what());
        return -1;
    }

    // 创建服务服务器
    ros::ServiceServer shoot_srv = nh.advertiseService("shoot", shootCallback);
    ros::ServiceServer close_srv = nh.advertiseService("close", closeCallback);

    ROS_INFO("Shooter controller ready");
    ros::spin();

    ser.close(); // 关闭串口
    return 0;
}