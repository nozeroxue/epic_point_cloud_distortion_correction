#ifndef CORRECTION_NODE_HPP_
#define CORRECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "pcl_tool.hpp"
#include <deque>
#include <mutex>

class CorrectionNode : public rclcpp::Node
{
public:
    explicit CorrectionNode(const rclcpp::NodeOptions & options);
    ~CorrectionNode() = default;

private:
    // 回调函数：处理Livox点云消息
    void livoxCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    
    // 回调函数：处理Odometry消息
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // 转换函数：将Livox CustomMsg转换为点云数据
    std::vector<pcl_tool::PointData> convertToPointData(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr& livox_msg);

    // ROS2 订阅者和发布者
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 参数
    std::string livox_topic_;
    std::string odom_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    
    // 畸变矫正参数
    bool enable_distortion_correction_;
    
    // 滤波参数
    bool enable_xyz_filter_;
    double x_filter_min_;
    double x_filter_max_;
    double y_filter_min_;
    double y_filter_max_;
    double z_filter_min_;
    double z_filter_max_;
    
    // 里程计数据缓存
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
    std::mutex odom_mutex_;
    static constexpr size_t MAX_ODOM_BUFFER_SIZE = 200;  // 缓存最多200个里程计数据
};

#endif  // CORRECTION_NODE_HPP_

