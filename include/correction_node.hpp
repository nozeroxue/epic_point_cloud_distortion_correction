#ifndef CORRECTION_NODE_HPP_
#define CORRECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

class CorrectionNode : public rclcpp::Node
{
public:
    explicit CorrectionNode(const rclcpp::NodeOptions & options);
    ~CorrectionNode() = default;

private:
    // 回调函数：处理Livox点云消息
    void livoxCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    
    // 转换函数：将Livox CustomMsg转换为PointCloud2
    sensor_msgs::msg::PointCloud2 convertToPointCloud2(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr& livox_msg);
    
    // 滤波函数：对点云进行xyz方向的滤波
    sensor_msgs::msg::PointCloud2 applyXYZFilter(
        const sensor_msgs::msg::PointCloud2& input_cloud);

    // ROS2 订阅者和发布者
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 参数
    std::string livox_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    
    // 滤波参数
    bool enable_xyz_filter_;
    double x_filter_min_;
    double x_filter_max_;
    double y_filter_min_;
    double y_filter_max_;
    double z_filter_min_;
    double z_filter_max_;
};

#endif  // CORRECTION_NODE_HPP_

