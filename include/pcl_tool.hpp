#ifndef PCL_TOOL_HPP_
#define PCL_TOOL_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <deque>

namespace pcl_tool
{

// 扩展的点云数据结构（用于内部处理）
struct PointData
{
    pcl::PointXYZI point;
    uint32_t offset_time;  // 相对于点云帧头的时间偏移（纳秒）
};

/**
 * @brief 对点云数据进行XYZ方向的PassThrough滤波
 * 
 * @param point_data 输入输出的点云数据向量
 * @param x_min X方向最小值
 * @param x_max X方向最大值
 * @param y_min Y方向最小值
 * @param y_max Y方向最大值
 * @param z_min Z方向最小值
 * @param z_max Z方向最大值
 */
void applyXYZFilter(
    std::vector<PointData>& point_data,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max);

/**
 * @brief 对点云数据进行畸变矫正
 * 
 * @param point_data 输入输出的点云数据向量
 * @param cloud_time 点云帧的时间戳
 * @param odom_buffer 里程计数据缓存
 */
void applyDistortionCorrection(
    std::vector<PointData>& point_data,
    const rclcpp::Time& cloud_time,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr>& odom_buffer);

/**
 * @brief 根据两个里程计位姿插值计算中间位姿
 * 
 * @param odom1 前一个里程计
 * @param odom2 后一个里程计
 * @param target_time 目标时间
 * @return 插值后的变换矩阵
 */
Eigen::Matrix4f interpolateOdom(
    const nav_msgs::msg::Odometry::SharedPtr& odom1,
    const nav_msgs::msg::Odometry::SharedPtr& odom2,
    const rclcpp::Time& target_time);

} // namespace pcl_tool

#endif  // PCL_TOOL_HPP_

