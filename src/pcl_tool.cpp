#include "pcl_tool.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/filters/passthrough.h>

namespace pcl_tool
{

void applyXYZFilter(
    std::vector<PointData>& point_data,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max)
{
    // 简单的范围过滤
    std::vector<PointData> filtered;
    filtered.reserve(point_data.size());
    
    for (const auto& pd : point_data) {
        const auto& pt = pd.point;
        if (pt.x >= x_min && pt.x <= x_max &&
            pt.y >= y_min && pt.y <= y_max &&
            pt.z >= z_min && pt.z <= z_max) {
            filtered.push_back(pd);
        }
    }
    
    point_data = std::move(filtered);
}

Eigen::Matrix4f interpolateOdom(
    const nav_msgs::msg::Odometry::SharedPtr& odom1,
    const nav_msgs::msg::Odometry::SharedPtr& odom2,
    const rclcpp::Time& target_time)
{
    // 计算插值比例
    rclcpp::Time t1(odom1->header.stamp);
    rclcpp::Time t2(odom2->header.stamp);
    
    double duration = (t2 - t1).seconds();
    double ratio = 0.0;
    if (duration > 1e-6) {
        ratio = (target_time - t1).seconds() / duration;
        ratio = std::clamp(ratio, 0.0, 1.0);
    }
    
    // 插值位置
    Eigen::Vector3f pos1(odom1->pose.pose.position.x, 
                         odom1->pose.pose.position.y, 
                         odom1->pose.pose.position.z);
    Eigen::Vector3f pos2(odom2->pose.pose.position.x, 
                         odom2->pose.pose.position.y, 
                         odom2->pose.pose.position.z);
    Eigen::Vector3f pos = pos1 + ratio * (pos2 - pos1);
    
    // 插值四元数（SLERP）
    Eigen::Quaternionf q1(odom1->pose.pose.orientation.w,
                          odom1->pose.pose.orientation.x,
                          odom1->pose.pose.orientation.y,
                          odom1->pose.pose.orientation.z);
    Eigen::Quaternionf q2(odom2->pose.pose.orientation.w,
                          odom2->pose.pose.orientation.x,
                          odom2->pose.pose.orientation.y,
                          odom2->pose.pose.orientation.z);
    Eigen::Quaternionf q = q1.slerp(ratio, q2);
    
    // 构建变换矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = q.toRotationMatrix();
    transform.block<3,1>(0,3) = pos;
    
    return transform;
}

void applyDistortionCorrection(
    std::vector<PointData>& point_data,
    const rclcpp::Time& cloud_time,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr>& odom_buffer)
{
    // 检查里程计缓存
    if (odom_buffer.size() < 2 || point_data.empty()) {
        return;  // 数据不足或点云为空，无法矫正
    }
    
    // 获取点云结束时刻对应的里程计位姿（作为参考位姿）
    rclcpp::Time cloud_end_time = cloud_time;  // 可以根据最大offset_time调整
    
    // 查找最接近点云结束时刻的里程计
    auto ref_odom = odom_buffer.back();
    for (size_t i = 0; i < odom_buffer.size(); ++i) {
        rclcpp::Time odom_time(odom_buffer[i]->header.stamp);
        if (odom_time >= cloud_end_time) {
            ref_odom = odom_buffer[i];
            break;
        }
    }
    
    Eigen::Matrix4f ref_transform = Eigen::Matrix4f::Identity();
    Eigen::Vector3f ref_pos(ref_odom->pose.pose.position.x,
                            ref_odom->pose.pose.position.y,
                            ref_odom->pose.pose.position.z);
    Eigen::Quaternionf ref_q(ref_odom->pose.pose.orientation.w,
                             ref_odom->pose.pose.orientation.x,
                             ref_odom->pose.pose.orientation.y,
                             ref_odom->pose.pose.orientation.z);
    ref_transform.block<3,3>(0,0) = ref_q.toRotationMatrix();
    ref_transform.block<3,1>(0,3) = ref_pos;
    
    Eigen::Matrix4f ref_transform_inv = ref_transform.inverse();
    
    // 对每个点进行畸变矫正
    for (auto& pd : point_data) {
        // 计算点的绝对时间戳
        rclcpp::Time point_time = cloud_time + rclcpp::Duration(0, pd.offset_time);
        
        // 在里程计缓存中查找对应的位姿
        Eigen::Matrix4f point_transform = Eigen::Matrix4f::Identity();
        
        // 查找包围点时间的两个里程计数据
        nav_msgs::msg::Odometry::SharedPtr odom_before = nullptr;
        nav_msgs::msg::Odometry::SharedPtr odom_after = nullptr;
        
        for (size_t i = 0; i < odom_buffer.size() - 1; ++i) {
            rclcpp::Time t1(odom_buffer[i]->header.stamp);
            rclcpp::Time t2(odom_buffer[i+1]->header.stamp);
            
            if (point_time >= t1 && point_time <= t2) {
                odom_before = odom_buffer[i];
                odom_after = odom_buffer[i+1];
                break;
            }
        }
        
        // 如果找到了对应的里程计数据，进行插值
        if (odom_before && odom_after) {
            point_transform = interpolateOdom(odom_before, odom_after, point_time);
        } else {
            // 使用最近的里程计数据
            auto closest_odom = odom_buffer.front();
            double min_diff = std::abs((point_time - rclcpp::Time(closest_odom->header.stamp)).seconds());
            
            for (const auto& odom : odom_buffer) {
                double diff = std::abs((point_time - rclcpp::Time(odom->header.stamp)).seconds());
                if (diff < min_diff) {
                    min_diff = diff;
                    closest_odom = odom;
                }
            }
            
            Eigen::Vector3f pos(closest_odom->pose.pose.position.x,
                               closest_odom->pose.pose.position.y,
                               closest_odom->pose.pose.position.z);
            Eigen::Quaternionf q(closest_odom->pose.pose.orientation.w,
                                closest_odom->pose.pose.orientation.x,
                                closest_odom->pose.pose.orientation.y,
                                closest_odom->pose.pose.orientation.z);
            point_transform.block<3,3>(0,0) = q.toRotationMatrix();
            point_transform.block<3,1>(0,3) = pos;
        }
        
        // 计算相对变换：从点时刻到参考时刻
        Eigen::Matrix4f relative_transform = ref_transform_inv * point_transform;
        
        // 应用变换到点
        Eigen::Vector4f pt(pd.point.x, pd.point.y, pd.point.z, 1.0f);
        Eigen::Vector4f pt_corrected = relative_transform * pt;
        
        // 更新点坐标
        pd.point.x = pt_corrected(0);
        pd.point.y = pt_corrected(1);
        pd.point.z = pt_corrected(2);
    }
}

} // namespace pcl_tool

