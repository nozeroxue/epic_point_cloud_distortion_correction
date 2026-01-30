#include "correction_node.hpp"

CorrectionNode::CorrectionNode(const rclcpp::NodeOptions & options)
    : Node("correction_node", options)
{
    // 声明和获取参数
    this->declare_parameter<std::string>("livox_topic", "/livox/lidar");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("output_topic", "/livox/pointcloud2");
    this->declare_parameter<std::string>("output_frame_id", "livox_frame");
    this->declare_parameter<bool>("use_best_effort_qos", false);
    
    // 声明畸变矫正参数
    this->declare_parameter<bool>("enable_distortion_correction", true);
    
    // 声明滤波参数
    this->declare_parameter<bool>("enable_xyz_filter", true);
    this->declare_parameter<double>("x_filter_min", -50.0);
    this->declare_parameter<double>("x_filter_max", 50.0);
    this->declare_parameter<double>("y_filter_min", -50.0);
    this->declare_parameter<double>("y_filter_max", 50.0);
    this->declare_parameter<double>("z_filter_min", -5.0);
    this->declare_parameter<double>("z_filter_max", 5.0);

    this->get_parameter("livox_topic", livox_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_frame_id", output_frame_id_);
    
    // 获取畸变矫正参数
    this->get_parameter("enable_distortion_correction", enable_distortion_correction_);
    
    // 获取滤波参数
    this->get_parameter("enable_xyz_filter", enable_xyz_filter_);
    this->get_parameter("x_filter_min", x_filter_min_);
    this->get_parameter("x_filter_max", x_filter_max_);
    this->get_parameter("y_filter_min", y_filter_min_);
    this->get_parameter("y_filter_max", y_filter_max_);
    this->get_parameter("z_filter_min", z_filter_min_);
    this->get_parameter("z_filter_max", z_filter_max_);

    // 创建订阅者
    livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_,
        10,
        std::bind(&CorrectionNode::livoxCloudCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
        std::bind(&CorrectionNode::odomCallback, this, std::placeholders::_1)
    );

    // 创建发布者 - 使用Reliable QoS以兼容RViz
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    );

    RCLCPP_INFO(this->get_logger(), "Correction Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to Livox: %s", livox_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to Odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Distortion Correction enabled: %s", enable_distortion_correction_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "XYZ Filter enabled: %s", enable_xyz_filter_ ? "true" : "false");
    if (enable_xyz_filter_) {
        RCLCPP_INFO(this->get_logger(), "X filter range: [%.2f, %.2f]", x_filter_min_, x_filter_max_);
        RCLCPP_INFO(this->get_logger(), "Y filter range: [%.2f, %.2f]", y_filter_min_, y_filter_max_);
        RCLCPP_INFO(this->get_logger(), "Z filter range: [%.2f, %.2f]", z_filter_min_, z_filter_max_);
    }
}

void CorrectionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    odom_buffer_.push_back(msg);
    
    // 限制缓存大小
    while (odom_buffer_.size() > MAX_ODOM_BUFFER_SIZE) {
        odom_buffer_.pop_front();
    }
}

void CorrectionNode::livoxCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received Livox message with %u points", msg->point_num);
    
    if (msg->point_num == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    // 1. 转换Livox CustomMsg为点云数据
    auto point_data = convertToPointData(msg);
    
    if (point_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to convert point cloud");
        return;
    }
    
    size_t original_size = point_data.size();
    
    // 2. 应用畸变矫正
    if (enable_distortion_correction_) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        if (odom_buffer_.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Insufficient odometry data for distortion correction (have %zu, need >= 2)", 
                odom_buffer_.size());
        } else {
            rclcpp::Time cloud_time(msg->header.stamp);
            pcl_tool::applyDistortionCorrection(point_data, cloud_time, odom_buffer_);
            RCLCPP_INFO(this->get_logger(), "Applied distortion correction");
        }
    }
    
    // 3. 应用xyz滤波
    if (enable_xyz_filter_) {
        pcl_tool::applyXYZFilter(
            point_data,
            x_filter_min_, x_filter_max_,
            y_filter_min_, y_filter_max_,
            z_filter_min_, z_filter_max_
        );
        // RCLCPP_INFO(this->get_logger(), "After filtering: %zu points (from %zu)", 
        //             point_data.size(), original_size);
    }
    
    // 4. 转换为PointCloud2格式并发布
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_cloud->points.reserve(point_data.size());
    for (const auto& pd : point_data) {
        pcl_cloud->points.push_back(pd.point);
    }
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = false;
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header = msg->header;
    if (cloud_msg.header.frame_id.empty()) {
        cloud_msg.header.frame_id = output_frame_id_;
    }
    
    cloud_pub_->publish(cloud_msg);
    
    // RCLCPP_INFO(this->get_logger(), "Published PointCloud2 with %zu points to %s", 
    //             point_data.size(), output_topic_.c_str());
}

std::vector<pcl_tool::PointData> CorrectionNode::convertToPointData(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& livox_msg)
{
    std::vector<pcl_tool::PointData> point_data;
    point_data.reserve(livox_msg->point_num);
    
    // 填充点云数据
    for (size_t i = 0; i < livox_msg->point_num; ++i) {
        const auto& src_point = livox_msg->points[i];
        
        pcl_tool::PointData pd;
        pd.point.x = src_point.x;
        pd.point.y = src_point.y;
        pd.point.z = src_point.z;
        pd.point.intensity = static_cast<float>(src_point.reflectivity);
        pd.offset_time = src_point.offset_time;
        
        point_data.push_back(pd);
    }
    
    return point_data;
}
