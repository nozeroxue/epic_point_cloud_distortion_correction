#include "correction_node.hpp"

CorrectionNode::CorrectionNode(const rclcpp::NodeOptions & options)
    : Node("correction_node", options)
{
    // 声明和获取参数
    this->declare_parameter<std::string>("livox_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/pointcloud2");
    this->declare_parameter<std::string>("output_frame_id", "livox_frame");
    this->declare_parameter<bool>("use_best_effort_qos", false);
    
    // 声明滤波参数
    this->declare_parameter<bool>("enable_xyz_filter", true);
    this->declare_parameter<double>("x_filter_min", -50.0);
    this->declare_parameter<double>("x_filter_max", 50.0);
    this->declare_parameter<double>("y_filter_min", -50.0);
    this->declare_parameter<double>("y_filter_max", 50.0);
    this->declare_parameter<double>("z_filter_min", -5.0);
    this->declare_parameter<double>("z_filter_max", 5.0);

    this->get_parameter("livox_topic", livox_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_frame_id", output_frame_id_);
    
    // 获取滤波参数
    this->get_parameter("enable_xyz_filter", enable_xyz_filter_);
    this->get_parameter("x_filter_min", x_filter_min_);
    this->get_parameter("x_filter_max", x_filter_max_);
    this->get_parameter("y_filter_min", y_filter_min_);
    this->get_parameter("y_filter_max", y_filter_max_);
    this->get_parameter("z_filter_min", z_filter_min_);
    this->get_parameter("z_filter_max", z_filter_max_);

    
    livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_,
        10,
        std::bind(&CorrectionNode::livoxCloudCallback, this, std::placeholders::_1)
    );

    // 创建发布者 - 使用Reliable QoS以兼容RViz
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    );

    RCLCPP_INFO(this->get_logger(), "Correction Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", livox_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "XYZ Filter enabled: %s", enable_xyz_filter_ ? "true" : "false");
    if (enable_xyz_filter_) {
        RCLCPP_INFO(this->get_logger(), "X filter range: [%.2f, %.2f]", x_filter_min_, x_filter_max_);
        RCLCPP_INFO(this->get_logger(), "Y filter range: [%.2f, %.2f]", y_filter_min_, y_filter_max_);
        RCLCPP_INFO(this->get_logger(), "Z filter range: [%.2f, %.2f]", z_filter_min_, z_filter_max_);
    }
}

void CorrectionNode::livoxCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Livox message with %u points", msg->point_num);
    
    if (msg->point_num == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    // 转换为PointCloud2格式
    auto cloud_msg = convertToPointCloud2(msg);
    
    // 应用xyz滤波
    if (enable_xyz_filter_) {
        cloud_msg = applyXYZFilter(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "After filtering: %u points", cloud_msg.width);
    }
    
    // 发布点云
    cloud_pub_->publish(cloud_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published PointCloud2 with %u points to %s", 
                cloud_msg.width, output_topic_.c_str());
}

sensor_msgs::msg::PointCloud2 CorrectionNode::convertToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& livox_msg)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    // 设置header
    cloud_msg.header = livox_msg->header;
    if (cloud_msg.header.frame_id.empty()) {
        cloud_msg.header.frame_id = output_frame_id_;
    }
    
    // 设置点云属性
    cloud_msg.height = 1;
    cloud_msg.width = livox_msg->point_num;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    
    // 定义PointCloud2的字段
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(7,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "tag", 1, sensor_msgs::msg::PointField::UINT8,
        "line", 1, sensor_msgs::msg::PointField::UINT8,
        "offset_time", 1, sensor_msgs::msg::PointField::UINT32
    );
    
    modifier.resize(livox_msg->point_num);
    
    // 创建迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(cloud_msg, "offset_time");
    
    // 填充点云数据
    for (size_t i = 0; i < livox_msg->point_num; ++i) {
        const auto& point = livox_msg->points[i];
        
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        *iter_intensity = static_cast<float>(point.reflectivity);
        *iter_tag = point.tag;
        *iter_line = point.line;
        *iter_offset_time = point.offset_time;
        
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_tag;
        ++iter_line;
        ++iter_offset_time;
    }
    
    return cloud_msg;
}

sensor_msgs::msg::PointCloud2 CorrectionNode::applyXYZFilter(
    const sensor_msgs::msg::PointCloud2& input_cloud)
{
    // 转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(input_cloud, *pcl_cloud);
    
    // X方向滤波
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(pcl_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min_, x_filter_max_);
    pass_x.filter(*filtered_cloud);
    
    // Y方向滤波
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filtered_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min_, y_filter_max_);
    pass_y.filter(*filtered_cloud);
    
    // Z方向滤波
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(filtered_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_filter_min_, z_filter_max_);
    pass_z.filter(*filtered_cloud);
    
    // 转换回ROS2消息格式
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*filtered_cloud, output_cloud);
    output_cloud.header = input_cloud.header;
    
    return output_cloud;
}

