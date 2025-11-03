# Point Cloud Distortion Correction

这个ROS2包用于读取Livox点云消息格式（CustomMsg），并将其转换为标准的PointCloud2格式进行发布。

## 功能特性

- 订阅Livox CustomMsg格式的点云数据
- 转换为sensor_msgs/PointCloud2格式
- 保留Livox点云的所有字段信息（x, y, z, intensity, tag, line, offset_time）
- 支持参数配置

## 依赖项

- ROS2 (Humble/Foxy或更高版本)
- rclcpp
- sensor_msgs
- livox_ros_driver2
- PCL
- pcl_conversions

## 编译

```bash
cd ~/costmap_ws
colcon build --packages-select point_cloud_distortion_correction
source install/setup.bash
```

## 使用方法

### 方法1：使用launch文件启动（推荐）

launch文件会自动读取配置文件 `config/correction_params.yaml`：

```bash
ros2 launch point_cloud_distortion_correction correction.launch.py
```

### 方法2：使用自定义参数覆盖配置文件

```bash
# 覆盖单个参数
ros2 launch point_cloud_distortion_correction correction.launch.py \
    livox_topic:=/your/livox/topic

# 覆盖多个参数
ros2 launch point_cloud_distortion_correction correction.launch.py \
    livox_topic:=/rear_lidar \
    output_topic:=/converted/pointcloud2 \
    use_best_effort_qos:=true
```

### 方法3：使用自定义配置文件

```bash
ros2 launch point_cloud_distortion_correction correction.launch.py \
    config_file:=/path/to/your/config.yaml
```

### 方法4：直接运行节点

```bash
ros2 run point_cloud_distortion_correction point_cloud_distortion_correction_node
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `livox_topic` | string | `/livox/lidar` | Livox CustomMsg输入话题名称 |
| `output_topic` | string | `/livox/pointcloud2` | PointCloud2输出话题名称 |
| `output_frame_id` | string | `livox_frame` | 输出点云的frame_id |
| `use_best_effort_qos` | bool | `false` | 使用BestEffort QoS（如果订阅失败可尝试设为true） |

## 配置文件

配置文件位于 `config/correction_params.yaml`：

```yaml
/**:
  ros__parameters:
    livox_topic: "/rear_lidar"
    output_topic: "/livox/pointcloud2"
    output_frame_id: "livox_frame"
    use_best_effort_qos: false
```

修改此文件后，重新启动launch文件即可生效（无需重新编译）。

## 话题

### 订阅话题

- `~/livox/lidar` (livox_ros_driver2/msg/CustomMsg): Livox点云数据输入

### 发布话题

- `~/livox/pointcloud2` (sensor_msgs/msg/PointCloud2): 转换后的标准点云数据

## PointCloud2字段说明

转换后的PointCloud2包含以下字段：

- `x`, `y`, `z` (float32): 点的3D坐标 (单位: 米)
- `intensity` (float32): 反射强度 (0-255)
- `tag` (uint8): Livox标签
- `line` (uint8): 激光线号
- `offset_time` (uint32): 相对于基准时间的偏移时间

## 示例

### 查看转换后的点云

```bash
# 在RViz2中可视化
rviz2

# 查看话题信息
ros2 topic echo /livox/pointcloud2

# 查看话题频率
ros2 topic hz /livox/pointcloud2
```

## 许可证

Apache-2.0

