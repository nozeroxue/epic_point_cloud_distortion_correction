# 故障排查指南

## 问题：没有消息发布

### 第一步：运行自动诊断脚本

```bash
cd /home/zixiangli/project/costmap_ws/src/point_cloud_distortion_correction
./debug_check.sh
```

### 第二步：手动检查

#### 1. 检查输入话题是否存在

```bash
# 列出所有话题
ros2 topic list

# 检查 /rear_lidar 是否在列表中
ros2 topic list | grep rear_lidar
```

**如果话题不存在**：
- 检查Livox驱动是否启动
- 确认话题名称配置是否正确

#### 2. 检查话题是否有数据发布

```bash
# 查看话题发布频率（等待几秒）
ros2 topic hz /rear_lidar

# 查看话题信息
ros2 topic info /rear_lidar -v

# 查看话题数据
ros2 topic echo /rear_lidar --no-arr
```

**如果没有数据**：
- Livox驱动可能没有连接到传感器
- 传感器可能没有正常工作

#### 3. 检查话题类型是否匹配

```bash
ros2 topic info /rear_lidar
```

应该显示：
```
Type: livox_ros_driver2/msg/CustomMsg
```

**如果类型不匹配**：需要修改代码以适配实际的消息类型

#### 4. 检查QoS策略

查看发布者的QoS：
```bash
ros2 topic info /rear_lidar -v
```

查看订阅者的QoS：
```bash
ros2 node info /correction_node
```

**QoS不匹配的解决方法**：

编辑配置文件，设置 `use_best_effort_qos: true`：
```bash
nano /home/zixiangli/project/costmap_ws/src/point_cloud_distortion_correction/config/correction_params.yaml
```

或启动时覆盖参数：
```bash
ros2 launch point_cloud_distortion_correction correction.launch.py \
    use_best_effort_qos:=true
```

#### 5. 检查节点是否接收到消息

重新编译并启动，观察日志：
```bash
cd /home/zixiangli/project/costmap_ws
colcon build --packages-select point_cloud_distortion_correction
source install/setup.bash

# 启动节点，查看详细日志
ros2 run point_cloud_distortion_correction point_cloud_distortion_correction_node
```

**应该看到**：
```
[INFO] [correction_node]: Correction Node initialized
[INFO] [correction_node]: Subscribing to: /rear_lidar
[INFO] [correction_node]: Publishing to: /livox/pointcloud2
[INFO] [correction_node]: Using SensorData QoS
```

**当收到数据时应该看到**：
```
[INFO] [correction_node]: Received Livox message with XXXX points
[INFO] [correction_node]: Published PointCloud2 with XXXX points to /livox/pointcloud2
```

**如果看不到"Received Livox message"**：
- QoS不匹配 → 尝试设置 `use_best_effort_qos: true`
- 话题名称错误 → 检查配置文件
- 消息类型不匹配 → 检查Livox驱动版本

#### 6. 检查输出话题

```bash
# 检查输出话题是否存在
ros2 topic list | grep pointcloud2

# 查看输出频率
ros2 topic hz /livox/pointcloud2

# 查看输出话题信息
ros2 topic info /livox/pointcloud2 -v
```

### 常见问题解决方案

#### 问题A：节点启动但看不到"Received"日志

**原因**：订阅者和发布者的QoS不匹配

**解决**：
```bash
# 方法1：修改配置文件
nano config/correction_params.yaml
# 设置：use_best_effort_qos: true

# 方法2：启动时覆盖
ros2 launch point_cloud_distortion_correction correction.launch.py \
    use_best_effort_qos:=true
```

#### 问题B：话题 /rear_lidar 不存在

**原因**：Livox驱动未启动或话题名称不对

**解决**：
```bash
# 1. 查找实际的Livox话题
ros2 topic list | grep -i "livox\|lidar"

# 2. 修改配置文件中的话题名称
nano config/correction_params.yaml

# 3. 或启动时指定
ros2 launch point_cloud_distortion_correction correction.launch.py \
    livox_topic:=/actual/topic/name
```

#### 问题C：收到消息但输出为空

**查看完整日志**：
```bash
ros2 run point_cloud_distortion_correction point_cloud_distortion_correction_node \
    --ros-args --log-level debug
```

检查是否有错误或警告信息。

#### 问题D：编译错误

```bash
# 清理并重新编译
cd /home/zixiangli/project/costmap_ws
rm -rf build/ install/ log/
colcon build --packages-select livox_ros_driver2 point_cloud_distortion_correction
source install/setup.bash
```

### 验证成功的标志

1. ✅ 节点启动时显示订阅和发布的话题
2. ✅ 持续看到 "Received Livox message" 日志
3. ✅ 持续看到 "Published PointCloud2" 日志
4. ✅ `ros2 topic hz /livox/pointcloud2` 显示消息频率
5. ✅ RViz2能够可视化 `/livox/pointcloud2`

### 快速测试命令集合

```bash
# 一键检查所有内容
ros2 topic list && \
ros2 topic hz /rear_lidar &
PID=$!
sleep 5
kill $PID
ros2 topic hz /livox/pointcloud2
```

### 联系支持

如果以上步骤都无法解决问题，请提供：
1. `ros2 topic list` 的输出
2. `ros2 topic info /rear_lidar -v` 的输出
3. 节点启动后的完整日志
4. `./debug_check.sh` 的输出

