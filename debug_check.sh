#!/bin/bash

echo "=========================================="
echo "点云转换节点调试检查"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "1. 检查节点是否运行..."
if ros2 node list | grep -q "correction_node"; then
    echo -e "${GREEN}✓ correction_node 正在运行${NC}"
else
    echo -e "${RED}✗ correction_node 未运行${NC}"
    echo "  请先启动节点: ros2 launch point_cloud_distortion_correction correction.launch.py"
fi
echo ""

echo "2. 检查输入话题 /rear_lidar..."
if ros2 topic list | grep -q "/rear_lidar"; then
    echo -e "${GREEN}✓ /rear_lidar 话题存在${NC}"
    echo "  话题信息:"
    ros2 topic info /rear_lidar
    echo ""
    echo "  话题频率:"
    timeout 3 ros2 topic hz /rear_lidar 2>&1 || echo "  ${YELLOW}⚠ 3秒内未检测到消息${NC}"
else
    echo -e "${RED}✗ /rear_lidar 话题不存在${NC}"
    echo "  当前可用的话题:"
    ros2 topic list | grep -i "lidar\|livox\|point"
fi
echo ""

echo "3. 检查输出话题 /livox/pointcloud2..."
if ros2 topic list | grep -q "/livox/pointcloud2"; then
    echo -e "${GREEN}✓ /livox/pointcloud2 话题存在${NC}"
    echo "  话题信息:"
    ros2 topic info /livox/pointcloud2
    echo ""
    echo "  话题频率:"
    timeout 3 ros2 topic hz /livox/pointcloud2 2>&1 || echo "  ${YELLOW}⚠ 3秒内未检测到消息${NC}"
else
    echo -e "${RED}✗ /livox/pointcloud2 话题不存在${NC}"
fi
echo ""

echo "4. 检查节点参数..."
if ros2 node list | grep -q "correction_node"; then
    echo "  节点参数:"
    ros2 param list /correction_node
    echo ""
    echo "  当前配置:"
    echo "    livox_topic: $(ros2 param get /correction_node livox_topic 2>/dev/null || echo '未设置')"
    echo "    output_topic: $(ros2 param get /correction_node output_topic 2>/dev/null || echo '未设置')"
    echo "    output_frame_id: $(ros2 param get /correction_node output_frame_id 2>/dev/null || echo '未设置')"
fi
echo ""

echo "=========================================="
echo "调试建议："
echo "=========================================="
echo "1. 如果输入话题不存在或无数据："
echo "   - 检查Livox驱动是否正常运行"
echo "   - 确认话题名称是否正确（当前配置: /rear_lidar）"
echo ""
echo "2. 如果有输入但无输出："
echo "   - 查看节点日志: ros2 node info /correction_node"
echo "   - 检查是否有错误信息"
echo "   - 确认QoS策略是否匹配"
echo ""
echo "3. 查看实时日志："
echo "   ros2 run point_cloud_distortion_correction point_cloud_distortion_correction_node --ros-args --log-level debug"
echo ""
echo "4. 手动测试话题订阅："
echo "   ros2 topic echo /rear_lidar --no-arr"
echo ""
echo "5. 查看话题连接："
echo "   ros2 topic info /rear_lidar -v"
echo ""

