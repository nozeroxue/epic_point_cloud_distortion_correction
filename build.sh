#!/bin/bash

# 切换到工作空间根目录
cd "$(dirname "$0")/../../.."

echo "=========================================="
echo "编译 point_cloud_distortion_correction"
echo "=========================================="

# 编译包
colcon build --packages-select point_cloud_distortion_correction --symlink-install

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "编译成功！"
    echo "=========================================="
    echo ""
    echo "使用方法："
    echo "1. source install/setup.bash"
    echo "2. ros2 launch point_cloud_distortion_correction correction.launch.py"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "编译失败，请检查错误信息"
    echo "=========================================="
    exit 1
fi

