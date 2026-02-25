#!/bin/bash

###############################################################################
# PCL 对象检测节点一键启动脚本
# 功能：自动加载 ROS 环境并启动障碍物检测节点
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 工作空间路径（根据实际情况修改）
WORKSPACE="/home/jetson/group2_ws"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   PCL 对象检测节点启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}错误：工作空间不存在：$WORKSPACE${NC}"
    exit 1
fi

# 进入工作空间
cd "$WORKSPACE" || exit 1

# 加载 ROS 环境
echo -e "${YELLOW}[1/3] 加载 ROS 环境...${NC}"
if [ -f "$WORKSPACE/devel/setup.zsh" ]; then
    source "$WORKSPACE/devel/setup.zsh"
    echo -e "${GREEN}✓ 已加载 zsh 环境${NC}"
elif [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
    echo -e "${GREEN}✓ 已加载 bash 环境${NC}"
else
    echo -e "${RED}✗ 错误：未找到 ROS 环境文件${NC}"
    exit 1
fi

# 检查 ROS 主节点是否运行
if ! roscore &> /dev/null; then
    echo -e "${YELLOW}[2/3] 启动 roscore...${NC}"
    roscore &
    sleep 3
    echo -e "${GREEN}✓ roscore 已启动${NC}"
else
    echo -e "${GREEN}✓ roscore 已在运行${NC}"
fi

# 启动检测节点
echo -e "${YELLOW}[3/3] 启动对象检测节点...${NC}"
echo -e "${GREEN}========================================${NC}"
roslaunch pcl_detection object_detector.launch
