#!/bin/bash
# obs.bash - PCL 对象检测节点启动脚本 (bash 版本)
# 用法：source src/pcl_detection/shell/obs.bash  或  ./obs.bash

# 获取脚本所在目录（兼容 source 和直接执行）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 工作空间路径（脚本所在目录的上上级目录）
# shell/ → pcl_detection/ → src/ → workspace/
WORKSPACE="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# source 工作空间环境
. "$WORKSPACE/devel/setup.bash"

# 启动 launch 文件
roslaunch pcl_detection object_detector.launch
