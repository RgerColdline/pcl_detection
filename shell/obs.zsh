#!/usr/bin/env zsh
# obs.zsh - PCL 对象检测节点启动脚本 (zsh 版本)
# 用法：source src/pcl_detection/shell/obs.zsh  或  ./obs.zsh

# 获取脚本所在目录（兼容 source 和直接执行）
if [[ -n "${ZSH_SOURCE[0]}" ]]; then
    SCRIPT_DIR="${ZSH_SOURCE[0]:A:h}"
else
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

# 工作空间路径（脚本所在目录的上上级目录）
# shell/ → pcl_detection/ → src/ → workspace/
WORKSPACE="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

# source 工作空间环境（zsh 用 . 命令）
. "$WORKSPACE/devel/setup.zsh" 2>/dev/null || . "$WORKSPACE/devel/setup.bash"

# 启动 launch 文件
roslaunch pcl_detection object_detector.launch
