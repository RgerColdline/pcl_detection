#!/bin/zsh

# 已存在会话就直接杀死
if tmux has-session -t pcl_test 2>/dev/null;then
    tmux kill-session -t pcl_test
fi

# 创建会话和第一个窗口
tmux new -d -s pcl_test -n roscore

# 第一面板放roscore
tmux send-keys -t pcl_test:0 'roscore' C-m

# 分出去一面
tmux split-window -h -t pcl_test:0

# 先给这一面source一下，一会launch
tmux send-keys -t pcl_test:0.1 'source ~/group2_ws/devel/setup.zsh' C-m

# 给这一面放launch
tmux send-keys -t pcl_test:0.1 'sleep 2 && roslaunch pcl_detection pcl_detection.launch' C-m

# 整理
# tmux select-layout -t pcl_test:0 tiled

# 加进那个会话
tmux attach-session -t pcl_test