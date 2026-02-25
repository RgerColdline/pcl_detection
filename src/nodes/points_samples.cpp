#include "nodes/points_sample.h"  // 确保包含头文件

#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <vector>

// 必须包含这些mavros消息头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// 全局配置参数
constexpr float ALTITUDE  = 0.7f;  // 悬停高度(米)
constexpr float TARGET_X  = 1.0f;  // 前进目标距离(米)
constexpr float HOVER_ERR = 0.1f;  // 悬停容差(米)

// ROS全局变量
mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
double yaw  = 0.0;                 // 无人机当前偏航角(弧度)
double roll = 0.0, pitch = 0.0;    // 添加roll和pitch变量
float init_x = 0.0, init_y = 0.0;  // 起飞点坐标
bool is_initialized = false;       // 坐标系初始化标志

// 控制指令
mavros_msgs::PositionTarget setpoint;

// 添加NodeHandle指针以便在回调中使用
ros::NodeHandle *nh_ptr = nullptr;

/************************************************************************
 * 核心回调函数
 ************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    local_pos = *msg;

    // 提取偏航角 - 修复：使用变量而非常量
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // 修复：传递变量而非常量

    // 初始化起飞坐标系
    if (!is_initialized && msg->pose.pose.position.z > 0.1) {
        init_x         = msg->pose.pose.position.x;
        init_y         = msg->pose.pose.position.y;
        is_initialized = true;
    }
}

// Livox回调包装函数
void livox_cb_wrapper(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
    if (nh_ptr) {
        livox_cb(msg, *nh_ptr);
    }
}

/************************************************************************
 * 基础飞控逻辑
 ************************************************************************/
bool reached_target(float x, float y, float z, float yaw_target) {
    if (!is_initialized) return false;

    const auto &pos = local_pos.pose.pose.position;
    return (std::abs(pos.x - (init_x + x)) < HOVER_ERR &&
            std::abs(pos.y - (init_y + y)) < HOVER_ERR && std::abs(pos.z - z) < HOVER_ERR &&
            std::abs(yaw - yaw_target) < 0.1);
}

void set_position(float x, float y, float z, float yaw_target) {
    setpoint.type_mask        = 0b10011111000;  // 仅位置控制
    setpoint.coordinate_frame = 1;              // LOCAL_NED
    setpoint.position.x       = init_x + x;
    setpoint.position.y       = init_y + y;
    setpoint.position.z       = z;
    setpoint.yaw              = yaw_target;
}

/************************************************************************
 * 主程序
 ************************************************************************/
int main(int argc, char **argv) {

    setlocale(LC_ALL, "");

    ros::init(argc, argv, "simple_flight");
    ros::NodeHandle nh;

    // 保存NodeHandle指针以便在回调中使用
    nh_ptr = &nh;

    // 初始化保存参数
    nh.setParam(SAVE_SAMPLE_PARAM, false);

    // 订阅状态与位置
    ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub   = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver2::CustomMsg>(
        "/livox/lidar", 10, livox_cb_wrapper);  // 使用包装函数

    // 发布控制指令
    ros::Publisher setpoint_pub =
        nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 服务客户端 - 修复：包含正确的头文件后使用正确的类型
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);  // 必须>2Hz

    // 等待飞控连接
    ROS_INFO("Waiting for vehicle connection...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 预发布100个悬停点
    set_position(0, 0, ALTITUDE, 0);
    for (int i = 100; ros::ok() && i > 0; --i) {
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    // 请求OFFBOARD模式 - 修复：正确声明类型
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 请求解锁 - 修复：正确声明类型
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value  = true;

    ros::Time last_request = ros::Time::now();
    ROS_INFO("Entering flight sequence...");

    while (ros::ok()) {
        // OFFBOARD模式切换
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        }
        // 无人机解锁
        else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // 飞行状态机
        if (current_state.armed) {
            static enum { TAKEOFF, CRUISE, HOVER } flight_phase = TAKEOFF;

            switch (flight_phase) {
            case TAKEOFF:  // 起飞阶段
                set_position(0, 0, ALTITUDE, 0);
                if (reached_target(0, 0, ALTITUDE, 0)) {
                    ROS_INFO("Takeoff complete - ascending to %.1fm", ALTITUDE);
                    flight_phase = CRUISE;
                }
                break;

            case CRUISE:  // 前进阶段
                set_position(TARGET_X, 0, ALTITUDE, 0);
                if (reached_target(TARGET_X, 0, ALTITUDE, 0)) {
                    ROS_INFO("Cruise complete - reached target position");
                    flight_phase = HOVER;
                }
                break;

            case HOVER:  // 悬停阶段
                set_position(TARGET_X, 0, ALTITUDE, 0);
                ROS_INFO_THROTTLE(2, "Hovering at (%.2f, %.2f, %.2f)",
                                  local_pos.pose.pose.position.x, local_pos.pose.pose.position.y,
                                  local_pos.pose.pose.position.z);
                break;
            }
        }

        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}