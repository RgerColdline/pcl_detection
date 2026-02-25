#include "points_sample.h"

#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <vector>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

constexpr float ALTITUDE  = 0.7f;
constexpr float TARGET_X  = 1.0f;
constexpr float HOVER_ERR = 0.1f;

mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
double yaw = 0.0, roll = 0.0, pitch = 0.0;
double init_x = 0.0, init_y = 0.0;
bool is_initialized = false;

mavros_msgs::PositionTarget setpoint;

ros::NodeHandle *nh_ptr = nullptr;

void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    local_pos = *msg;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!is_initialized && msg->pose.pose.position.z > 0.1) {
        init_x = msg->pose.pose.position.x;
        init_y = msg->pose.pose.position.y;
        is_initialized = true;
    }
}

void livox_cb_wrapper(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {
    if (nh_ptr) {
        livox_cb(msg, *nh_ptr);
    }
}

bool reached_target(float x, float y, float z, float yaw_target) {
    if (!is_initialized) return false;

    const auto &pos = local_pos.pose.pose.position;
    return (std::abs(pos.x - (init_x + x)) < HOVER_ERR &&
            std::abs(pos.y - (init_y + y)) < HOVER_ERR && 
            std::abs(pos.z - z) < HOVER_ERR &&
            std::abs(yaw - yaw_target) < 0.1);
}

void set_position(float x, float y, float z, float yaw_target) {
    setpoint.type_mask = 0b10011111000;
    setpoint.coordinate_frame = 1;
    setpoint.position.x = init_x + x;
    setpoint.position.y = init_y + y;
    setpoint.position.z = z;
    setpoint.yaw = yaw_target;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "simple_flight");
    ros::NodeHandle nh;

    nh_ptr = &nh;
    nh.setParam(SAVE_SAMPLE_PARAM, false);

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber livox_sub = nh.subscribe<livox_ros_driver2::CustomMsg>(
        "/livox/lidar", 10, livox_cb_wrapper);

    ros::Publisher setpoint_pub =
        nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

    ROS_INFO("Waiting for vehicle connection...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    set_position(0, 0, ALTITUDE, 0);
    for (int i = 100; ros::ok() && i > 0; --i) {
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ROS_INFO("Entering flight sequence...");

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        }
        else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        if (current_state.armed) {
            static enum { TAKEOFF, CRUISE, HOVER } flight_phase = TAKEOFF;

            switch (flight_phase) {
            case TAKEOFF:
                set_position(0, 0, ALTITUDE, 0);
                if (reached_target(0, 0, ALTITUDE, 0)) {
                    ROS_INFO("Takeoff complete");
                    flight_phase = CRUISE;
                }
                break;

            case CRUISE:
                set_position(TARGET_X, 0, ALTITUDE, 0);
                if (reached_target(TARGET_X, 0, ALTITUDE, 0)) {
                    ROS_INFO("Cruise complete");
                    flight_phase = HOVER;
                }
                break;

            case HOVER:
                set_position(TARGET_X, 0, ALTITUDE, 0);
                ROS_INFO_THROTTLE(2, "Hovering at (%.2f, %.2f, %.2f)",
                                  local_pos.pose.pose.position.x, 
                                  local_pos.pose.pose.position.y,
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
