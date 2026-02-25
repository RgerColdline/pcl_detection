#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>

void livox_cb(const livox_ros_driver2::CustomMsg::ConstPtr& msg, ros::NodeHandle& nh);

#define SAVE_SAMPLE_PARAM "save_sample"
