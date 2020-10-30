#ifndef CALL_BACK_H
#define CALL_BACK_H

#include "public_pkg/cmd_msg.h"
#include "ur_msgs/IOStates.h"
#include "public_pkg/status_digital_msg.h"
#include "public_pkg/status_analog_msg.h"
#include "fusion/server_position_msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "public_pkg/heart_beat_msg.h"
#include "public_pkg/ground_interact_msg.h"
#include "public_pkg/range_msg.h"


// cmd_msg回调函数，负责命令信息的解析和分发
void cmd_call_back(const public_pkg::cmd_msg& msg);

// 地面站确认雷达选点完成信号
void lidar_point_confirm(const std::string& cmd_str);

// 监听命令回复消息
void left_arm_io_state_call_back(const ur_msgs::IOStates& msg);
void right_arm_io_state_call_back(const ur_msgs::IOStates& msg);
// 左臂末端位姿数据话题回调
void left_arm_tool_position_call_back(const geometry_msgs::TwistStamped& msg);
// 右臂末端位姿数据话题回调
void right_arm_tool_position_call_back(const geometry_msgs::TwistStamped& msg);

// 监听通用数字量信息
void status_digital_call_back(const public_pkg::status_digital_msg& msg);

// 监听通用模拟量信息
void status_analog_call_back(const public_pkg::status_analog_msg& msg);

// 监听地面站心跳信息
void remotecontrol_heartbeat_call_back(const public_pkg::heart_beat_msg& msg);

// 自动选点回调
void automatic_point_selection_call_back(const public_pkg::status_digital_msg& msg);

//【Add by hexiaoxu 2019-8-5】
// 监听lidar1信息
void status_lidar1_call_back(const fusion::server_position_msg& msg);

 //【Add by hexiaoxu 2019-8-5】
// 监听lidar2信息
void status_lidar2_call_back(const fusion::server_position_msg& msg);

//【Add by hepei 2019-8-22】
// 监听lidar1信息
void status_lidar1b_call_back(const fusion::server_position_msg& msg);

 //【Add by hepei 2019-8-22】
// 监听lidar2信息
void status_lidar2b_call_back(const fusion::server_position_msg& msg);

#endif // CALL_BACK_H
