#include "call_back.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

#include <memory>

using namespace std;

// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;

// 监听命令回复消息
void left_arm_io_state_call_back(const ur_msgs::IOStates& msg)
{
    const ur_msgs::Digital rsp = msg.digital_out_states[0];
    const bool state = rsp.state;
	//ROS_INFO_STREAM("Notifying response to left arm  state" << state);
    if (state) // state == true,arm reached,or state == false, arm moveing
    {
        auto& event = scheduler->left_arm_reach_flag_;
        if (event.isWaiting())
        {
            ROS_INFO("Notifying response to left arm");
			send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"通知主臂已到位","info");
            event.notify(true);
        }
    }
}

// 监听命令回复消息
void right_arm_io_state_call_back(const ur_msgs::IOStates& msg)
{
    const ur_msgs::Digital rsp = msg.digital_out_states[0];
    const bool state = rsp.state;
	//ROS_INFO_STREAM("Notifying response to right arm  state" << state);
    if (state)
    {
        auto& event = scheduler->right_arm_reach_flag_;
        if (event.isWaiting())
        {
            ROS_INFO("Notifying response to right arm");
			send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"通知从臂已到位","info");
            event.notify(true);
        }
    }
}

// 左臂末端位姿数据话题回调
void left_arm_tool_position_call_back(const geometry_msgs::TwistStamped& msg)
{
	//ROS_ERROR("left_arm_tool_position_call_back.");
	DoubleValueArray point = {0, 0, 0, 0, 0, 0};
	point[0] = msg.twist.linear.x;
	point[1] = msg.twist.linear.y;
	point[2] = msg.twist.linear.z;
	point[3] = msg.twist.angular.x;
	point[4] = msg.twist.angular.y;
	point[5] = msg.twist.angular.z;

	auto& event = scheduler->left_arm_pp_received_flag_;
    if (event.isWaiting())
    {
    	try
    	{
			scheduler->setArm1PP(point);
			ROS_INFO_STREAM("left_arm_tool_position_point: " << point);
			//ROS_INFO("Notifying arm position point value received.");
			event.notify(true);
    	}
		catch (const std::exception& e) 
		{
	        ROS_ERROR_STREAM("ERROR: failed to get position point value: " << e.what());
	    }
    }
}

// 右臂末端位姿数据话题回调
void right_arm_tool_position_call_back(const geometry_msgs::TwistStamped& msg)
{
	DoubleValueArray point = {0, 0, 0, 0, 0, 0};
	point[0] = msg.twist.linear.x;
	point[1] = msg.twist.linear.y;
	point[2] = msg.twist.linear.z;
	point[3] = msg.twist.angular.x;
	point[4] = msg.twist.angular.y;
	point[5] = msg.twist.angular.z;

	auto& event = scheduler->right_arm_pp_received_flag_;
	if (event.isWaiting())
	{
		try
		{
			scheduler->setArm2PP(point);
			ROS_INFO_STREAM("right_arm_tool_position_point: " << point);
			//ROS_INFO("Notifying arm position point value received.");
			event.notify(true);
		}
		catch (const std::exception& e) 
		{
		    ROS_ERROR_STREAM("ERROR: failed to get position point value: " << e.what());
		}
	}
}