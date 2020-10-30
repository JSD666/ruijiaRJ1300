#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

using namespace std;

// 一键剥皮指令功能函数
CmdLineFuncRsp strip_onekey_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_ONEKEY_SEQ, 1);

    // 等待动作执行结束
    ROS_INFO("[STRIP_ONEKEY IS WAITING FOR RESULT]");
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_ONEKEY_START,"info");
    

    auto& event = scheduler.strip_wire_tool_strip_onekey_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_ONEKEY_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_ONEKEY_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮初始化指令功能函数
CmdLineFuncRsp strip_init_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_INIT_SEQ, 1);

    // 等待动作执行结束
    ROS_INFO("[STRIP_INIT IS WAITING FOR RESULT]");
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_INIT_START,"info");
    

    auto& event = scheduler.strip_wire_tool_strip_init_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_INIT_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_INIT_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮正转功能函数
CmdLineFuncRsp strip_rotate_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	vector<string> fields = JobParser::GetCommandFields(line);
	if (fields.size() != 2)
	{
		ROS_ERROR_STREAM("Expected " << 2 - 1 << " arguments, but got " << fields.size() - 1);
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}

	int time_ms;
	try
	{
		time_ms = std::stoi(fields[1]);
	}
	catch (const std::exception &e)
	{
		ROS_ERROR_STREAM("Cannot convert argument \"" << fields[1] << "\" to int time");
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}
	if (time_ms < 0)
	{
		ROS_ERROR("Argument INT_TIME should not be negative");
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}

	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_ROTATE_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_ROTATE_START,"info");
    
	if (time_ms > 0) // 延时一小会，发送停止指令
	{
		ROS_INFO_STREAM("Sleeping " << time_ms << " ms to send STRIP_STOP");
		std::this_thread::sleep_for(chrono::milliseconds(time_ms));
		send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_STOP_SEQ, 1);
	}

	// 等待动作执行结束
    ROS_INFO("[STRIP_ROTATE IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_rotate_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_ROTATE_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_ROTATE_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮反转功能函数
CmdLineFuncRsp strip_backrotate_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	vector<string> fields = JobParser::GetCommandFields(line);
	if (fields.size() != 2)
	{
		ROS_ERROR_STREAM("Expected " << 2 - 1 << " arguments, but got " << fields.size() - 1);
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}

	int time_ms;
	try
	{
		time_ms = std::stoi(fields[1]);
	}
	catch (const std::exception &e)
	{
		ROS_ERROR_STREAM("Cannot convert argument \"" << fields[1] << "\" to int time");
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}
	if (time_ms < 0)
	{
		ROS_ERROR("Argument INT_TIME should not be negative");
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}

	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(),CMD_MSG_CLAMP_WIRE_TOOL_STRIP_BACKROTATE_SEQ, 1);

	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_BACKROTATE_START,"info");
    
	if (time_ms > 0) // 延时一小会，发送停止指令
	{
		ROS_INFO_STREAM("Sleeping " << time_ms << " ms to send STRIP_STOP");
		std::this_thread::sleep_for(chrono::milliseconds(time_ms));
		send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_STOP_SEQ, 1);
	}

	// 等待动作执行结束
    ROS_INFO("[STRIP_BACKROTATE IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_back_rotate_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_BACK_ROTATE_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_BACKROTATE_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮开口向上指令功能函数
CmdLineFuncRsp strip_upward_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_UPWARD_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_UPWARD_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_UPWARD IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_upward_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_UPWARD_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_UPWARD_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮推杆左指令功能函数
CmdLineFuncRsp strip_linkleft_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKLEFT_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_LINKLEFT_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_LINKLEFT IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_link_left_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_LEFT_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_LINKLEFT_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮推杆右指令功能函数
CmdLineFuncRsp strip_linkright_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKRIGHT_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_LINKRIGHT_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_LINKRIGHT IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_link_right_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_RIGHT_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_LINKRIGHT_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮推杆上指令功能函数
CmdLineFuncRsp strip_linkup_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKUP_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_LINKUP_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_LINKUP IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_link_up_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_UP_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_LINKUP_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮推杆下指令功能函数
CmdLineFuncRsp strip_linkdown_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKDOWM_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_LINKDOWN_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_LINKDOWN IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_link_down_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_DOWN_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_LINKDOWN_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剥皮停止指令功能函数
CmdLineFuncRsp strip_stop_line_func(SegmentScheduler &scheduler, const std::string &line)
{
	send_command_to_strip_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAMP_WIRE_TOOL_STRIP_STOP_SEQ, 1);
	send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,STRIP_STOP_START,"info");
   
	// 等待动作执行结束
    ROS_INFO("[STRIP_STOP IS WAITING FOR RESULT]");

    auto& event = scheduler.strip_wire_tool_strip_stop_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_STRIP_WIRE_TOOL_STRIP_STOP_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,STRIP_STOP_TIMEOUT,"info");
   
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
	else if (event.get() == false)
	{
		return CMD_LINE_FUNC_RSP_FAILED;
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}
