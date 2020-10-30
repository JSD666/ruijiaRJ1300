#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

using namespace std;

// 接线拧断开始指令功能函数
CmdLineFuncRsp connect_tightenstart_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 2)
    {
        ROS_ERROR_STREAM("Expected " << 2-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int time_ms;
    try {
        time_ms = std::stoi(fields[1]);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Cannot convert argument \"" << fields[1] << "\" to int time");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    if (time_ms < 0)
    {
        ROS_ERROR("Argument INT_TIME should not be negative");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CLAMP_TIGHTEN_START,"info");
    

    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTART_SEQ, 1);

    if (time_ms > 0) // 延时一小会，发送停止指令
    {
		ROS_INFO_STREAM("Sleeping " << time_ms << " ms to send CONNECT_TIGHTENSTOP");
        std::this_thread::sleep_for(chrono::milliseconds(time_ms));
        send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                        CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTOP_SEQ, 1);
    }

    // 等待动作执行结束
    ROS_INFO("[CONNECT_TIGHTEN_START IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_tighten_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_TIGHTEN_START_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CLAMP_TIGHTEN_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线拧断停止指令功能函数
CmdLineFuncRsp connect_tightenstop_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTOP_SEQ, 1);

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线拧断反转指令功能函数
CmdLineFuncRsp connect_tightenback_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 2)
    {
        ROS_ERROR_STREAM("Expected " << 2-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int time_ms;
    try {
        time_ms = std::stoi(fields[1]);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Cannot convert argument \"" << fields[1] << "\" to int time");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    if (time_ms < 0)
    {
        ROS_ERROR("Argument INT_TIME should not be negative");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CLAMP_TIGHTENBACK_START,"info");
    

    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENBACK_SEQ, 1);

    if (time_ms > 0) // 延时一小会，发送停止指令
    {
		ROS_INFO_STREAM("Sleeping " << time_ms << " ms to send CONNECT_TIGHTENSTOP");
        std::this_thread::sleep_for(chrono::milliseconds(time_ms));
        send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                        CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTOP_SEQ, 1);
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线拧断初始化指令功能函数
CmdLineFuncRsp connect_tighteninit_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENINIT_SEQ, 1);

    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CLAMP_TIGHTENINIT_START,"info");
                                    

    // 等待动作执行结束
    ROS_INFO("[CONNECT_TIGHTEN_INIT IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_tighten_init_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_TIGHTEN_INIT_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CLAMP_TIGHTENINIT_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线线夹锁定指令功能函数
CmdLineFuncRsp connect_clip_lock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_CLIPLOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CLIP_LOCK_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CONNECT_CLIPLOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_clip_lock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_CLIP_LOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CLIP_LOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线线夹解锁指令功能函数
CmdLineFuncRsp connect_clipunlock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_CLIPUNLOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CLIP_UNLOCK_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CONNECT_CLIPUNLOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_clip_unlock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_CLIP_UNLOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CLIP_UNLOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线套筒锁定指令功能函数
CmdLineFuncRsp connect_sleevelock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVELOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,SLEEVE_LOCK_START,"info");
    
    ROS_INFO("[CONNECT_SLEEVELOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_sleeve_lock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_SLEEVE_LOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,SLEEVE_LOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线套筒解锁指令功能函数
CmdLineFuncRsp connect_sleeveunlock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVEUNLOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,SLEEVE_UNLOCK_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CONNECT_SLEEVEUNLOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_sleeve_unlock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_SLEEVE_UNLOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,SLEEVE_UNLOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线套筒停止指令功能函数
CmdLineFuncRsp connect_sleevestop_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVESTOP_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,SLEEVE_STOP_START,"info");
    
    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线支线夹紧指令功能函数
CmdLineFuncRsp connect_branchlinelock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINELOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,BRANCH_LOCK_START,"info");
    
    ROS_INFO("[CONNECT_BRANCHLINE_LOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_branchline_lock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINE_LOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,BRANCH_LOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 接线支线解锁指令功能函数
CmdLineFuncRsp connect_branchlineunlock_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_clamp_wire_tool(scheduler.getCmdMsgPub(),
                                    CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINEUNLOCK_SEQ, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,BRANCH_UNLOCK_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CONNECT_BRANCHLINE_UNLOCK IS WAITING FOR RESULT]");

    auto& event = scheduler.clamp_wire_tool_connect_branchline_unlock_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINE_UNLOCK_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,BRANCH_UNLOCK_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}