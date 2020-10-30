#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

using namespace std;

// 剪线开始指令功能函数
CmdLineFuncRsp cut_start_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_cut_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CUT_WIRE_TOOL_CUT_START, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CUT_WIRE_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CUT_START IS WAITING FOR RESULT]");

    auto& event = scheduler.cut_wire_tool_cut_result_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CUT_WIRE_TOOL_CUT_START_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CUT_WIRE_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 剪线复位指令功能函数
CmdLineFuncRsp cut_init_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_cut_wire_tool(scheduler.getCmdMsgPub(), CMD_MSG_CUT_WIRE_TOOL_CUT_RESET, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,CUT_INIT_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CUT_INIT IS WAITING FOR RESULT]");

    auto& event = scheduler.cut_wire_tool_cut_init_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CUT_WIRE_TOOL_CUT_INIT_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,CUT_INIT_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}