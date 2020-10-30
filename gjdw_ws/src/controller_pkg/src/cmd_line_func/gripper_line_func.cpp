#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

using namespace std;

// 打开夹爪指令功能函数
CmdLineFuncRsp open_gripper_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_claw_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAW_TOOL_RESET_GRIPPER, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,GRIPPER_OPEN_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[OPENGRIPPER IS WAITING FOR RESULT]");

    auto& event = scheduler.claw_tool_gripper_open_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAW_TOOL_GRIPPER_OPEN_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,GRIPPER_OPEN_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 关闭夹爪指令功能函数
CmdLineFuncRsp close_gripper_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_command_to_claw_tool(scheduler.getCmdMsgPub(), CMD_MSG_CLAW_TOOL_CLOSE_GRIPPER, 1);
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,GRIPPER_CLOSE_START,"info");
    
    // 等待动作执行结束
    ROS_INFO("[CLOSEGRIPPER IS WAITING FOR RESULT]");

    auto& event = scheduler.claw_tool_gripper_close_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_CLAW_TOOL_GRIPPER_CLOSE_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,GRIPPER_CLOSE_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}