#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"

#include <vector>
#include <cassert>
#include <chrono>
#include <thread>
#include <map>
#include <future>

using namespace std;

extern std::shared_ptr<SegmentScheduler> scheduler;

// 将命令行返回值类型转为字符串
std::string to_string(const CmdLineFuncRsp rsp)
{
    static std::map<CmdLineFuncRsp, string> cmd_rsp_to_str = {
        { CMD_LINE_FUNC_RSP_SUCCESS,        "SUCCESS" },
        { CMD_LINE_FUNC_RSP_FAILED,         "FAILED" },
        { CMD_LINE_FUNC_RSP_SYNTAX_ERROR,   "SYNTAX ERROR" },
        { CMD_LINE_FUNC_RSP_TIMEOUT,        "TIMEOUT" },
    };

    assert (cmd_rsp_to_str.find(rsp) != cmd_rsp_to_str.cend());
    return cmd_rsp_to_str.at(rsp);
}

CmdLineFuncRsp waittime_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command WAITTIME");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"正在执行等待延时指令","info");
    


    vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 2)
    {
        ROS_ERROR_STREAM("expected " << 2-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    try {
        const int time_ms = std::stoi(fields[1]);

        ROS_INFO_STREAM("Wait " << time_ms << " milliseconds");
        std::this_thread::sleep_for(chrono::milliseconds(time_ms));    
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("cannot convert argument \"" << fields[1] << "\" to int time");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp manually_confirm_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() < 2)
    {
        ROS_ERROR_STREAM("expected " << 2-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    std::string manually_confirm_parameter = fields[1];
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,manually_confirm_parameter,"info");
    

    // 进入手工确认模式（作用域结束时会自动退出手工模式）
    SegmentScheduler::ManuallyOperationScope scope(scheduler);

    ROS_INFO("Waiting for manually confirm");
    if (!scheduler.waitManuallyConfirm())
    {
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

// 复位工具
CmdLineFuncRsp tool_reset_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"正在复位工具","info");
    
    const unsigned int time_seconds = 5;

    auto reset_tool = [&] (SegmentScheduler::CmdLineFunc reset_func, BoolEvent& event,
                                    const string& tool_name) -> CmdLineFuncRsp {
        ROS_INFO_STREAM("[" << tool_name << "] Checking connection status");
        if (event.wait_for(chrono::seconds(time_seconds)))
        {
            if (event.get() == true)
            {
                ROS_INFO_STREAM("[" << tool_name << "] Connection ok, reseting");
                const CmdLineFuncRsp response = reset_func(scheduler, line);
                if (response == CMD_LINE_FUNC_RSP_SUCCESS)
                {
                    ROS_INFO_STREAM("[" << tool_name << "] " << to_string(response));
                }
                else
                {
                    ROS_ERROR_STREAM("[" << tool_name << "] " << to_string(response));
                }
                return response;
            }
            else
            {
                ROS_ERROR_STREAM("[" << tool_name << "] Interrupted by outer request");
                return CMD_LINE_FUNC_RSP_FAILED;
            }
        }
        ROS_INFO_STREAM("[" << tool_name << "] Connection not ok, skipped");
        return CMD_LINE_FUNC_RSP_SUCCESS;
    };

    // 存储指令运行结果
    vector<future<CmdLineFuncRsp>> results(4);

    // 如果剥线工具启用，执行剥线工具初始化指令
    results[0] = std::async(std::launch::async, [&] () {
                    return reset_tool(strip_init_line_func,
                                      scheduler.strip_wire_tool_connection_state_flag_, "strip wire tool");
                 });

    // 如果接线工具启用，执行套筒锁定
    results[1] = std::async(std::launch::async, [&] () {
                    return reset_tool(connect_sleevelock_line_func,
                                      scheduler.clamp_wire_tool_connection_state_flag_, "clamp wire tool");
                 });

    // 如果手爪工具启用，执行手爪打开指令
    results[2] = std::async(std::launch::async, [&] () {
                    return reset_tool(open_gripper_line_func,
                                      scheduler.claw_tool_connection_state_flag_, "claw tool");
                 });

    // 如果断线工具启用，执行钳口复位指令
    results[3] = std::async(std::launch::async, [&] () {
                    return reset_tool(cut_init_line_func,
                                      scheduler.cut_wire_tool_connection_state_flag_, "cut wire tool");
                 });

    // 检查指令运行结果
    CmdLineFuncRsp response = CMD_LINE_FUNC_RSP_SUCCESS;
    for_each(results.begin(), results.end(), [&] (future<CmdLineFuncRsp>& result) {
        const CmdLineFuncRsp sub_task_response = result.get();
        if (sub_task_response != CMD_LINE_FUNC_RSP_SUCCESS)
        {
            response = sub_task_response;
        }
    });
    if (response == CMD_LINE_FUNC_RSP_FAILED)
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,"复位末端工具失败","info");
    
    }
    

    return response;
}

