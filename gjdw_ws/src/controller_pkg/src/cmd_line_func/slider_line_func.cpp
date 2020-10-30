#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "ros/console.h"
#include "public_pkg/cmd_msg.h"
#include "public_pkg/write_srv.h"

#include <vector>
#include <cassert>
#include <chrono>
#include <thread>

using namespace std;

// 发送滑台运动指令
static void send_slider_move_cmd(ros::Publisher& pub,
    const int slider_object, const int seq, const double value, const char* name)
{
    public_pkg::cmd_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_SLIDER;
    msg.header.dstNodeType = NODE_TYPE_SLIDER;
    msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    msg.object = slider_object;

    const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %lf, \"name\": \"%s\"}]}",
                                    seq, "double", value, name);
    msg.content = content;
    pub.publish(msg);
}

// 发送滑台运动指令
static void send_slider_go_home_cmd(ros::Publisher& pub,
    const int slider_object, const int seq, const unsigned int value, const char* name)
{
    public_pkg::cmd_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_NAME_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_SLIDER;
    msg.header.dstNodeType = NODE_TYPE_SLIDER;
    msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    msg.object = slider_object;

    const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
                                    seq, "uint", value, name);
    msg.content = content;
    pub.publish(msg);
}

static CmdLineFuncRsp call_slider_write_srv(ros::ServiceClient& service_client,
    const int seq, const float value, const char* name)
{
    const string content = format("{\"cmd\": {\"seq\": %d, \"type\": \"%s\", \"value\": %.6f, \"name\": \"%s\"}}",
                                    seq, "float", value, name);

    ROS_INFO_STREAM(name << ": " << content);

    // public_pkg::write_srv srv;
    // srv.request.args = content;
    // if (!service_client.call(srv))
    // {
    //     ROS_ERROR("Failed to call service");
    //     return CMD_LINE_FUNC_RSP_FAILED;
    // }
    // const string rsp_content = srv.response.content;

    // 用于DEBUG
    const string rsp_content = "{\"cmdRsp\": {\"seq\": 1, \"type\": \"int32_t\", \"value\": 0, \"name\": \"SliderCmdRsp\", \"errCode\": 0}}";

    int err_code = 0;
    if (!get_cmd_rsp_errcode(rsp_content, err_code))
    {
        ROS_ERROR_STREAM("Failed to parse cmdRsp: " << rsp_content);
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    // 滑台控制服务出错
    if (err_code != 0)
    {
        ROS_ERROR_STREAM("Failed to control slider, error code: " << err_code);
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    return CMD_LINE_FUNC_RSP_SUCCESS;
}


CmdLineFuncRsp slider_move_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command SLIDERMOVE");

    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 3)
    {
        ROS_ERROR_STREAM("Expected " << 3-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int id;
    try {
        id = std::stoi(fields[1]);
        if (id != CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER && id != CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER)
        {
            ROS_ERROR_STREAM("Unrecognized ID: " << id);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
    } catch (...) {
        ROS_ERROR_STREAM("Argument ID invalid: " << fields[1]);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    float pos;
    try {
        pos = std::stof(fields[2]);
    } catch (...) {
        ROS_ERROR_STREAM("Argument SLIDERPOS invalid: " << fields[2]);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    Slider_moveabs_func(scheduler,id,pos);

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp slider_go_home_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command SLIDERGOHOME");

    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 2)
    {
        ROS_ERROR_STREAM("Expected " << 2-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int id;
    try {
        id = std::stoi(fields[1]);
        if (id != CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER && id != CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER)
        {
            ROS_ERROR_STREAM("Unrecognized ID: " << id);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
    } catch (...) {
        ROS_ERROR_STREAM("Argument ID invalid: " << fields[1]);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    switch (id)
    {
    case CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER:
        send_slider_go_home_cmd(scheduler.getCmdMsgPub(), CONTROL_OBJECT_HORIZONTAL_SLIDER,
                                COMMAND_CONTROL_SEQ_HORIZONTAL_SLIDER_GO_HOME, 0, "HorSliderHomming");
        ROS_INFO_STREAM("[HORIZONTAL SLIDER IS WAITING FOR GOHOME]");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,HSLIDER_HOME_START,"info");
    
        
        std::this_thread::sleep_for(chrono::milliseconds(500));
        {
            auto& event = scheduler.horizontal_slider_go_home_flag_;
            if (!event.wait_for(chrono::seconds(DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS)))
            {
                send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,HSLIDER_HOME_TIMEOUT,"info");
    
                return CMD_LINE_FUNC_RSP_TIMEOUT;
            }
            else if (event.get() == false)
            {
                return CMD_LINE_FUNC_RSP_FAILED;
            }
        }
        break;
    
    case CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER:
        send_slider_go_home_cmd(scheduler.getCmdMsgPub(), CONTROL_OBJECT_VERTICAL_SLIDER,
                                COMMAND_CONTROL_SEQ_VERTICAL_SLIDER_GO_HOME, 0, "VerSliderHomming");
        ROS_INFO_STREAM("[VERTICAL SLIDER IS WAITING FOR GOHOME]");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,VSLIDER_HOME_START,"info");
    
        std::this_thread::sleep_for(chrono::milliseconds(500));
        {
            auto& event = scheduler.vertical_slider_go_home_flag_;
            if (!event.wait_for(chrono::seconds(DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS)))
            {
                send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,VSLIDER_HOME_TIMEOUT,"info");
    
                return CMD_LINE_FUNC_RSP_TIMEOUT;
            }
            else if (event.get() == false)
            {
                return CMD_LINE_FUNC_RSP_FAILED;
            }
        }
        break;

    default:
        break;
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp slider_record_pos_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command RECORDSLIDERPOS");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待滑台位置确认信号","info");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),12,"主控服务器指示地面站开始确认水平滑台位置","info");
    VariableTable& var_table = scheduler.getVariableTable();

    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != 3)
    {
        ROS_ERROR_STREAM("Expected " << 3-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int id;
    try {
        id = std::stoi(fields[1]);
    } catch (...) {
        ROS_ERROR_STREAM("Argument ID invalid: " << fields[1]);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    string var_name, var_typename;
    JobParser::ParseVariableDefine(fields[2], var_name, var_typename);

    // 检查变量类型是否符合定义
    VariablePointer& vp = var_table.at(var_name);
    if (vp->getTypeName() != var_typename)
    {
        ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    if (vp->getType() != Variable::VARIABLE_TYPE_DOUBLE)
    {
        ROS_ERROR_STREAM("Variable type should be INT intread of : " << vp->getTypeName());
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int cmd_id;
    string slider_name;
    std::function<double()> get_slider_pos;

    switch (id)
    {
    case CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER:
        cmd_id = CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_POS;
        slider_name = "horizontal slider";
        get_slider_pos = std::bind(&SegmentScheduler::getHorizontalSliderPos, &scheduler);
        break;
    
    case CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER:
        cmd_id = CONTROL_COMMAND_ID_VERTICAL_SLIDER_POS;
        slider_name = "vertical slider";
        get_slider_pos = std::bind(&SegmentScheduler::getVerticalSliderPos, &scheduler);
        break;
    
    default:
        ROS_ERROR_STREAM("Unknown slider ID: " << id);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 发送滑台选点信号给地面站
    ROS_INFO("Sending %s's position selection signal to ground station", slider_name.c_str());
    send_command_to_ground_station(scheduler.getStatusDigitalMsgPub(), cmd_id, 1);

    // 进入手工确认模式（作用域结束时会自动退出手工模式）
    SegmentScheduler::ManuallyOperationScope scope(scheduler);

    // 等待滑台位置确认信号
    ROS_INFO("Waiting for %s's position confirm signal", slider_name.c_str());

    auto& event = scheduler.slider_pos_confirm_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS)))
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,SLIDER_RECORDPOS_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    // 读取滑台位置数据并写入变量
    const double pos = get_slider_pos();
    vp->setValue<DoubleValue>(pos);
    //vp->setValue<IntValue>(static_cast<int>(pos));
    ROS_INFO_STREAM("Assigned " << pos << " to variable: " << *vp);

    return CMD_LINE_FUNC_RSP_SUCCESS;
}


CmdLineFuncRsp Slider_moveabs_func(SegmentScheduler& scheduler,int id,double pos)
{
  
switch (id)
    {
    case CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER:
        send_slider_move_cmd(scheduler.getCmdMsgPub(), CONTROL_OBJECT_HORIZONTAL_SLIDER,
                             COMMAND_CONTROL_SEQ_HORIZONTAL_SLIDER_MOVE, pos, "HorSliderGo");
        ROS_INFO_STREAM("[HORIZONTAL SLIDER IS WAITING FOR REACH]");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,HSLIDER_MOVE_START,"info");
    
        std::this_thread::sleep_for(chrono::milliseconds(500));
        {
            auto& event = scheduler.horizontal_slider_reach_flag_;
            if (!event.wait_for(chrono::seconds(DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS)))
            {
                send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,HSLIDER_MOVE_TIMEOUT,"info");
    
                return CMD_LINE_FUNC_RSP_TIMEOUT;
            }
            else if (event.get() == false)
            {
                return CMD_LINE_FUNC_RSP_FAILED;
            }
        }
        break;
    
    case CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER:
        send_slider_move_cmd(scheduler.getCmdMsgPub(), CONTROL_OBJECT_VERTICAL_SLIDER,
                             COMMAND_CONTROL_SEQ_VERTICAL_SLIDER_MOVE, pos, "VerSliderGo");
        ROS_INFO_STREAM("[VERTICAL SLIDER IS WAITING FOR REACH]");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,VSLIDER_MOVE_START,"info");
    
        std::this_thread::sleep_for(chrono::milliseconds(500));
        {
            auto& event = scheduler.vertical_slider_reach_flag_;
            if (!event.wait_for(chrono::seconds(DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS)))
            {
                send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,VSLIDER_MOVE_TIMEOUT,"info");
    
                return CMD_LINE_FUNC_RSP_TIMEOUT;
            }
            else if (event.get() == false)
            {
                return CMD_LINE_FUNC_RSP_FAILED;
            }
        }
        break;

    default:
        break;
    }
    return CMD_LINE_FUNC_RSP_SUCCESS;
}