#include "call_back.h"
#include "SegmentScheduler.h"
#include "config.h"
#include "utils.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h"

#include "ros/console.h"

#include <memory>

using namespace std;

bool arm_emergency_flaged = false;
bool l_arm_error_flaged = false, r_arm_error_flaged = false;
bool l_arm_protected_flaged = false, r_arm_protected_flaged = false;
bool l_arm_connection_flaged = false, r_arm_connection_flaged = false;
bool l_arm_enable_flaged = false, r_arm_enable_flaged = false;
bool l_arm_running_flaged = false, r_arm_running_flaged = false;
bool l_arm_paused_flaged = false, r_arm_paused_flaged = false;

// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;

// 监听通用数字量信息
void status_digital_call_back(const public_pkg::status_digital_msg &msg)
{
    if (msg.header.msgType != MSG_TYPE_STATUS_DIGITAL_MSG)
    {
        ROS_ERROR_STREAM("status_digital_call_back: drop one message because type mismatch. "
                         "msgType should be "
                         << MSG_TYPE_STATUS_DIGITAL_MSG << ", instead of " << msg.header.msgType);
        return;
    }

    if (msg.header.srcNodeName == NODE_NAME_CONTROLLER)
    {
        return;
    }

    // 消息频率高，因此使用DEBUG接口
    ROS_DEBUG("status_digital_call_back: received content: \"%s\"", msg.content.c_str());

    // 过滤自己发的包

    rapidjson::Document d;
    d.Parse(msg.content.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("status_digital_call_back: received content invalid");
        return;
    }

    const rapidjson::Value &digitalArray = d["digitalArray"];
    if (!digitalArray.IsArray())
    {
        ROS_ERROR("status_digital_call_back: received content invalid, cannot parse digitalArray");
        return;
    }

    { // 等待设备遥信到达设定值
        auto &event = scheduler->wait_yx_signal_arrive;
        if (msg.header.srcNodeName == event.signal_device && msg.header.srcNodeType == event.signal_device)
        {
            for (size_t i = 0; i < digitalArray.Size(); ++i)
            {
                const rapidjson::Value &seq = digitalArray[i]["seq"];
                if (!seq.IsInt())
                {
                    ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                    return;
                }
                const rapidjson::Value &value = digitalArray[i]["value"];
                if (!value.IsInt())
                {
                    ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                    return;
                }
                if (value != 1)
                    continue;

                if (seq.GetInt() == event.signal_seq)
                {
                    if (event.isWaiting())
                    {
                        if (value.GetInt() == event.signal_value)
                        {
                            event.notify(value.GetInt());
                        }
                    }
                }
            }
        }
    }

    // 来自滑台结点的消息
    if (msg.header.srcNodeName == NODE_NAME_SLIDER && msg.header.srcNodeType == NODE_TYPE_SLIDER)
    {
        //ROS_INFO_STREAM("SLIDER msg content: " << msg.content);
        for (size_t i = 0; i < digitalArray.Size(); ++i)
        {
            const rapidjson::Value &seq = digitalArray[i]["seq"];
            if (!seq.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                return;
            }
            const rapidjson::Value &value = digitalArray[i]["value"];
            if (!value.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                return;
            }

            if (value != 1)
                continue;

            switch (seq.GetInt())
            {
            case GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_REACH_FLAG_SEQ: // 水平滑台位置到位信号
            {
                auto &event = scheduler->horizontal_slider_reach_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Notifying response to horizontal slider reach");
                    event.notify(true);
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_GO_HOME_FLAG_SEQ: // 水平滑台零位到位信号
            {
                auto &event = scheduler->horizontal_slider_go_home_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Notifying response to horizontal slider go home");
                    event.notify(true);
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_REACH_FLAG_SEQ: // 垂直滑台位置到位信号
            {
                auto &event = scheduler->vertical_slider_reach_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Notifying response to vertical slider reach");
                    event.notify(true);
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_GO_HOME_FLAG_SEQ: // 垂直滑台零位到位信号
            {
                auto &event = scheduler->vertical_slider_go_home_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Notifying response to vertical slider go home");
                    event.notify(true);
                }
            }
            break;

            default:
                break;
            }
        }
    }
    // 来自末端接线工具结点的消息
    else if (msg.header.srcNodeName == NODE_NAME_CLAMP_WIRE_TOOL && msg.header.srcNodeType == NODE_TYPE_CLAMP_WIRE_TOOL)
    {
        for (size_t i = 0; i < digitalArray.Size(); ++i)
        {
            const rapidjson::Value &seq = digitalArray[i]["seq"];
            if (!seq.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                return;
            }
            const rapidjson::Value &value = digitalArray[i]["value"];
            if (!value.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                return;
            }

            switch (seq.GetInt())
            {
            // 接线工具连接状态
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG:
            {
                auto &event = scheduler->clamp_wire_tool_connection_state_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("Clamp wire tool connection: " << value.GetInt());

                    if (value.GetInt() == 1)
                    {
                        ROS_INFO("Notifying connection success to clamp wire tool");
                        event.notify(true);
                    }
                    else
                    {
                        ROS_INFO("Notifying connection failure to clamp wire tool");
                        event.notify(false);
                    }
                }
            }
            break;

            // 拧断电机状态遥信点
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGHTEN_MOTOR_STATE:
            {
                auto &event = scheduler->clamp_wire_tool_connect_tighten_result_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("Tighten_start: " << value.GetInt());

                    if (value.GetInt() == 3)
                    {
                        ROS_INFO("Notifying tighten start success to clamp wire tool");
                        event.notify(true);
                    }
                    else if (value.GetInt() == 2)
                    {
                        ROS_INFO("Notifying tighten start failure to clamp wire tool");
                        event.notify(false);
                    }
                }
            }
            break;

            // 拧断电机复位遥信点
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGHTEN_MOTOR_INIT:
            {
                auto &event = scheduler->clamp_wire_tool_connect_tighten_init_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("Tighten_init: " << value.GetInt());

                    if (value.GetUint() == 1)
                    {
                        ROS_INFO("Notifying tighten init to clamp wire tool");
                        event.notify(true);
                    }
                }
            }
            break;

            // 线夹锁定状态遥信点
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_CLIP_LOCK_STATE:
            {
                auto &event = scheduler->clamp_wire_tool_connect_clip_lock_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("clip_lock: " << value.GetInt());

                    if (value.GetUint() == 0)
                    {
                        ROS_INFO("Notifying clip lock to clamp wire tool");
                        event.notify(true);
                    }
                }
            }
                {
                    auto &event = scheduler->clamp_wire_tool_connect_clip_unlock_flag_;
                    if (event.isWaiting())
                    {
                        // ROS_INFO_STREAM("clip_unlock: " << value.GetInt());

                        if (value.GetUint() == 1)
                        {
                            ROS_INFO("Notifying clip unlock to clamp wire tool");
                            event.notify(true);
                        }
                    }
                }
                break;

            // 套筒锁定状态遥信点
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_SLEEVE_LOCK_STATE:
            {
                auto &event = scheduler->clamp_wire_tool_connect_sleeve_lock_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("sleeve_lock: " << value.GetInt());

                    if (value.GetUint() == 3)
                    {
                        ROS_INFO("Notifying sleeve lock success to clamp wire tool");
                        event.notify(true);
                    }
                    else if (value.GetUint() == 2)
                    {
                        ROS_INFO("Notifying sleeve lock false to clamp wire tool");
                        event.notify(false);
                    }
                }
            }
                {
                    auto &event = scheduler->clamp_wire_tool_connect_sleeve_unlock_flag_;
                    if (event.isWaiting())
                    {
                        // ROS_INFO_STREAM("sleeve_unlock: " << value.GetInt());

                        if (value.GetUint() == 1)
                        {
                            ROS_INFO("Notifying sleeve unlock to clamp wire tool");
                            event.notify(true);
                        }
                    }
                }
                break;

            // 支线夹紧状态遥信点
            case GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_IN_BRANCHLINE_LOCK_STATE:
            {
                auto &event = scheduler->clamp_wire_tool_connect_branchline_lock_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("branchline_lock: " << value.GetInt());

                    if (value.GetUint() == 3)
                    {
                        ROS_INFO("Notifying branchline lock to clamp wire tool");
                        event.notify(true);
                    }
                }
            }
                {
                    auto &event = scheduler->clamp_wire_tool_connect_branchline_unlock_flag_;
                    if (event.isWaiting())
                    {
                        // ROS_INFO_STREAM("branchline_unlock: " << value.GetInt());

                        if (value.GetUint() == 1)
                        {
                            ROS_INFO("Notifying branchline unlock to clamp wire tool");
                            event.notify(true);
                        }
                    }
                }
                break;

            default:
                break;
            }
        }
    }
    // 来自末端剪线工具结点的消息
    else if (msg.header.srcNodeName == NODE_NAME_CUT_WIRE_TOOL && msg.header.srcNodeType == NODE_TYPE_CUT_WIRE_TOOL)
    {
        // ROS_INFO_STREAM("\nContent from cut wire tool: " << msg.content << "\n");

        for (size_t i = 0; i < digitalArray.Size(); ++i)
        {
            const rapidjson::Value &seq = digitalArray[i]["seq"];
            if (!seq.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                return;
            }
            const rapidjson::Value &value = digitalArray[i]["value"];
            if (!value.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                return;
            }

            switch (seq.GetInt())
            {
            // 剪线工具连接状态
            case GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG:
            {
                auto &event = scheduler->cut_wire_tool_connection_state_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("Cut wire tool connection: " << value.GetInt());

                    if (value.GetInt() == 1)
                    {
                        ROS_INFO("Notifying connection success to cut wire tool");
                        event.notify(true);
                    }
                    else
                    {
                        ROS_INFO("Notifying connection failure to cut wire tool");
                        event.notify(false);
                    }
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_CUT_STATE:
            {
                auto &event = scheduler->cut_wire_tool_cut_result_flag_;
                if (event.isWaiting() && value.GetUint() == 2)
                {
                    ROS_INFO("Notifying cut success to cut wire tool");
                    event.notify(true);
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_CUT_MOTOR_INIT_STATE:
            {
                auto &event = scheduler->cut_wire_tool_cut_init_flag_;
                if (event.isWaiting() && value.GetUint() == 1)
                {
                    ROS_INFO("Notifying cut init to cut wire tool");
                    event.notify(true);
                }
            }
            break;

            default:
                break;
            }
        }
    }
    // 来自末端夹爪工具结点的消息
    else if (msg.header.srcNodeName == NODE_NAME_CLAW_TOOL && msg.header.srcNodeType == NODE_TYPE_CLAW_TOOL)
    {
        // ROS_INFO("Received content from claw tool: \"%s\"", msg.content.c_str());

        for (size_t i = 0; i < digitalArray.Size(); ++i)
        {
            const rapidjson::Value &seq = digitalArray[i]["seq"];
            if (!seq.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                return;
            }
            const rapidjson::Value &value = digitalArray[i]["value"];
            if (!value.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                return;
            }

            switch (seq.GetInt())
            {
            // 夹爪工具连接状态
            case GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG:
            {
                auto &event = scheduler->claw_tool_connection_state_flag_;
                if (event.isWaiting())
                {
                    // ROS_INFO_STREAM("Claw tool connection: " << value.GetInt());

                    if (value.GetInt() == 1)
                    {
                        ROS_INFO("Notifying connection success to claw tool");
                        event.notify(true);
                    }
                    else
                    {
                        ROS_INFO("Notifying connection failure to claw tool");
                        event.notify(false);
                    }
                }
            }
            break;

            case GENERAL_DIGITAL_MSG_SEQ_CLAW_TOOL_GRIPPER_STATE:
            {
                auto &event = scheduler->claw_tool_gripper_open_flag_;
                if (event.isWaiting() && value.GetUint() == 0)
                {
                    ROS_INFO("Notifying gripper open to claw wire tool");
                    event.notify(true);
                }
            }
                {
                    auto &event = scheduler->claw_tool_gripper_close_flag_;
                    if (event.isWaiting() && value.GetUint() == 1)
                    {
                        ROS_INFO("Notifying gripper close to claw wire tool");
                        event.notify(true);
                    }
                }
                break;

            default:
                break;
            }
        }
    }
    // 来自末端剥线工具结点的消息
    else if (msg.header.srcNodeName == NODE_NAME_STRIP_WIRE_TOOL && msg.header.srcNodeType == NODE_TYPE_STRIP_WIRE_TOOL)
    {
        if (digitalArray.Size() < GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_MAXIMUM_SEQ)
        {
            ROS_ERROR_STREAM("digitalArray (size " << digitalArray.Size() << ") from " << msg.header.srcNodeName
                                                   << " too small, should be" << GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_MAXIMUM_SEQ);
            return;
        }
        // 检查各个标志位是否需要通知

        {
            // 检查剥线工具连接状态
            auto &event = scheduler->strip_wire_tool_connection_state_flag_;
            if (event.isWaiting())
            {
                const unsigned int value = digitalArray[GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG - 1]["value"].GetUint();
                if (value == 1)
                {
                    ROS_INFO("Notifying connection success to strip wire tool");
                    event.notify(true);
                }
                else
                {
                    ROS_INFO("Notifying connection failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_onekey_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_ONEKEY && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying onekey success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying onekey failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_init_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying init success to strip wire tool");
                    event.notify(true);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_rotate_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action != GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying rotate success to strip wire tool");
                    event.notify(true);
                }
                else
                {
                    ROS_INFO("Notifying rotate failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_back_rotate_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action != GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying rotate success to strip wire tool");
                    event.notify(true);
                }
                else
                {
                    ROS_INFO("Notifying rotate failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_upward_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();
                //ROS_INFO_STREAM("************ ");
                //ROS_INFO_STREAM("upward  action: " << action  << "  upward  state: " << state);
                //ROS_INFO_STREAM("************ ");

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_UPWARD && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying upward success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying upward failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_link_left_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKLEFT && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying linkleft success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying linkleft failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_link_right_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKRIGHT && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying linkright success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying linkright failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_link_up_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKUP && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying linkup success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying linkup failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_link_down_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKDOWN && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED)
                {
                    ROS_INFO("Notifying linkdown success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying linkdown failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
        {
            auto &event = scheduler->strip_wire_tool_strip_stop_result_flag_;
            if (event.isWaiting())
            {
                const unsigned int action = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG - 1]["value"].GetUint();
                const unsigned int state = digitalArray[GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG - 1]["value"].GetUint();

                // ROS_INFO_STREAM("Action: " << action << ", state: " << state);

                if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_NO_MOTION && state == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_NO_MOTION)
                {
                    ROS_INFO("Notifying stop success to strip wire tool");
                    event.notify(true);
                }
                else if (action == GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT)
                {
                    ROS_INFO("Notifying stop failure to strip wire tool");
                    event.notify(false);
                }
            }
        }
    }

    // 来自电源管理遥信消息机械臂连接状态
    else if (msg.header.srcNodeName == NODE_NAME_ARM_status_yx && msg.header.srcNodeType == NODE_TYPE_ARM_status_yx)
    {
        // static bool arm_emergency_flaged = false;// 注：此处初始化会回调一次初始化一次
        for (size_t i = 0; i < digitalArray.Size(); ++i)
        {
            const rapidjson::Value &seq = digitalArray[i]["seq"];
            if (!seq.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse seq of digitalArray");
                return;
            }
            const rapidjson::Value &value = digitalArray[i]["value"];
            if (!value.IsInt())
            {
                ROS_ERROR("status_digital_call_back: received content invalid, cannot parse value of digitalArray");
                return;
            }

            switch (seq.GetInt())
            {
            //由于电源板上电，机械臂立即连接成功，因此不判定连接状态也是可以的
            case LEFT_ARM_CONNECTED: // 电源管理）左臂连接成功标志
            {
                auto &event = scheduler->left_arm_connection_flag_;
                if (value == 1)
                {
                    if (event.isWaiting())
                    {
                        ROS_INFO("Left_arm_connection_success,Notifying left_arm_power_on");
                        event.notify(LEFT_ARM_CONNECTED); //通知左臂上电
                    }
                    l_arm_connection_flaged = true;
                }
                else
                {
                    l_arm_connection_flaged = false;
                }

                break;
            }

            case LEFT_ARM_ENABLED: // 电源管理）
            {
                if (value == 1)
                {
                    l_arm_enable_flaged = false;

                    auto &event = scheduler->left_arm_enable_flag_;
                    if (event.isWaiting())
                    {
                        event.notify(LEFT_ARM_ENABLED); //通知左臂电机抱闸
                    }
                }
                else
                {
                    if (!l_arm_enable_flaged) // false
                    {
                        //伺服使能未打开
                        l_arm_enable_flaged = true;
                    }
                }

                break;
            }

            case LEFT_ARM_ERROR: // 电源管理）//机械臂报错
            {
                if (value == 1)
                {
                    if ((!l_arm_error_flaged) && (!arm_emergency_flaged)) // && (!l_arm_protected_flaged))
                    {
                        ROS_ERROR("error: left arm occurred error");
                        //send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),16,"错误-机械臂1处于故障状态，请人工解除后继续运行！-转人工控制","info");
                    }
                    l_arm_error_flaged = true;
                }
                else
                {
                    l_arm_error_flaged = false;
                }

                break;
            }

            case LEFT_ARM_POWER_ON: // 电源管理）通知左臂上电
            {
                if (value == 1)
                {
                    auto &event = scheduler->left_arm_power_on_flag_;
                    if (event.isWaiting())
                    {
                        ROS_INFO("left arm already power on,ready to release break");
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 13, "主臂已上电，开始准备解除抱闸", "info");
                        event.notify(LEFT_ARM_POWER_ON); //通知左臂打开电机抱闸
                    }
                }

                break;
            }

            //case LEFT_ARM_EMERGENCY_STOPPED: // 电源管理）
            case RIGHT_ARM_EMERGENCY_STOPPED:
            {
                if (value == 1)
                {
                    if (!arm_emergency_flaged) // false
                    {
                        ROS_ERROR("Warn: arm emergency stopped");
                        if (!scheduler->isIdle()) // 急停状态暂停作业
                        {
                            scheduler->requestToPause();
                        }
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 16, "错误-机械臂处于急停状态，请人工解除后继续运行！-转人工控制", "error");
                    }
                    arm_emergency_flaged = true;
                }
                else
                {
                    auto &event = scheduler->arm_emergency_stopped_flag_;

                    if (event.isWaiting())
                    {
                        ROS_INFO("Notifying arm_emergency_stopped");
                        event.notify(LEFT_ARM_EMERGENCY_STOPPED); //通知机械臂上电
                    }

                    if (arm_emergency_flaged) // false
                    {
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 13, "急停状态已解除，开始准备上电");
                        arm_emergency_flaged = true;
                        ROS_INFO("Notifying arm_emergency_stopped_release");
                    }
                    arm_emergency_flaged = false;
                }

                break;
            }

            case LEFT_ARM_PROTECTIVE_STOPPED: // 左臂保护性停止状态
            {
                if (value == 1)
                {
                    if (!l_arm_protected_flaged)
                    {
                        l_arm_protected_flaged = true;
                        ROS_ERROR("Warn: left arm protective stoppec");
                        if (!scheduler->isIdle()) // 保护性停止状态暂停作业
                        {
                            scheduler->requestToPause();
                        }
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 15, "警告-机械臂1处于保护性停止状态，请人工解除后继续运行！-转人工控制", "warn");
                    }

                    auto &event = scheduler->left_arm_protective_stop_flag_;
                    if (event.isWaiting())
                    {
                        ROS_INFO("Notifying left_arm_protective_stop_flag true ");
                        event.notify(true);
                    }
                }
                else
                {
                    if (l_arm_protected_flaged)
                    {
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 13, "警告-主臂已解除保护性停止状态！", "info");
                    }

                    l_arm_protected_flaged = false;
                }

                break;
            }

            case LEFT_ARM_PROGRAM_RUNNING: // 电源管理）主(左)臂程序运行
            {
                break;
            }

            case LEFT_ARM_PROGRAM_PAUSED: // 电源管理）暂停
            {
                break;
            }
            //由于电源板上电，机械臂立即连接成功，因此不判定连接状态也是可以的
            case RIGHT_ARM_CONNECTED: // 电源管理）右臂连接成功标志
            {
                auto &event = scheduler->right_arm_connection_flag_;
                if (value == 1)
                {
                    if (event.isWaiting())
                    {
                        ROS_INFO("Notifying right_arm_connection");
                        //send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"机械臂2连接正常！","info");
                        event.notify(RIGHT_ARM_CONNECTED); //通知右臂
                    }
                }
                break;
            }

            case RIGHT_ARM_ENABLED: // 电源管理）
            {
                if (value == 1)
                {
                    r_arm_enable_flaged = false;
                    auto &event = scheduler->right_arm_enable_flag_;
                    if (event.isWaiting())
                    {
                        ROS_INFO("Notifying right_arm__eable");

                        event.notify(RIGHT_ARM_ENABLED); //通知从臂电机伺服已打开
                    }
                }
                else
                {
                    if (!r_arm_enable_flaged) // false
                    {
                        //伺服使能未打开
                        r_arm_enable_flaged = true;
                    }
                }

                break;
            }

            case RIGHT_ARM_POWER_ON: // 电源管理）通知右臂上电
            {
                if (value == 1)
                {
                    auto &event = scheduler->right_arm_power_on_flag_;
                    if (event.isWaiting())
                    {
                        ROS_INFO("right arm already power on,ready to release break");
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 13, "从臂已上电,开始准备解除抱闸", "info");
                        event.notify(RIGHT_ARM_POWER_ON); //通知右臂
                    }
                }
                break;
            }

            case RIGHT_ARM_ERROR: // 电源管理）//机械臂报错
            {
                if (value == 1)
                {
                    if ((!r_arm_error_flaged) && (!arm_emergency_flaged)) // && (!r_arm_protected_flaged))
                    {
                        ROS_ERROR("error: right arm occurred error");
                        //send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),16,"错误-机械臂1处于故障状态，请人工解除后继续运行！-转人工控制","info");
                    }
                    r_arm_error_flaged = true;
                }
                else
                {
                    r_arm_error_flaged = false;
                }
                break;
            }

            case RIGHT_ARM_PROTECTIVE_STOPPED: // 右臂保护性停止状态
            {
                if (value == 1)
                {
                    if (!r_arm_protected_flaged)
                    {
                        r_arm_protected_flaged = true;
                        ROS_ERROR("Warn: right arm protective stoppec");
                        if (!scheduler->isIdle()) // 保护性停止状态暂停作业
                        {
                            scheduler->requestToPause();
                        }
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 15, "警告-机从臂处于保护性停止状态，请人工解除后继续运行！-解除保护性停止|转人工控制", "warn");
                    }

                    auto &event = scheduler->right_arm_protective_stop_flag_;
                    if (event.isWaiting())
                    {
                        ROS_INFO("Notifying right_arm_protective_stop_flag");
                        event.notify(true); //
                    }
                    //std::this_thread::sleep_for(chrono::seconds(3));//
                }
                else
                {
                    if (r_arm_protected_flaged)
                    {
                        send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(), 13, "警告-从臂已解除保护性停止状态！", "info");
                    }

                    r_arm_protected_flaged = false;
                }

                break;
            }

            case RIGHT_ARM_PROGRAM_RUNNING: // 电源管理）从（右）臂程序运行
            {

                break;
            }

            case RIGHT_ARM_PROGRAM_PAUSED: // 电源管理）
            {
                break;
            }

            default:
                break;
            }
        }
    }

    // 来自机械臂结点的消息
    else if (msg.header.srcNodeName == NODE_NAME_ROBOT_Script && msg.header.srcNodeType == NODE_TYPE_ROBOT_Script)
    {
        // 目前不使用机械臂的遥信消息
    }
    else
    {
        ROS_DEBUG_STREAM("Skipped status_digital_msg, srcNodeName: " << msg.header.srcNodeName << ", srcNodeType: " << msg.header.srcNodeType);
    }
}