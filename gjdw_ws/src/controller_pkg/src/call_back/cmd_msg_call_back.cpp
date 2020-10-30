#include "call_back.h"
#include "SegmentScheduler.h"
#include "config.h"
#include "utils.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h"
#include "public_pkg/arms_power_srv.h"
#include "ros/console.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

using namespace std;

// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;
extern bool arm_emergency_flaged;
extern bool r_arm_protected_flaged;
extern bool l_arm_protected_flaged;
extern bool l_arm_enable_flaged;
extern bool r_arm_enable_flaged;

// 解析处理地面站发给主控结点请求控制主控的命令
static void control_master_controller(const std::string& cmd_str)
{
    creat_logfile(LOG_REMOTE_CONTROL, cmd_str.c_str());
    rapidjson::Document d;
    d.Parse(cmd_str.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("control_master_controller: received content invalid");
        return;
    }

    const rapidjson::Value& cmdArray = d["cmdArray"];
    if (!cmdArray.IsArray())
    {
        ROS_ERROR("control_master_controller: received content invalid, cannot parse cmdArray");
        return;
    }

    // 循环处理每一个命令
    for (size_t i = 0; i < cmdArray.Size(); ++i)
    {
        const rapidjson::Value& seq = cmdArray[i]["seq"];
        if (!seq.IsInt())
        {
            ROS_ERROR("control_master_controller: received content invalid, cannot parse seq of cmdArray");
            return;
        }

        // 除了开始报文,其他命令都回一个包
        if (seq.GetInt() != CONTROL_COMMAND_ID_BEGIN_JOB)
        {
            send_command_response_to_ground_station(scheduler->getCmdRspMsgPub(), seq.GetInt());
        }

        const rapidjson::Value& value = cmdArray[i]["value"];

        switch (seq.GetInt())
        {
        case CONTROL_COMMAND_ID_BEGIN_JOB:
            ROS_INFO("Remote ground station ask to begin job");
            // 如果已经开始作业，忽略本次请求
            if ((!scheduler->isIdle())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            
            {
                ROS_ERROR("A job is still running, the new job request will be discarded");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"已经开始作业或机械臂故障，忽略本次请求","info");
            }
            else
            {
                if (!value.IsArray())
                {
                    ROS_ERROR_STREAM("cmd_call_back: cannot parse value of cmdArray, seq: " << seq.GetInt());
                    return;
                }
                if (value.Size() < 10)
                {
                    ROS_ERROR("control_master_controller: start package should has atleast 10 elements");
                    return;
                }
                // if (value.Size() == 10) // 没有步序数据，不执行调度
                // {
                //     ROS_INFO("cmd_call_back: segment index empty, the shceduling will be skipped");
                //     return;
                // }

                string filename = string(JOB_FILE_DIRECTORY) + "/C";      // 作业文件名
                vector<int> segment_indexs;     // 需要运行的段序
                string partname[10];
                bool getToolMode = false;
                for (size_t i = 0; i < value.Size(); ++i)
                {
                    const unsigned int v = value[i].GetInt();
                    if (i < 10) // 前10个int决定调度文件名
                    {
                        if(getToolMode == false)
                            partname[i] = format("%02X", v);
                        else
                            partname[i] = "00";
                        
                    }
                       
                    else
                        segment_indexs.push_back(v);

                    // 激光雷达中使用
                    if(i == 1)
                    {
                        extern int ConnectType;
                        ConnectType=v;
                    }
                    if(i == 3)
                    {
                        if((v == 4)||(v == 5))//取工具模式
                        {
                            partname[2] = "00";
                            getToolMode = true;
                        }
                    }
                }

                for (size_t i = 0; i < 10; ++i)
                {
                    filename += partname[i];
                }

                filename += ".csv";

                // 需要提前调用SegmentScheduler::start()
                // 因为线程执行顺序是不确定的，start()后立刻schedule()可能线程还没start，导致同步信息丢失
                ROS_INFO_STREAM("cmd_call_back: scheduling file " << filename);
                if (!scheduler->schedule(filename, segment_indexs))
                {
                    ROS_ERROR("cmd_call_back: failed to schedule segment");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),16,"调度作业失败，请重新开始作业","info");
                }
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"作业调度成功，开始作业","info");
                
            }
            break;

        // 停止作业
        case CONTROL_COMMAND_ID_STOP_JOB:
        {
            auto& event = scheduler->wait_continue_flag_;
            if (event.isWaiting())
            {
                ROS_INFO("Not safe pos choose end job");
                event.notify(0);
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"系统空闲","info");
            
                break;
            }
            ROS_INFO("Remote ground station ask to stop job");

            if (!scheduler->isIdle())
            {
                scheduler->requestToStop();
                if (scheduler->isWaitingManuallyConfirm())
                {
                    scheduler->notifyManuallyConfirm();
                }
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"系统空闲","info");
                
            }
            else
            {
                ROS_INFO("But no job is scheduling, so the request will be discarded");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                scheduler->notifyPowerServiceQuitReach(0);
            }
            break;
        }

        // 暂停作业
        case CONTROL_COMMAND_ID_PAUSE_JOB:
            ROS_INFO("Remote ground station ask to pause job");
            if (!scheduler->isIdle())
            {
                scheduler->requestToPause();
                if (scheduler->isWaitingManuallyConfirm())
                {
                    scheduler->notifyManuallyConfirm();
                }
            }
            else
            {
                ROS_INFO("But no job is scheduling, so the request will be discarded");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                scheduler->notifyPowerServiceQuitReach(0);
            }
            break;

        // 继续作业
        case CONTROL_COMMAND_ID_CONTINUE_JOB:
            // 为安全着想，禁止在非手工模式下继续当前作业行
            if ((!scheduler->isInManuallyOperationMode())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            {
                ROS_ERROR("For safety, you won't be able to redo job while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"禁止在非手工模式下，或机械臂故障状态下继续当前作业行","info");
            }
            else
            {
                auto& event = scheduler->wait_continue_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Not safe pos but choose continue");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"不在安全位置，选择了继续运行作业文件","info");
            
                    event.notify(1);
                    break;
                }
                if (!scheduler->isIdle())//&&(l_arm_protected_flaged==false)&&(r_arm_protected_flaged==false))
                {
                    ROS_INFO("Remote ground station ask to continue job");
                    scheduler->requestToContinue();
                    if (scheduler->isWaitingManuallyConfirm())
                    {
                        scheduler->notifyManuallyConfirm();
                    }
                }
                else
                {
                    ROS_INFO("But no job is scheduling, so the request will be discarded");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                }

            }
            break;

        // 重新调度段
        case CONTROL_COMMAND_ID_RESCHEDULE_JOB:
            // 为安全着想，禁止在非手工模式下重新开始作业（或段）
            if ((!scheduler->isInManuallyOperationMode())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            {
                ROS_ERROR("For safety, you won't be able to redo job while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"禁止在非手工模式下或机械臂故障状态下，重新开始作业（或段），忽略本次请求","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to redo job");
                if (!scheduler->isIdle())
                {
                    scheduler->requestToRescheduleSegment();
                    if (scheduler->isWaitingManuallyConfirm())
                    {
                        scheduler->notifyManuallyConfirm();
                    }
                }
                else
                {
                    ROS_INFO("But no job is scheduling, so the request will be discarded");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                }
            }
            break;

        // 确认滑台到位按钮
        case CONTROL_COMMAND_ID_CONFIRM_SLIDER_POS_BUTTON:
            {
                auto& event = scheduler->slider_pos_confirm_flag_;
                if (event.isWaiting())
                {
                    ROS_INFO("Notify slider position confirm");
                    event.notify(true);
                }
            }
            break;

        // 机械臂张开换电池
        case CONTROL_COMMAND_ID_OPEN_ARM_TO_CHANGE_BATTERY:
            if (!scheduler->isIdle())
            {
                ROS_ERROR("A job is still running, open arm to change battery request will be discarded");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"有作业正在进行，无法进行换电池","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to open arm to change battery");
                // 作业文件名
                string filename = string(JOB_FILE_DIRECTORY) + "/OriposToChangebattery.csv"; //txt or csv     
                vector<int> segment_indexs;     // 需要运行的段序
                segment_indexs.push_back(1);
                //ROS_INFO_STREAM("cmd_call_back: scheduling file " << filename);
                if (!scheduler->schedule(filename, segment_indexs))
                {
                    ROS_ERROR("cmd_call_back: failed to schedule segment");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"机械臂张开换电池作业文件出错","info");
                }
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"执行机械臂张开换电池作业","info");
                
                
            }
            break;

        // 机械臂换电池回位
        case CONTROL_COMMAND_ID_CLOSE_ARM_AFTER_CHANGE_BATTERY:
            if (!scheduler->isIdle())
            {
                ROS_ERROR("A job is still running, close arm after change battery request will be discarded");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"有作业进行中，无法进行换电池回位","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to close arm after change battery");
                // 作业文件名
                string filename = string(JOB_FILE_DIRECTORY) + "/ChangebatteryToOripos.csv";      
                vector<int> segment_indexs;     // 需要运行的段序
                segment_indexs.push_back(1);
                //ROS_INFO_STREAM("cmd_call_back: scheduling file " << filename);
                if (!scheduler->schedule(filename, segment_indexs))
                {
                    ROS_ERROR("cmd_call_back: failed to schedule segment");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"机械臂换电池回位作业失败","info");
                
                }
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"执行机械臂换电池回位作业","info");
                
            }
            break;

        // 解除机械臂保护性停止
        case CONTROL_COMMAND_ID_UNLOCK_ARM_PROTECTIVE_STOP:
            if (!scheduler->isInManuallyOperationMode())
            {
                ROS_ERROR("For safety, you won't be able to left arm protective stop while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),15,"警告-当前非手工模式，无法解除！-转人工控制","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to unlock lift arm protective stop");
                scheduler->notifyServiceReach(UNLOCK_ARM_PROTECTIVE_STOP);
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"正在解除保护性停止","info");
            }
            break;

        // 步进执行（前进）
        case CONTROL_COMMAND_ID_STEP_EXECUTION_FORWARD:
            if ((!scheduler->isInManuallyOperationMode())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            {
                ROS_ERROR("For safety, you won't be able to step forward while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),15,"警告-当前非手工模式或机械臂故障状态下，不能步进！-转人工控制","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to step forward");
                if (!scheduler->isIdle())
                {
                    scheduler->requestToStepForward();
                    if (scheduler->isWaitingManuallyConfirm())
                    {
                        scheduler->notifyManuallyConfirm();
                    }
                }
                else
                {
                    ROS_INFO("But no job is scheduling, so the request will be discarded");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                }
            }
            break;

        // 步进执行（后退）
        case CONTROL_COMMAND_ID_STEP_EXECUTION_BACKWARD:
            if ((!scheduler->isInManuallyOperationMode())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            {
                ROS_ERROR("For safety, you won't be able to step backward while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),15,"警告-当前非手工模式或机械臂故障状态下，不能步退！-转人工控制","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to step backward");
                if (!scheduler->isIdle())
                {
                    scheduler->requestToStepBackward();
                    if (scheduler->isWaitingManuallyConfirm())
                    {
                        scheduler->notifyManuallyConfirm();
                    }
                }
                else
                {
                    ROS_INFO("But no job is scheduling, so the request will be discarded");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                }
            }
            break;

        case 17:
            if ((!scheduler->isInManuallyOperationMode())||(l_arm_protected_flaged==true)||(r_arm_protected_flaged==true)||(l_arm_enable_flaged==true)||(r_arm_enable_flaged==true))
            {
                ROS_ERROR("For safety, you won't be able to step skip while not in manually operation mode.");
                send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),15,"警告-当前非手工模式或机械臂故障状态下，不能跳步作业！-转人工控制","info");
            }
            else
            {
                ROS_INFO("Remote ground station ask to step skip");
                if (!scheduler->isIdle())
                {
                    scheduler->requestToSkip();
                    if (scheduler->isWaitingManuallyConfirm())
                    {
                        scheduler->notifyManuallyConfirm();
                    }
                }
                else
                {
                    ROS_INFO("But no job is scheduling, so the request will be discarded");
                    send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),14,"暂无作业调度，忽略本次请求","info");
                }
            }
            break;

        default:
            break;
        }
    }
}

// 解析处理地面站发给主控结点请求控制电源板的命令
static void control_power_manager(const std::string& cmd_str)
{
    rapidjson::Document d;
    d.Parse(cmd_str.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("control_power_manager: received content invalid");
        return;
    }

    const rapidjson::Value& cmdArray = d["cmdArray"];
    if (!cmdArray.IsArray())
    {
        ROS_ERROR("control_power_manager: received content invalid, cannot parse cmdArray");
        return;
    }

    // 循环处理每一个命令
    for (size_t i = 0; i < cmdArray.Size(); ++i)
    {
        const rapidjson::Value& seq = cmdArray[i]["seq"];
        if (!seq.IsInt())
        {
            ROS_ERROR("control_power_manager: received content invalid, cannot parse seq of cmdArray");
            return;
        }

        const rapidjson::Value& value = cmdArray[i]["value"];
        if (!value.IsNumber())
        {
            ROS_ERROR("control_power_manager: received content invalid, cannot parse value of cmdArray");
            return;
        }

        switch (seq.GetInt())
        {
        case POWER_MANAGER_ARM_POWER_ON: // 机械臂开机 话题(电源板上电)+服务（机械臂上电）
            //ROS_INFO("Sending ===power on=== command to arm_power manager"); 
            send_command_to_power_manager(scheduler->getCmdMsgPub(), POWER_MANAGER_ARM_POWER_ON, 0, ARM_POWER_ON_TOPIC);
            scheduler->notifyServiceReach(POWER_MANAGER_ARM_POWER_ON);
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"正在通知机械臂开机","info");  
            break;

        case POWER_MANAGER_ARM_POWER_OFF: // 机械臂关机 话题控制即可
            //ROS_INFO("Sending ===power off=== command to arm_power manager");
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"机械臂已关机","info"); 
            send_command_to_power_manager(scheduler->getCmdMsgPub(), POWER_MANAGER_ARM_POWER_OFF, 0, ARM_POWER_OFF_TOPIC);
            break;

        case ARM_EMERGENCY_STOP: // 机械臂急停 话题控制即可
            //ROS_INFO("Sending ===emergency stop=== command to power manager");
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"机械臂已急停","info"); 
            send_command_to_power_manager(scheduler->getCmdMsgPub(), ARM_EMERGENCY_STOP, 0, ARM_EMERGENCY_STOP_TOPIC);
            if (!scheduler->isIdle())
            {
                ROS_INFO("Also stopping the scheduling job");
                scheduler->requestToStop();
            }
           break;

        case ARM_EMERGENCY_STOP_RELIEVE: // 系统解除急停 话题+服务
            //ROS_INFO("Sending ===release emergency stop=== command to power manager");
            //send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"正在通知解除急停","info"); 
            send_command_to_power_manager(scheduler->getCmdMsgPub(), ARM_EMERGENCY_STOP_RELIEVE, 0, ARM_EMERGENCY_STOP_RELIEVE_TOPIC);
            scheduler->notifyServiceReach(ARM_EMERGENCY_STOP_RELIEVE);
            break;

        default:
            break;
        }
    }
}

// 解析处理地面站发给主控结点请求控制左臂的命令
static void control_left_arm(const std::string& cmd_str)
{
    static std::map<int, std::string> cmd_script_table = {
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_BASE_DIRECTION_1_MOTION, "/joint/J1_1.txt"},        // 主臂基座方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_BASE_DIRECTION_2_MOTION, "/joint/J1_0.txt"},        // 主臂基座方向2运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_SHOULDER_DIRECTION_1_MOTION, "/joint/J2_1.txt"},    // 主臂肩部方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_SHOULDER_DIRECTION_2_MOTION, "/joint/J2_0.txt"},    // 主臂肩部方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_ELBOW_DIRECTION_1_MOTION, "/joint/J3_1.txt"},       // 主臂肘部方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_ELBOW_DIRECTION_2_MOTION, "/joint/J3_0.txt"},       // 主臂肘部方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_1_DIRECTION_1_MOTION, "/joint/J4_1.txt"},     // 主臂手腕1方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_1_DIRECTION_2_MOTION, "/joint/J4_0.txt"},     // 主臂手腕1方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_2_DIRECTION_1_MOTION, "/joint/J5_1.txt"},     // 主臂手腕2方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_2_DIRECTION_2_MOTION, "/joint/J5_0.txt"},     // 主臂手腕2方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_3_DIRECTION_1_MOTION, "/joint/J6_1.txt"},     // 主臂手腕3方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_3_DIRECTION_2_MOTION, "/joint/J6_0.txt"},     // 主臂手腕3方向1运动

        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION, "/base/X_trans_base1.txt"},  // （基座坐标系）主臂末端位移方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION, "/base/X_trans_base0.txt"},  // （基座坐标系）主臂末端位移方向2运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION, "/base/Y_trans_base1.txt"},  // （基座坐标系）主臂末端位移方向3运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION, "/base/Y_trans_base0.txt"},  // （基座坐标系）主臂末端位移方向4运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION, "/base/Z_trans_base1.txt"},  // （基座坐标系）主臂末端位移方向5运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION, "/base/Z_trans_base0.txt"},  // （基座坐标系）主臂末端位移方向6运动

        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_1_MOTION, "/base/RX_trans_base1.txt"},        // （基座坐标系）主臂末端姿态方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_2_MOTION, "/base/RX_trans_base0.txt"},        // （基座坐标系）主臂末端姿态方向2运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_3_MOTION, "/base/RY_trans_base1.txt"},        // （基座坐标系）主臂末端姿态方向3运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_4_MOTION, "/base/RY_trans_base0.txt"},        // （基座坐标系）主臂末端姿态方向4运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_5_MOTION, "/base/RZ_trans_base1.txt"},        // （基座坐标系）主臂末端姿态方向5运动
        {CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_6_MOTION, "/base/RZ_trans_base0.txt"},        // （基座坐标系）主臂末端姿态方向6运动

        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION, "/tool/X_trans_TCP1.txt"},  // （工具坐标系）主臂末端位移方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION, "/tool/X_trans_TCP0.txt"},  // （工具坐标系）主臂末端位移方向2运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION, "/tool/Y_trans_TCP1.txt"},  // （工具坐标系）主臂末端位移方向3运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION, "/tool/Y_trans_TCP0.txt"},  // （工具坐标系）主臂末端位移方向4运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION, "/tool/Z_trans_TCP1.txt"},  // （工具坐标系）主臂末端位移方向5运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION, "/tool/Z_trans_TCP0.txt"},  // （工具坐标系）主臂末端位移方向6运动

        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_1_MOTION, "/tool/RX_trans_TCP1.txt"},        // （工具坐标系）主臂末端姿态方向1运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_2_MOTION, "/tool/RX_trans_TCP0.txt"},        // （工具坐标系）主臂末端姿态方向2运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_3_MOTION, "/tool/RY_trans_TCP1.txt"},        // （工具坐标系）主臂末端姿态方向3运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_4_MOTION, "/tool/RY_trans_TCP0.txt"},        // （工具坐标系）主臂末端姿态方向4运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_5_MOTION, "/tool/RZ_trans_TCP1.txt"},        // （工具坐标系）主臂末端姿态方向5运动
        {CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_6_MOTION, "/tool/RZ_trans_TCP0.txt"},        // （工具坐标系）主臂末端姿态方向6运动

        {CONTROL_COMMAND_ID_LEFT_ARM_STOP_MOTION, "/stop.txt"},  // 主臂停止运动
    };

    rapidjson::Document d;
    d.Parse(cmd_str.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("control_left_arm: received content invalid");
        return;
    }

    const rapidjson::Value& cmdArray = d["cmdArray"];
    if (!cmdArray.IsArray())
    {
        ROS_ERROR("control_left_arm: received content invalid, cannot parse cmdArray");
        return;
    }

    // 循环处理每一个命令
    for (size_t i = 0; i < cmdArray.Size(); ++i)
    {
        const rapidjson::Value& seq = cmdArray[i]["seq"];
        if (!seq.IsInt())
        {
            ROS_ERROR("control_left_arm: received content invalid, cannot parse seq of cmdArray");
            return;
        }

        if (cmd_script_table.find(seq.GetInt()) == cmd_script_table.cend())
        {
            ROS_ERROR_STREAM("control_left_arm: unsupported cmd seq: " << seq.GetInt());
            return;
        }

        const string script_path = string(ARM_CONTROL_SCRIPT_DIRECTORY) +  cmd_script_table.at(seq.GetInt());

        if (!script_path.empty())
        {
            ROS_INFO_STREAM("Executing left arm script " << script_path);

            const rapidjson::Value& value = cmdArray[i]["value"];
            exec_arm_script(scheduler->getLeftArmControlPub(), script_path,1,ROBOT_ARM_LEFT_ARM);
            // sleep(5);
            exec_arm_script(scheduler->getLeftArmControlPub(), script_path,2,ROBOT_ARM_LEFT_ARM);
        }
    }
}

// 解析处理地面站发给从控结点请求控制右臂的命令
static void control_right_arm(const std::string& cmd_str)
{
    static std::map<int, std::string> cmd_script_table = {
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_BASE_DIRECTION_1_MOTION, "/joint/J1_1.txt"},        // 从臂基座方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_BASE_DIRECTION_2_MOTION, "/joint/J1_0.txt"},        // 从臂基座方向2运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_SHOULDER_DIRECTION_1_MOTION, "/joint/J2_1.txt"},    // 从臂肩部方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_SHOULDER_DIRECTION_2_MOTION, "/joint/J2_0.txt"},    // 从臂肩部方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_ELBOW_DIRECTION_1_MOTION, "/joint/J3_1.txt"},       // 从臂肘部方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_ELBOW_DIRECTION_2_MOTION, "/joint/J3_0.txt"},       // 从臂肘部方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_1_DIRECTION_1_MOTION, "/joint/J4_1.txt"},     // 从臂手腕1方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_1_DIRECTION_2_MOTION, "/joint/J4_0.txt"},     // 从臂手腕1方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_2_DIRECTION_1_MOTION, "/joint/J5_1.txt"},     // 从臂手腕2方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_2_DIRECTION_2_MOTION, "/joint/J5_0.txt"},     // 从臂手腕2方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_3_DIRECTION_1_MOTION, "/joint/J6_1.txt"},     // 从臂手腕3方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_3_DIRECTION_2_MOTION, "/joint/J6_0.txt"},     // 从臂手腕3方向1运动

        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION, "/base/X_trans_base1.txt"},  // （基座坐标系）从臂末端位移方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION, "/base/X_trans_base0.txt"},  // （基座坐标系）从臂末端位移方向2运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION, "/base/Y_trans_base1.txt"},  // （基座坐标系）从臂末端位移方向3运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION, "/base/Y_trans_base0.txt"},  // （基座坐标系）从臂末端位移方向4运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION, "/base/Z_trans_base1.txt"},  // （基座坐标系）从臂末端位移方向5运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION, "/base/Z_trans_base0.txt"},  // （基座坐标系）从臂末端位移方向6运动

        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_1_MOTION, "/base/RX_trans_base1.txt"},        // （基座坐标系）从臂末端姿态方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_2_MOTION, "/base/RX_trans_base0.txt"},        // （基座坐标系）从臂末端姿态方向2运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_3_MOTION, "/base/RY_trans_base1.txt"},        // （基座坐标系）从臂末端姿态方向3运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_4_MOTION, "/base/RY_trans_base0.txt"},        // （基座坐标系）从臂末端姿态方向4运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_5_MOTION, "/base/RZ_trans_base1.txt"},        // （基座坐标系）从臂末端姿态方向5运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_6_MOTION, "/base/RZ_trans_base0.txt"},        // （基座坐标系）从臂末端姿态方向6运动

        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION, "/tool/X_trans_TCP1.txt"},  // （工具坐标系）从臂末端位移方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION, "/tool/X_trans_TCP0.txt"},  // （工具坐标系）从臂末端位移方向2运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION, "/tool/Y_trans_TCP1.txt"},  // （工具坐标系）从臂末端位移方向3运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION, "/tool/Y_trans_TCP0.txt"},  // （工具坐标系）从臂末端位移方向4运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION, "/tool/Z_trans_TCP1.txt"},  // （工具坐标系）从臂末端位移方向5运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION, "/tool/Z_trans_TCP0.txt"},  // （工具坐标系）从臂末端位移方向6运动

        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_1_MOTION, "/tool/RX_trans_TCP1.txt"},        // （工具坐标系）从臂末端姿态方向1运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_2_MOTION, "/tool/RX_trans_TCP0.txt"},        // （工具坐标系）从臂末端姿态方向2运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_3_MOTION, "/tool/RY_trans_TCP1.txt"},        // （工具坐标系）从臂末端姿态方向3运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_4_MOTION, "/tool/RY_trans_TCP0.txt"},        // （工具坐标系）从臂末端姿态方向4运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_5_MOTION, "/tool/RZ_trans_TCP1.txt"},        // （工具坐标系）从臂末端姿态方向5运动
        {CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_6_MOTION, "/tool/RZ_trans_TCP0.txt"},        // （工具坐标系）从臂末端姿态方向6运动

        {CONTROL_COMMAND_ID_RIGHT_ARM_STOP_MOTION, "/stop.txt"},  // 从臂停止运动
    };

    rapidjson::Document d;
    d.Parse(cmd_str.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("control_right_arm: received content invalid");
        return;
    }

    const rapidjson::Value& cmdArray = d["cmdArray"];
    if (!cmdArray.IsArray())
    {
        ROS_ERROR("control_right_arm: received content invalid, cannot parse cmdArray");
        return;
    }

    // 循环处理每一个命令
    for (size_t i = 0; i < cmdArray.Size(); ++i)
    {
        const rapidjson::Value& seq = cmdArray[i]["seq"];
        if (!seq.IsInt())
        {
            ROS_ERROR("control_right_arm: received content invalid, cannot parse seq of cmdArray");
            return;
        }

        if (cmd_script_table.find(seq.GetInt()) == cmd_script_table.cend())
        {
            ROS_ERROR_STREAM("control_right_arm: unsupported cmd seq: " << seq.GetInt());
            return;
        }

        const string script_path = string(ARM_CONTROL_SCRIPT_DIRECTORY) +  cmd_script_table.at(seq.GetInt());

        if (!script_path.empty())
        {
            ROS_INFO_STREAM("Executing right arm script " << script_path);

            const rapidjson::Value& value = cmdArray[i]["value"];
            exec_arm_script(scheduler->getRightArmControlPub(), script_path,1,ROBOT_ARM_RIGHT_ARM);
            exec_arm_script(scheduler->getRightArmControlPub(), script_path,2,ROBOT_ARM_RIGHT_ARM);
        }
    }
}

// 地面站确认雷达选点完成信号
void lidar_point_confirm(const std::string& msg)
{
    rapidjson::Document d;
    d.Parse(msg.c_str());
    if (!d.IsObject())
    {
        ROS_ERROR("lidar_point_confirm: received content invalid");
        return;
    }

    const rapidjson::Value& cmdArray = d["cmdArray"];
    
    for (size_t i = 0; i < cmdArray.Size(); ++i)
    {
        const rapidjson::Value& seq = cmdArray[i]["seq"];
        if (!seq.IsInt())
        { return; }

        const rapidjson::Value& value = cmdArray[i]["value"];

        if (value != 1) continue;

        switch (seq.GetInt())
        {
        case WAIT_LIDAR_POINT_CONFIRM: // 选点完成信号
        {
            //ROS_INFO("VisionNode Notifying lidar point confirm");
            auto& event = scheduler->lidar_point_confirm_flag_;
            if (event.isWaiting())
            {
                ROS_INFO("Notifying lidar point confirmed to arm...");
                event.notify(true);
            }
        break;
        }

        default:
        break;
       }
    }
}



// cmd_msg回调函数，负责命令信息的解析和分发
void cmd_call_back(const public_pkg::cmd_msg& msg)
{
    /*{ // 解决cmd_call_back中 中文不能识别，显示乱码
    std::wstring ws = s2ws(msg.content);
    std::string  ss = ws2s(ws);
    //ROS_INFO_STREAM(" " << ss);
    }*/
    
    if (msg.header.msgType != MSG_TYPE_CMD_MSG)
    {
        ROS_ERROR_STREAM("cmd_call_back: drop one message because type mismatch. "
                         "msgType should be " << MSG_TYPE_CMD_MSG << ", instead of " << msg.header.msgType);
        return;
    }

    // 只接收来自地面站，通知雷达选点完成信号指令
    if (msg.header.srcNodeName == NODE_NAME_GROUND_STATION && msg.header.dstNodeName == NODE_NAME_LIDAR_POING_CONFIRM )
    {
        if ( msg.object == LIDAR_POINT_CONFIRM ) 
        {
            lidar_point_confirm(msg.content);
        }
    }

    // 只接收自地面站，发给从控的命令
    if (msg.header.dstNodeName != NODE_NAME_CONTROLLER || msg.header.srcNodeName != NODE_NAME_GROUND_STATION ) 
    {
        // ROS_INFO("This command is not for me, leave it alone.");
        return;
    }

    ROS_INFO_STREAM("cmd_call_back: object: " << msg.object << ", content: " << msg.content);

    switch (msg.object)
    {
    case CONTROL_OBJECT_MASTER_CONTROLLER:
        control_master_controller(msg.content);
        break;
    
    case CONTROL_OBJECT_LEFT_ARM:
        control_left_arm(msg.content);
        break;

    case CONTROL_OBJECT_RIGHT_ARM:
        control_right_arm(msg.content);
        break;

    case CONTROL_OBJECT_VERTICAL_SLIDER:
        break;

    case CONTROL_OBJECT_HORIZONTAL_SLIDER:
        break;

    case CONTROL_OBJECT_POWER_MANAGER:
        control_power_manager(msg.content);
        break;
    
    /*case LIDAR_POINT_CONFIRM:
        lidar_point_confirm(msg.content);
        break;*/

    default:
        ROS_INFO("This command is not handled yet, leave it alone.");
        break;
    }
}