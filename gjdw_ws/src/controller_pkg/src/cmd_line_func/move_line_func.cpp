#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"

#include "std_msgs/String.h"
#include "ros/console.h"

#include <vector>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <map>
//#include <>
#include <iterator>
#include <regex>

using namespace std;

// 参数名 -> 参数值
using ArgNameValueMap = std::map<std::string, std::string>;

// 根据给定的参数值以及运动类型，生成机械臂可执行脚本
static std::string generate_move_script(const ArgNameValueMap& argname_to_field_value, const bool is_move_j)
{
    // 检查是否有关节数据
    const bool has_joint_data = argname_to_field_value.find("J1") != argname_to_field_value.cend();

    const char* prefix_template =
        "def function():\n"
        " set_digital_out(0,False)\n"
        " set_tcp(p[%s,%s,%s,%s,%s,%s])\n"
        " set_payload(%s)\n";

    const char* postfix_template =
        " set_digital_out(0,True)\n"
        "end\n";

    // 格式化输出不支持string，需要将string转为c_str
    auto argname_to_value = [&] (const std::string& name) {
        return argname_to_field_value.at(name).c_str();
    };

    const string prefix = format(prefix_template,
                                 argname_to_value("TCPX"), argname_to_value("TCPY"), argname_to_value("TCPZ"),
                                 argname_to_value("TCPRX"), argname_to_value("TCPRY"), argname_to_value("TCPRZ"),
                                 argname_to_value("PAYLOAD"));
    const string postfix = postfix_template;
    string cmd;

    if (!is_move_j)     // MOVEL指令
    {
        const char* cmd_template = " movel (p[%s,%s,%s,%s,%s,%s],a=%s,v=%s)\n";
        cmd = format(cmd_template,
                     argname_to_value("X"), argname_to_value("Y"), argname_to_value("Z"),
                     argname_to_value("RX"), argname_to_value("RY"), argname_to_value("RZ"),
                     argname_to_value("A"), argname_to_value("V"));
    }
    else if (has_joint_data)    // MOVEJ指令，带关节数据
    {
        // const char* cmd_template = " movej ([%s,%s,%s,%s,%s,%s],a=%s,v=%s)\n";
        // cmd = format(cmd_template,
        //              argname_to_value("J1"), argname_to_value("J2"), argname_to_value("J3"),
        //              argname_to_value("J4"), argname_to_value("J5"), argname_to_value("J6"),
        //              argname_to_value("A"), argname_to_value("V"));

        const char* cmd_template = " movej (get_inverse_kin(p[%s,%s,%s,%s,%s,%s],qnear=[%s,%s,%s,%s,%s,%s]),a=%s,v=%s)\n";
        cmd = format(cmd_template,
                     argname_to_value("X"), argname_to_value("Y"), argname_to_value("Z"),
                     argname_to_value("RX"), argname_to_value("RY"), argname_to_value("RZ"),
                     argname_to_value("J1"), argname_to_value("J2"), argname_to_value("J3"),
                     argname_to_value("J4"), argname_to_value("J5"), argname_to_value("J6"),
                     argname_to_value("A"), argname_to_value("V"));
    }
    else    // MOVEJ指令，不带关节数据
    {
        const char* cmd_template = " movej (p[%s,%s,%s,%s,%s,%s],a=%s,v=%s)\n";
        cmd = format(cmd_template,
                     argname_to_value("X"), argname_to_value("Y"), argname_to_value("Z"),
                     argname_to_value("RX"), argname_to_value("RY"), argname_to_value("RZ"),
                     argname_to_value("A"), argname_to_value("V"));
    }
    return prefix + cmd + postfix;
}

// MOVE类指令解析函数
static CmdLineFuncRsp move_func(SegmentScheduler& scheduler, const std::string& line,
    const vector<string>& arg_names, const bool is_move_j)
{
    // 解析命令字段内容
    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != arg_names.size())
    {
        ROS_ERROR_STREAM("Expected " << arg_names.size() << " arguments, but got " << fields.size());
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 设定字段名称到字段值的映射
    map<string, string> argname_to_field_value;
    for (size_t i = 0; i < arg_names.size(); ++i)
    {
        argname_to_field_value[arg_names[i]] = fields[i];
    }

    const VariableTable& var_table = scheduler.getVariableTable();

    // 设定变量参数解包规则
    static const map<string, vector<string>> arg_to_new_args = {
        {"TCP", {"TCPX", "TCPY", "TCPZ", "TCPRX", "TCPRY", "TCPRZ"}},
        {"PP", {"X", "Y", "Z", "RX", "RY", "RZ"}},
        {"Joint", {"J1", "J2", "J3", "J4", "J5", "J6"}},
    };

    // 检查每个字段，解析其中的变量字段
    const regex num_pattern("[0-9]*.*[0-9]+");    // 数值匹配模式
    for (auto &p : argname_to_field_value)
    {
        const string argname = p.first, field_value = p.second;

        string var_name, var_typename;
        if (!JobParser::ParseVariableDefine(field_value, var_name, var_typename))   // 不是变量字段，跳过
        {
            // 检查变量值类型，不是变量意味着参数应是数值
            if (argname != "CMD" && argname != "SingalDevice" && !std::regex_match(field_value, num_pattern))
            {
                ROS_ERROR_STREAM("Argument \"" << argname << "\" should be number, instead of \""
                                 << field_value << "\"");
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            continue;
        }

        assert (var_table.find(var_name) != var_table.cend());

        // 检查变量定义
        const VariablePointer& vp = var_table.at(var_name);
        if (vp->getTypeName() != var_typename)
        {
            ROS_ERROR_STREAM("Unsupported variable type: " << var_typename);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }

        // 根据变量类型读取变量的值
        switch (vp->getType())
        {
        case Variable::VARIABLE_TYPE_DOUBLE_6:
            // 只解包占位符是PP，TCP和Joint的变量，否则报错
            if (argname != "PP" && argname != "TCP" && argname != "JOINT")
            {
                ROS_ERROR_STREAM("Argument type of \"" << argname << "\" shouldn't be \""
                                 << field_value << "\"");
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }

            // 解包PP，TCP，Joint参数
            {
                assert (arg_to_new_args.find(vp->getTypeName()) != arg_to_new_args.cend());

                const vector<string>& new_argnames = arg_to_new_args.at(vp->getTypeName());
                const DoubleValueArray& values = vp->getValue<DoubleValueArray>();
                assert (values.size() == new_argnames.size());

                for (size_t i = 0; i < new_argnames.size(); ++i)
                {
                    argname_to_field_value[new_argnames[i]] = to_string(values[i]);
                }
            }
            break;

        case Variable::VARIABLE_TYPE_DOUBLE:
            // 占位符是PP，TCP和Joint的变量，其类型应为DOUBLE_6
            if (argname == "PP" || argname == "TCP" || argname == "JOINT")
            {
                ROS_ERROR_STREAM("Argument type of \"" << argname << "\" shouldn't be \""
                                 << vp->getTypeName() << "\"");
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            argname_to_field_value[argname] = std::to_string(vp->getValue<DoubleValue>());
            break;

        case Variable::VARIABLE_TYPE_INT:
            // 占位符是PP，TCP和Joint的变量，其类型应为DOUBLE_6
            if (argname == "PP" || argname == "TCP" || argname == "JOINT")
            {
                ROS_ERROR_STREAM("Argument type of \"" << argname << "\" shouldn't be \""
                                 << vp->getTypeName() << "\"");
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            argname_to_field_value[argname] = std::to_string(vp->getValue<IntValue>());
            break;

        case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
        default:
            ROS_ERROR_STREAM("Unsupported variable type: " << var_typename);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
    }

    // 提取命令字段ID
    int ID = 0;
    try {
        ID = stoi(argname_to_field_value["ID"]);
        if (ID != CMD_FUNC_ARM_ID_LEFT_ARM && ID != CMD_FUNC_ARM_ID_RIGHT_ARM)
        {
            ROS_ERROR_STREAM("Unrecognized ID: " << ID);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
    } catch (...) {
        ROS_ERROR_STREAM("ID should be integer or variable definition, instead of \""
                         << argname_to_field_value["ID"] << "\"");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }



    // 生成机械臂可执行脚本
    std_msgs::String msg;
    msg.data = generate_move_script(argname_to_field_value, is_move_j);

    // 发送执行脚本给机械臂并等待到位信号
    CmdLineFuncRsp response = CMD_LINE_FUNC_RSP_SUCCESS;
    if (ID  == CMD_FUNC_ARM_ID_LEFT_ARM)
    {
        ROS_INFO_STREAM("sending script to left arm:\n" << msg.data);
        send_command_to_robot_script(scheduler.getLeftArmControlPub(),1,msg.data,ID);
        send_command_to_robot_script(scheduler.getLeftArmControlPub(),2,msg.data,ID);
        //scheduler.getLeftArmControlPub().publish(msg);

        ROS_INFO_STREAM("[LEFT ARM IS WAITING FOR REACH]");
       
        //std::this_thread::sleep_for(chrono::seconds(1));
        std::this_thread::sleep_for(chrono::milliseconds(500)); 

        if(fields[0]=="#MOVEL2_WAITSIGNAL")
        {
            //调用等待信号函数
            bool bWaitSignal = false;
            send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"主臂等待工具停止信号","info");
    
            ROS_INFO_STREAM("[LEFT ARM IS WAITING FOR SIGNAL]");
            bWaitSignal = wait_device_signal(scheduler,argname_to_field_value["SingalDevice"],stoi(argname_to_field_value["SignalSeq"]),stoi(argname_to_field_value["SignalValue"]));
            if (bWaitSignal == true)
            {
                scheduler.stopArmMotion();
                ROS_INFO("[LEFT ARM already stoped]");
                std::this_thread::sleep_for(chrono::milliseconds(500));
                return CMD_LINE_FUNC_RSP_SUCCESS;
            }
            // else
            // {
            //     return CMD_LINE_FUNC_RSP_TIMEOUT;
            // }
            
        }

        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待主臂到达指定位置","info");
    

        auto& event = scheduler.left_arm_reach_flag_;
        if (!event.wait_for(chrono::seconds(DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS)))
        {
            send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,LEFTARM_ARRIVE_TIMEOUT,"info");
            ROS_INFO("[LEFT ARM reached timeout]");
            response =  CMD_LINE_FUNC_RSP_TIMEOUT;
        }
        else if (event.get() == false)
        {
            response = CMD_LINE_FUNC_RSP_FAILED;
            ROS_INFO("[LEFT ARM reached failed]");
        }
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"主臂已经到达指定位置","info");
    
    }
    else
    {
        ROS_INFO_STREAM("sending script to right arm:\n" << msg.data);
        send_command_to_robot_script(scheduler.getRightArmControlPub(),1,msg.data,ID);
        send_command_to_robot_script(scheduler.getRightArmControlPub(),2,msg.data,ID);
        //scheduler.getRightArmControlPub().publish(msg);

        ROS_INFO_STREAM("[RIGHT ARM IS WAITING FOR REACH]");
        
        std::this_thread::sleep_for(chrono::milliseconds(500)); 

        if(fields[0]=="#MOVEL2_WAITSIGNAL")
        {
            //调用等待信号函数
            bool bWaitSignal = false;
            ROS_INFO_STREAM("[RIGHT ARM IS WAITING FOR SIGNAL]");
            send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"从臂等待工具停止信号","info");
    
            bWaitSignal = wait_device_signal(scheduler,argname_to_field_value["SingalDevice"],stoi(argname_to_field_value["SignalSeq"]),stoi(argname_to_field_value["SignalValue"]));
            if (bWaitSignal == true)
            {
                scheduler.stopArmMotion();
                std::this_thread::sleep_for(chrono::milliseconds(500));
                send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"工具停止信号已到达","info");
                return CMD_LINE_FUNC_RSP_SUCCESS;
            }
            // else
            // {
            //     return CMD_LINE_FUNC_RSP_TIMEOUT;
            // }
            
        }

        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待从臂到达指定位置","info");
    
        auto& event = scheduler.right_arm_reach_flag_;
        if (!event.wait_for(chrono::seconds(DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS)))
        {
            send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,RIGHTARM_ARRIVE_TIMEOUT,"info");
    
            response =  CMD_LINE_FUNC_RSP_TIMEOUT;
        }
        else if (event.get() == false)
        {
            response = CMD_LINE_FUNC_RSP_FAILED;
        }
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"从臂已经到达指定位置","info");
    
    }
    if (response != CMD_LINE_FUNC_RSP_SUCCESS)
    {
        return response;
    }


    

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp movej_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEJ");

    const vector<string> arg_names{"CMD", "ID", 
                                    "X", "Y", "Z", "RX", "RY", "RZ", 
                                    "J1", "J2", "J3", "J4", "J5", "J6",
                                    "TCPX", "TCPY", "TCPZ", "TCPRX", "TCPRY", "TCPRZ",
                                    "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, true);
}

CmdLineFuncRsp movej1_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEJ1");

    vector<string> arg_names{"CMD", "ID", 
                             "X", "Y", "Z", "RX", "RY", "RZ", 
                             "J1", "J2", "J3", "J4", "J5", "J6",
                             "TCP",
                             "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, true);
}

CmdLineFuncRsp movej2_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEJ2");

    vector<string> arg_names{"CMD", "ID", 
                             "PP", "JOINT", "TCP",
                             "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, true);
}

CmdLineFuncRsp movej3_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEJ3");

    vector<string> arg_names{"CMD", "ID", 
                             "PP", "TCP",
                             "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, true);
}

CmdLineFuncRsp movel_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEL");

    const vector<string> arg_names{"CMD", "ID", 
                                   "X", "Y", "Z", "RX", "RY", "RZ", 
                                   "TCPX", "TCPY", "TCPZ", "TCPRX", "TCPRY", "TCPRZ",
                                   "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, false);
}

CmdLineFuncRsp movel1_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEL1");

    vector<string> arg_names{"CMD", "ID", 
                             "X", "Y", "Z", "RX", "RY", "RZ", 
                             "TCP",
                             "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, false);
}

CmdLineFuncRsp movel2_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEL2");

    vector<string> arg_names{"CMD", "ID", 
                             "PP", "TCP",
                             "PAYLOAD", "V", "A"};

    return move_func(scheduler, line, arg_names, false);
}
// V 不能过大，速度太快还没来得及等到信号就已经到位了，0.01最佳
CmdLineFuncRsp movel2_waitsinal_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command MOVEL2_WAITSIGNAL");

    vector<string> arg_names{"CMD", "ID", 
                             "PP", "TCP","PAYLOAD", "V", "A",
                             "SingalDevice","SignalSeq","SignalValue"};

    return move_func(scheduler, line, arg_names, false);
}

bool wait_device_signal(SegmentScheduler& scheduler, const std::string& SingalDevice, const int& SingalSeq,const int& SingalValue)
{
    auto& event = scheduler.wait_yx_signal_arrive;
    event.signal_device = SingalDevice;
    event.signal_seq = SingalSeq;
    event.signal_value = SingalValue;
    
    if (!event.wait_for(chrono::seconds(20)))
    {
        return false;
    }
    else if (event.get() == -100)
    {
        return false;
    }
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"机械臂信号获取完成","info");
    
    return true;    
}