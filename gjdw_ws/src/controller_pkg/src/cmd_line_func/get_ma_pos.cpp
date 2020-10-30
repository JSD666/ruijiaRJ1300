#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"
#include "math_func.h"

#include "ros/console.h"

#include <vector>
#include <cassert>
#include <map>
#include <iterator>

using namespace std;

// 获取机械臂PP数据
CmdLineFuncRsp get_PP_value(SegmentScheduler& scheduler, int MaID, DoubleValueArray& point, const DoubleValueArray& tcp)
{
	CmdLineFuncRsp retVal = CMD_LINE_FUNC_RSP_SUCCESS;

	// 等待机械臂遥测数据返回
	ROS_INFO("[WAITING FOR MA POSITION POINT VALUE %d]", MaID);
   
	// 等待机械臂位姿数据回调
	if (MaID == CMD_FUNC_ARM_ID_LEFT_ARM)
	{
		auto& event = scheduler.left_arm_pp_received_flag_;
		if (!event.wait_for(chrono::seconds(DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS)))
		{
			send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,LEFTARM_GETPP_TIMEOUT,"info");
    
			retVal = CMD_LINE_FUNC_RSP_TIMEOUT;
		}
		else if (event.get() == false)
		{
			retVal = CMD_LINE_FUNC_RSP_FAILED;
		}
	}
	else if (MaID == CMD_FUNC_ARM_ID_RIGHT_ARM)
	{
		auto& event = scheduler.right_arm_pp_received_flag_;
		if (!event.wait_for(chrono::seconds(DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS)))
		{
			send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,RIGHTARM_GETPP_TIMEOUT,"info");
    
			retVal = CMD_LINE_FUNC_RSP_TIMEOUT;
		}
		else if (event.get() == false)
		{
			retVal = CMD_LINE_FUNC_RSP_FAILED;
		}
	}
	else
	{
		retVal = CMD_LINE_FUNC_RSP_FAILED;
	}


	if (retVal == CMD_LINE_FUNC_RSP_SUCCESS)
	{
		if (MaID == CMD_FUNC_ARM_ID_LEFT_ARM)
		{
			point = scheduler.getArm1PP();
		}
		else if (MaID == CMD_FUNC_ARM_ID_RIGHT_ARM)
		{
			point = scheduler.getArm2PP();
		}
	}
	
	return retVal;

}

// 获取机械臂Joint遥测数据
CmdLineFuncRsp get_Joint_value(SegmentScheduler& scheduler, int MaID, DoubleValueArray& point)
{
	// 等待机械臂遥测数据返回
	ROS_INFO("[WAITING FOR MA JOINT POINT VALUE %d]", MaID);
	
	auto& event = scheduler.arm_analog_info_received_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS)))
    {
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),14,ARM_GETJOINT_TIMEOUT,"info");
    
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

	if (MaID == CMD_FUNC_ARM_ID_LEFT_ARM)
	{
		point = scheduler.getArm1Joint();
	}
	else if (MaID == CMD_FUNC_ARM_ID_RIGHT_ARM)
	{
		point = scheduler.getArm2Joint();
	}	
	
	return CMD_LINE_FUNC_RSP_SUCCESS;
}


//写point参数到变量表
CmdLineFuncRsp set_variable_double6(SegmentScheduler& scheduler, const std::string& var_define, const DoubleValueArray& point)
{        
	string var_name, var_typename;
	// 提取变量字段
	
    if (!JobParser::ParseVariableDefine(var_define, var_name, var_typename))
	{
		ROS_ERROR_STREAM("ERROR: variable definition invalid: " << var_define);
		return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	}

    // 检查变量类型是否符合定义
    VariableTable& var_table = scheduler.getVariableTable();
	
	// 若变量已创建，检查类型是否匹配；否则，创建它
    if (var_table.find(var_name) != var_table.end())
    {
        const VariablePointer vp = var_table[var_name];
        if (vp->getTypeName() != var_typename)
	    {
	        ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
	        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
	    }
	    if (vp->getType() != Variable::VARIABLE_TYPE_DOUBLE_6)
	    {
	        ROS_ERROR_STREAM("Variable type should be VARIABLE_TYPE_DOUBLE_6 intread of : " << vp->getTypeName());
	        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    	}

		vp->setValue<DoubleValueArray>(point);
		ROS_INFO_STREAM("Variable:" << var_define << ": " << *vp);
    }
	else
	{
		// 变量不存在，创建变量
	    try 
		{
			const Variable::Type var_type = Variable::VARIABLE_TYPE_DOUBLE_6;
	        VariablePointer vp = VariablePointer(new Variable(var_name, var_typename, var_type));
	        var_table[var_name] = vp;
			vp->setValue<DoubleValueArray>(point);
			ROS_INFO_STREAM("Variable:" << var_define << ": " << *vp);
	    } 
		catch (const std::exception& e) 
		{
	        ROS_ERROR_STREAM("ERROR: failed to new Variable, skipped: " << e.what());
	    }
	}

	return CMD_LINE_FUNC_RSP_SUCCESS;
}


CmdLineFuncRsp get_ma_pos_func(SegmentScheduler& scheduler, const std::string& line)
{
	DoubleValueArray var_pp, var_joint, var_tcp;
	string var_define;

    ROS_INFO_STREAM("get_ma_pos_func" );    
	
	const vector<string> arg_names{"CMD", "ID",	"PP", "Joint", "TCP"};

    // 解析命令字段内容
    const vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != arg_names.size())
    {
        ROS_ERROR_STREAM("Expected " << arg_names.size() - 1 << " arguments, but got " << fields.size() - 1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 设定字段名称到字段值的映射
    map<string, string> argname_to_field_value;
    for (size_t i = 0; i < arg_names.size(); ++i)
    {
        argname_to_field_value[arg_names[i]] = fields[i];
    }

	// 提取命令字段ID
    unsigned int MaID;
    try 
    {
         MaID = stoi(argname_to_field_value["ID"]);
    }
    catch(...)
    {
        ROS_ERROR_STREAM("#GETMAPOS* ID should be an int" );
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
    }
    
    if (MaID != CMD_FUNC_ARM_ID_LEFT_ARM && MaID != CMD_FUNC_ARM_ID_RIGHT_ARM)
    {
        ROS_ERROR_STREAM("unrecognized MaID: " << MaID);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

	if (MaID == CMD_FUNC_ARM_ID_LEFT_ARM)
	{
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"读取主臂当前位置","info");

	}
	else
	{
		send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"读取从臂当前位置","info");

	}
	


	// 获取到遥测数据，赋值变量
	CmdLineFuncRsp retVal = get_PP_value(scheduler, MaID, var_pp, var_tcp);
	if (retVal != CMD_LINE_FUNC_RSP_SUCCESS)
	{
		return retVal;
	}
	var_define = argname_to_field_value["PP"];
	set_variable_double6(scheduler, var_define, var_pp);
	retVal = get_Joint_value(scheduler, MaID, var_joint);
	if (retVal != CMD_LINE_FUNC_RSP_SUCCESS)
	{
		return retVal;
	}
	var_define = argname_to_field_value["Joint"];
	set_variable_double6(scheduler, var_define, var_joint);
	
	
    return CMD_LINE_FUNC_RSP_SUCCESS;
}

