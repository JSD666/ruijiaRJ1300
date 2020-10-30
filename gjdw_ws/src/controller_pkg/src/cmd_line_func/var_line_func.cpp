#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"
#include "math_func.h"

#include "ros/console.h"

#include <vector>
#include <cassert>
#include <regex>

using namespace std;

CmdLineFuncRsp var_operator_assignment_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    vector<string> fields = getlines(line, '=', true);
    if (fields.size() != 2)
    {
        ROS_ERROR("variable assignment invalid");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 第一个是数组元素的位置
    int pos_first_array_member = -1;
    const regex num_pattern("[0-9]*.*[0-9]+");    // 数值匹配模式
    string var_name[fields.size()], var_typename[fields.size()];
    // 将命令行各段的类型和名字存起来
    // 用于存储下标位置
    vector<int>  element_flag(fields.size(),-1);
    // 用于存储第某个参数数据,第一位为结果数据
    vector<double>  numbers(fields.size(),0);
    // 判断是对数组元素进行操作还是对数组整体进行操作,通过检测是否含有[],若存在则确定要进行第几个元素的提取
    for(int num_fields = 0;num_fields < 2;num_fields++)
    {
        int start = -1,end =-1;
        bool flag_start=false,flag_end=false;
        bool err_format = false;
         // 未检测到完整的[],则报错
        for(int pos_fields = 0;pos_fields<fields[num_fields].size();pos_fields++)
        {
            if(fields[num_fields][pos_fields]=='[')
            {
                if(flag_start)
                {
                    err_format = true;
                    break;
                }
                else{
                    flag_start = true;
                    start = pos_fields;
                }
            }
        if(fields[num_fields][pos_fields]==']')
            {
                if(flag_end)
                {
                    err_format = true;
                    break;
                }
                else{
                    flag_end = true;
                    end = pos_fields;
                }
            }
        }
        if(err_format)
        {
            ROS_ERROR_STREAM("Illegal Array Index (Incorrect '[]') " << var_typename[num_fields]);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }

        // 从start和end之间读取数组元素的下标
        if(start!=-1&&end!=-1)
        {
            // 下标只能是0-5的整数
            if(end-start!=2)
            {
                ROS_ERROR_STREAM("Illegal Array Index " << var_typename[num_fields]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            if(fields[num_fields][start+1]>'5'||fields[num_fields][start+1]<'0')
            {
                ROS_ERROR_STREAM("Illegal Array Index (Index Not Between 0-5)" << var_typename[num_fields]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            element_flag[num_fields]  = fields[num_fields][start+1]-'0';
            fields[num_fields] =  fields[num_fields].substr(0,start);
            //ROS_ERROR_STREAM("new command " << fields[num_fields]);
        }
    }



    JobParser::ParseVariableDefine(fields[0], var_name[0], var_typename[0]);

    VariableTable& var_table = scheduler.getVariableTable();
    assert (var_table.find(var_name[0]) != var_table.cend());
    // 检查变量定义
    VariablePointer& vp = var_table.at(var_name[0]);
    if (vp->getTypeName() != var_typename[0])
    {
        ROS_ERROR_STREAM("variable definition conflicts with: " << *vp);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    ROS_INFO_STREAM("Target Value before Operation:"<<*vp);

    // 变量间赋值
    if (fields[1].find("var") != string::npos)
    {
        // 获取右操作变量
        string var_name, var_typename;
        JobParser::ParseVariableDefine(fields[1], var_name, var_typename);

        assert (var_table.find(var_name) != var_table.cend());
        VariablePointer& vp2 = var_table.at(var_name);

        if (vp->getType() != vp2->getType())
        {
            ROS_ERROR_STREAM("cannot assign " << vp2->getType() << " type to " << vp->getType());
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        ROS_INFO_STREAM("element_flag[0]:" << element_flag[0] << "element_flag[1]:" << element_flag[1]);
        if (((element_flag[1]!=-1)&&(element_flag[0]==-1))||((element_flag[1]==-1)&&(element_flag[0]!=-1)))
        {
            ROS_ERROR_STREAM("two variable only one have []");
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        switch (vp->getType())
        {
        case Variable::VARIABLE_TYPE_DOUBLE_6:
        case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
        {
            if (element_flag[1] == -1)
            {
                vp->setValue(vp2->getValue<DoubleValueArray>());
            }
            else
            {
                DoubleValueArray temp = vp->getValue<DoubleValueArray>();
                // for(int i=0;i<6;i++)
                // {
                //     temp[i] = values_temp[i];
                // }
                const DoubleValueArray& values_temp = vp2->getValue<DoubleValueArray>();
                temp[element_flag[0]] = values_temp[element_flag[1]];
                vp->setValue(temp);
            }
            
            break;
        }

        case Variable::VARIABLE_TYPE_DOUBLE:
        {
            if(element_flag[1]!=-1)
            {
                ROS_ERROR_STREAM("Argument "<<fields[1]<<" is not array but has '[]'" );
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            vp->setValue(vp2->getValue<DoubleValue>());
            break;
        }

        case Variable::VARIABLE_TYPE_INT:
        {
            if(element_flag[1]!=-1)
            {
                ROS_ERROR_STREAM("Argument "<<fields[1]<<" is not array but has '[]'" );
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            vp->setValue(vp2->getValue<IntValue>());
            break;
        }

        default:
            ROS_ERROR_STREAM("Unsupported operation for variable: " <<  *vp);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            break;
        }
        ROS_INFO_STREAM("Target Value After Assignment:"<<*vp);
        return CMD_LINE_FUNC_RSP_SUCCESS;
    }
    // 字面值赋值
    switch (vp->getType())
    {
    case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
    case Variable::VARIABLE_TYPE_DOUBLE_6:
        {
            if (element_flag[0] == -1)
            {
                vector<double> values;
                for (auto& value_str : getlines(fields[1], ',', true))
                {
                    try {
                        const double value = std::stod(value_str);
                        values.push_back(value);
                    } catch (const std::exception& e) {
                        ROS_ERROR_STREAM("invalid operand: " << value_str);
                        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                    }
                }
                if (vp->getType() == Variable::VARIABLE_TYPE_DOUBLE_6 && values.size() != 6)
                {
                    ROS_ERROR_STREAM("expected 6 operand, but got " << values.size());
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                vp->setValue(values);
            }
            else
            {
                DoubleValueArray temp = vp->getValue<DoubleValueArray>();
                try
                { 
                temp[element_flag[0]] = std::stod(fields[1]);
                vp->setValue(temp);
                }
                catch(const std::exception& e) 
                {
                ROS_ERROR_STREAM("invalid operand: " << fields[1]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }               
            }
            
        }
        break;

    case Variable::VARIABLE_TYPE_DOUBLE:
        {
            try {
                const double value = std::stod(fields[1]);
                vp->setValue(DoubleValue(value));
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("invalid operand: " << fields[1]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
        }
        break;

    case Variable::VARIABLE_TYPE_INT:
        {
            try {
                const int value = std::stoi(fields[1]);
                vp->setValue(IntValue(value));
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("invalid operand: " << fields[1]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
        }
        break;

    default:
        ROS_ERROR_STREAM("Unsupported operation for variable: " <<  *vp);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    ROS_INFO_STREAM("Target Value After Assignment:"<<*vp);
    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp offset_line_func(SegmentScheduler& scheduler, const std::string& line)
{
    VariableTable& var_table = scheduler.getVariableTable();

    const vector<string> fields = JobParser::GetCommandFields(line);
    const int num_expected_fields = 7;
    if (fields.size() != num_expected_fields)
    {
        ROS_ERROR_STREAM("Expected " << num_expected_fields-1 << " arguments, but got " << fields.size()-1);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 解析字面值参数
    int feature;
    try {
        feature = std::stoi(fields[3]);
        if (feature != OFFSET_FEATURE_BASE_COORDINATE && feature != OFFSET_FEATURE_TOOL_COORDINATE)
        {
            ROS_ERROR("Argument value of \"Feature\" invalid");
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
    } catch (...) {
        ROS_ERROR("Argument type of \"Feature\" invalid, should be INT");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    int direction;
    try {
        direction = std::stoi(fields[4]);
        switch (direction)
        {
        // 用来判断参数是否非法
        case OFFSET_DIRECTION_X:
        case OFFSET_DIRECTION_Y:
        case OFFSET_DIRECTION_Z:
        case OFFSET_DIRECTION_RX:
        case OFFSET_DIRECTION_RY:
        case OFFSET_DIRECTION_RZ:
            break;

        default:
            ROS_ERROR("Argument value of \"Direction\" invalid");
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            break;
        }
    } catch (...) {
        ROS_ERROR("Argument type of \"Direction\" invalid, should be INT");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    double value;
    try {
        value = std::stod(fields[5]);
    } catch (...) {
        ROS_ERROR("Argument type of \"Value\" invalid");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 解析变量参数
    DoubleValueArray pp;
    {
        const string& var_define = fields[1];

        string var_name, var_typename;
        JobParser::ParseVariableDefine(var_define, var_name, var_typename);

        // 检查变量类型是否符合定义
        VariablePointer& vp = var_table.at(var_name);
        if (vp->getTypeName() != var_typename)
        {
            ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        if (vp->getTypeName() != "PP")
        {
            ROS_ERROR_STREAM("Variable definition \"" << var_define << "\" invalid, should be PP type"); 
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        pp = vp->getValue<DoubleValueArray>();
    }

    DoubleValueArray tcp;
    {
        const string& var_define = fields[2];

        string var_name, var_typename;
        JobParser::ParseVariableDefine(var_define, var_name, var_typename);

        // 检查变量类型是否符合定义
        VariablePointer& vp = var_table.at(var_name);
        if (vp->getTypeName() != var_typename)
        {
            ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        if (vp->getTypeName() != "TCP")
        {
            ROS_ERROR_STREAM("Variable definition \"" << var_define << "\" invalid, should be TCP type"); 
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        tcp = vp->getValue<DoubleValueArray>();
    }

    VariablePointer output_pp;
    {
        const string& var_define = fields[6];

        string var_name, var_typename;
        JobParser::ParseVariableDefine(var_define, var_name, var_typename);

        // 检查变量类型是否符合定义
        VariablePointer& vp = var_table.at(var_name);
        if (vp->getTypeName() != var_typename)
        {
            ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        if (vp->getTypeName() != "PP")
        {
            ROS_ERROR_STREAM("Variable definition \"" << var_define << "\" invalid, should be PP type"); 
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        output_pp = vp;
    }

    // 进行偏移计算
    switch (feature)
    {
    case OFFSET_FEATURE_BASE_COORDINATE:
        try {
            pp = calculate_offset_base_coor(pp, value, direction);
        } catch (...) {
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        break;

    case OFFSET_FEATURE_TOOL_COORDINATE:
        try {
            pp = calculate_offset_tool_coor(pp, value, direction, true);
        } catch (...) {
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        break;

    default:
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    output_pp->setValue<DoubleValueArray>(pp);

    ROS_INFO_STREAM("After offset, variable became: " << *output_pp);

    return CMD_LINE_FUNC_RSP_SUCCESS;
}