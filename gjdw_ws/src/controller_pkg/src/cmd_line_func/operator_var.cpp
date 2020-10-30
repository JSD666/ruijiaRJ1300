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
#include <thread>
#include <iterator>
#include <regex>

using namespace std;

// 参数名 -> 参数值
using ArgNameValueMap = std::map<std::string, std::string>;

// 加减乘除
// 指令格式为#ADD, var_TCP_tool1[1], var_TCP_tool2[1], var_TCP_tool1[2]
// 即:运算符,目标存储位置,操作数1,操作数2
CmdLineFuncRsp operator_var(SegmentScheduler& scheduler, const std::string& line)
{
    // 读取结果保存位置，操作数1和操作数2，将两个操作数的运算结果保存在指定的保存位置
   // 应该加入类判断，TCP[]=TCP[]?TCP[]或者PP[]=PP[]?PP[]或者一个类型的变量运算后放在指定位置  
    ROS_INFO("Running Command Operator_new");

    // 解析命令字段内容
    // fields为将命令行分割成几个string的数组
    vector<string> fields = JobParser::GetCommandFields(line);
    vector<string> arg_names{"CMD", "TCP", "TCP","TCP"};
    // 指令长度至少为4,若输入的命令行更长,则进行拓展
    for(int i=fields.size()-arg_names.size();i>0;i--)
    {
        arg_names.push_back("TCP");
    }
    if (fields.size() != arg_names.size())
    {
        ROS_ERROR_STREAM("Expected " << arg_names.size() << " arguments, but got " << fields.size());
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    
    // 去掉最前面的起始符#
    string operation_type = fields[0].substr(1,fields[0].size()-1);
    // 获取运算符号,"ADD","SUB","MUL","DIV"中的一种

     // 提取变量。变量解析时就创建了，因此如果这里报错，说明程序有BUG
    VariableTable& var_table = scheduler.getVariableTable();

    // 第一个是数组元素的位置
    int pos_first_array_member = -1;
    const regex num_pattern("[0-9]*.*[0-9]+");    // 数值匹配模式
    string var_name[fields.size() ], var_typename[fields.size() ];
    // 将命令行各段的类型和名字存起来
    // 用于存储下标位置
    vector<int>  element_flag(arg_names.size(),-1);
    // 用于存储第某个参数数据,第一位为结果数据
    vector<double>  numbers(arg_names.size(),0);
    // 判断是对数组元素进行操作还是对数组整体进行操作,通过检测是否含有[],若存在则确定要进行第几个元素的提取
    for(int num_fields = 1;num_fields<fields.size();num_fields++)
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
         

         // 如果除了目标位置之外的其他参数没有检测到[],则可能是单独一个int或double数字
         // 此时对其进行类型检查,之后读取数据.如果element_flag不为-1,则取对应类型数组的该下标数字.否则读取数据
        if (!JobParser::ParseVariableDefine(fields[num_fields], var_name[num_fields], var_typename[num_fields])) 
        {
            if (!std::regex_match(fields[num_fields], num_pattern))
            {
                ROS_ERROR_STREAM("Unsupported Variable Type: " << var_typename[num_fields]);
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
            }
            else
            {
            // 和模式匹配上,说明是一个直接数据
            // 第一位操作量(存储位置不允许为直接数据)
                if(num_fields==1)
                {
                    ROS_ERROR_STREAM("Not Store Position For  Result" );
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }

                double temp1 = 0;
                double temp2 = 0;
                bool decimal = false;
                bool positive = true;

                numbers[num_fields] =  stod(fields[num_fields]);
                
            }
            continue;
        }
        else
        {
        //  ROS_ERROR_STREAM(" variable type: " << var_typename[i]);
            arg_names[num_fields] = var_typename[num_fields];
        }

             // 不是直接数据,按照类型提取数据,从对应下标处获取数值
        if (var_table.find(var_name[num_fields]) == var_table.cend())
        {
            ROS_ERROR_STREAM("Cannot Find Variable In VAR_TABLE: " );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        VariablePointer& vp_temp = var_table.at(var_name[num_fields]);
        
        if (num_fields ==1)
        ROS_INFO_STREAM("Target Value before Operation:"<<*vp_temp);
        // 操作量是变量的情况下
        switch(vp_temp->getType())
        {
            case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
            case Variable::VARIABLE_TYPE_DOUBLE_6:
            {
                if(pos_first_array_member==-1)
                {
                    pos_first_array_member = num_fields;
                }
                if(arg_names[num_fields]!=arg_names[pos_first_array_member])
                {
                    // 首先对第一个位置的变量类型进行存储
                    // 之后的变量如果不是直接数据,而且还是数组元素,则应该和它类型相同,否则报错
                    // 如将PP和TCP的元素相加,一般则是错误操作
                    // 包含了第一个元素是变量的情况
                    // 在此用pos_first_array_member表示第一个数组元素的位置,让后续的数组元素和它做类型比较
                    ROS_ERROR_STREAM("Variable Type Inconsistency " );
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                const DoubleValueArray& values_temp = vp_temp->getValue<DoubleValueArray>();
                if(element_flag[num_fields]==-1)
                {
                    // 变量名是数组,如果含有[]则前面已经做出判断和处理
                    // 此处标志位为-1,只能说明变量名时数组,但是不含[],所以不是对数组元素进行操作,而是传入整个数组
                    // 返回错误不做处理.
                    ROS_ERROR_STREAM("Array Parameter  Does Not Contain Index" );
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                numbers[num_fields] = values_temp[element_flag[num_fields]];
                break;
            }
            case Variable::VARIABLE_TYPE_DOUBLE:
            {
                // 防止变量不是数组,缺带着[]的情况,如var_INT_varname[2]等
                // element_flag不为-1,说明含有合法的[x],但变量类型不是数组,报错
                if(element_flag[num_fields]!=-1)
                {
                    ROS_ERROR_STREAM("Argument "<<fields[num_fields]<<" is not array but has '[]'" );
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                numbers[num_fields] = vp_temp->getValue<DoubleValue>();
                break;
            }
            case Variable::VARIABLE_TYPE_INT:
            {
                // 防止变量不是数组,缺带着[]的情况,如var_INT_varname[2]等
                // element_flag不为-1,说明含有合法的[x],但变量类型不是数组,报错
                if(element_flag[num_fields]!=-1)
                {
                    ROS_ERROR_STREAM("Argument "<<fields[num_fields]<<" is not array but has '[]'" );
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                numbers[num_fields] = vp_temp->getValue<IntValue>();
                break;
            }
            default:
                ROS_ERROR_STREAM("Unsupported Variable Type: " );
                return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }         
    }

    

    //答案初始值存放第一个操作数(指令的第三个位置,下标为2)的值,指令长度至少为4
    numbers[1] = numbers[2];
    ROS_INFO_STREAM("Operating. Operation Type: "<<operation_type );
    if(operation_type=="ADD")
    {
        for(int i=3;i<arg_names.size();i++)
        {
             numbers[1] += numbers[i];
        }
    }
    else if(operation_type=="SUB")
    {
        for(int i=3;i<arg_names.size();i++)
        {
             numbers[1] -= numbers[i];
        }
    }
    else if(operation_type=="MUL")
    {
        for(int i=3;i<arg_names.size();i++)
        {
             numbers[1] *= numbers[i];
        }
    }
    else if(operation_type=="DIV")
    {
        for(int i=3;i<arg_names.size();i++)
        {
            if(numbers[i]==0){
                ROS_ERROR_STREAM("Divide By Zero Exception" );
                return CMD_LINE_FUNC_RSP_FAILED;
            }
             numbers[1] /= numbers[i];
        }
    }
    else
    {
        ROS_ERROR_STREAM("Illegal Operator " );
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 指向于目标存储位置,前面已经做过类型检查(第一个元素必须是变量,否则报错;是变量的情况,则将是运算结果的存储位置)
    VariablePointer& vp_target = var_table.at(var_name[1]);
    if(element_flag[1]!=-1)
    {
        //是数组元素,则先拷贝值,再修改对应下标的值,再赋值放回去
        const DoubleValueArray& values_temp = vp_target->getValue<DoubleValueArray>();
        DoubleValueArray temp(6);
        for(int i=0;i<6;i++)
        {
            temp[i] = values_temp[i];
        }
        temp[element_flag[1]] = numbers[1];
         vp_target->setValue(temp);
    }
    else
    {
        //不是数组元素,则存储位置只是一个变量
        //ROS_INFO_STREAM("Answer:"<<numbers[1]);
        if(vp_target->getType()==Variable::VARIABLE_TYPE_INT)
             vp_target->setValue(int(numbers[1]));
        else if(vp_target->getType()==Variable::VARIABLE_TYPE_DOUBLE)
             vp_target->setValue(double(numbers[1]));
    }
    
    ROS_INFO_STREAM("Target Value After Operation:"<<*vp_target);
    // ROS_ERROR_STREAM("Target Value:"<<*vp_target);
    return CMD_LINE_FUNC_RSP_SUCCESS;
}


