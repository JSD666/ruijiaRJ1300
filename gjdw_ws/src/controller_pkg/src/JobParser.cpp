#include "JobParser.h"
#include "CommandBlock.h"
#include "utils.h"

#include <iostream>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <set>
#include <regex>

using namespace std;

// 设定变量类型解析规则
std::map<std::string, Variable::Type> JobParser::typename_to_type_  = {
    {"INT", Variable::VARIABLE_TYPE_INT},
    {"DOUBLE", Variable::VARIABLE_TYPE_DOUBLE},
    // {"DOUBLEA", Variable::VARIABLE_TYPE_DOUBLE_ARRAY},
    {"PP", Variable::VARIABLE_TYPE_DOUBLE_6},
    {"TCP", Variable::VARIABLE_TYPE_DOUBLE_6},
    {"Joint", Variable::VARIABLE_TYPE_DOUBLE_6},
};

// JobParser构造函数，读取job文件并解析
JobParser::JobParser(const std::string& job_path)
    : job_path_(job_path), job_status_(JOB_STATUS_OK)
{
    if (job_path.empty()) return;

    // 读取作业文件
    job_content_ = read_file(job_path_.c_str());
    if (job_content_.size() == 0) // 找不到文件（或为空）
    {
        job_status_ = JOB_STATUS_FILE_NOT_FOUND;
        outputStatus();
        return;
    }

    // 调用解析函数
    job_status_ = parse();
    if (job_status_ != JOB_STATUS_OK)
    {
        outputStatus();
        return;
    }
}

// 解析作业行，获取命令类型
JobLineType JobParser::ParseJobLineType(std::string line)
{
    // 去除行头行尾的空白字符
    trim(line);

    // 空行
    if (line.empty()) return JOB_LINE_TYPE_BLANK;

    // 部分命令第一个字段是它的名称，故可以使用map加速查询
    {
        static const std::map<std::string, JobLineType> job_line_type_pattern_table_ = {
            { "#PARALLEL BEGIN",        JOB_LINE_TYPE_PARALLEL_BEGIN },
            { "#PARALLEL END",          JOB_LINE_TYPE_PARALLEL_END },
            { "#CALCULATION BEGIN",     JOB_LINE_TYPE_CALCULATION_BEGIN },
            { "#CALCULATION END",       JOB_LINE_TYPE_CALCULATION_END },

            { "#MOVEL",     JOB_LINE_TYPE_MOVEL },
            { "#MOVEL1",    JOB_LINE_TYPE_MOVEL_1 },
            { "#MOVEL2",    JOB_LINE_TYPE_MOVEL_2 },
            { "#MOVEL2_WAITSIGNAL",    JOB_LINE_TYPE_MOVEL_2_WAITSIGNAL },
            { "#MOVEJ",     JOB_LINE_TYPE_MOVEJ },
            { "#MOVEJ1",    JOB_LINE_TYPE_MOVEJ_1 },
            { "#MOVEJ2",    JOB_LINE_TYPE_MOVEJ_2 },
            { "#MOVEJ3",    JOB_LINE_TYPE_MOVEJ_3 },
            { "#GETMAPOS",    JOB_LINE_TYPE_GETMAPOS },

            { "#SLIDERMOVE",        JOB_LINE_TYPE_SLIDER_MOVE },
            { "#SLIDERGOHOME",      JOB_LINE_TYPE_SLIDER_GOHOME },
            { "#RECORDSLIDERPOS",   JOB_LINE_TYPE_RECORD_SLIDER_POS },

            { "#LIDAR1",        JOB_LINE_TYPE_LIDAR_1 },
            { "#LIDAR2",        JOB_LINE_TYPE_LIDAR_2 },
            { "#LIDARTOMA",     JOB_LINE_TYPE_LIDARTOMA },
            { "#LIDARTOMA1",    JOB_LINE_TYPE_LIDARTOMA_1 },
            { "#LIDARTOMA2",    JOB_LINE_TYPE_LIDARTOMA_2 },
            { "#LIDARTOMA3",    JOB_LINE_TYPE_LIDARTOMA_3 },

            { "#LIDARTOMA1_AT",    JOB_LINE_TYPE_LIDARTOMA1_AT },
            { "#LIDARTOMA2_AT",    JOB_LINE_TYPE_LIDARTOMA2_AT },
            { "#LIDARTOMA3_AT",    JOB_LINE_TYPE_LIDARTOMA3_AT },

            { "#OFFSET",            JOB_LINE_TYPE_OFFSET },
            { "#WAITTIME",          JOB_LINE_TYPE_WAITTIME },
            { "#MANUALLY CONFIRM",  JOB_LINE_TYPE_MANNUALLY_CONFIRM },
            { "#TOOLRESET",         JOB_LINE_TYPE_TOOLRESET },

            { "#STRIP_ONEKEY",       JOB_LINE_TYPE_STRIP_ONEKEY },
            { "#STRIP_INIT",         JOB_LINE_TYPE_STRIP_INIT },
            { "#STRIP_ROTATE",       JOB_LINE_TYPE_STRIP_ROTATE },
            { "#STRIP_BACKROTATE",   JOB_LINE_TYPE_STRIP_BACKROTATE },
            { "#STRIP_UPWARD",       JOB_LINE_TYPE_STRIP_UPWARD },
            { "#STRIP_LINKLEFT",     JOB_LINE_TYPE_STRIP_LINKLEFT },
            { "#STRIP_LINKRIGHT",    JOB_LINE_TYPE_STRIP_LINKRIGHT },
            { "#STRIP_LINKUP",       JOB_LINE_TYPE_STRIP_LINKUP },
            { "#STRIP_LINKDOWN",     JOB_LINE_TYPE_STRIP_LINKDOWN },
            { "#STRIP_STOP",         JOB_LINE_TYPE_STRIP_STOP },

            { "#CONNECT_TIGHTENSTART",       JOB_LINE_TYPE_CONNECT_TIGHTENSTART },
            { "#CONNECT_TIGHTENSTOP",        JOB_LINE_TYPE_CONNECT_TIGHTENSTOP },
            { "#CONNECT_TIGHTENBACK",        JOB_LINE_TYPE_CONNECT_TIGHTENBACK },
            { "#CONNECT_TIGHTENINIT",        JOB_LINE_TYPE_CONNECT_TIGHTENINIT },
            { "#CONNECT_CLIPLOCK",           JOB_LINE_TYPE_CONNECT_CLIPLOCK },
            { "#CONNECT_CLIPUNLOCK",         JOB_LINE_TYPE_CONNECT_CLIPUNLOCK },
            { "#CONNECT_SLEEVELOCK",         JOB_LINE_TYPE_CONNECT_SLEEVELOCK },
            { "#CONNECT_SLEEVEUNLOCK",       JOB_LINE_TYPE_CONNECT_SLEEVEUNLOCK },
            { "#CONNECT_SLEEVESTOP",         JOB_LINE_TYPE_CONNECT_SLEEVESTOP },
            { "#CONNECT_BRANCHLINELOCK",     JOB_LINE_TYPE_CONNECT_BRANCHLINELOCK },
            { "#CONNECT_BRANCHLINEUNLOCK",   JOB_LINE_TYPE_CONNECT_BRANCHLINEUNLOCK },

            { "#CUT_START",         JOB_LINE_TYPE_CUT_START },
            { "#CUT_INIT",          JOB_LINE_TYPE_CUT_INIT },
            { "#OPENGRIPPER",       JOB_LINE_TYPE_OPENGRIPPER },
            { "#CLOSEGRIPPER",      JOB_LINE_TYPE_CLOSEGRIPPER },

            { "#FIXEDPP BEGIN",     JOB_LINE_TYPE_FIXEDPP_BEGIN },
            { "#FIXEDPP END",       JOB_LINE_TYPE_FIXEDPP_END },

            { "#NOOP",       JOB_LINE_TYPE_NO_OP },
            
            {"#ADD", JOB_LINE_TYPE_ADD},
            {"#SUB", JOB_LINE_TYPE_SUB},
            {"#MUL", JOB_LINE_TYPE_MUL},
            {"#DIV", JOB_LINE_TYPE_DIV},
        };

        const string& command_name = GetCommandField(line);
        if (job_line_type_pattern_table_.find(command_name) != job_line_type_pattern_table_.cend())
        {
            return job_line_type_pattern_table_.at(command_name);
        }
    }

    JobLineType job_line_type = JOB_LINE_TYPE_INVALID_FORMAT;

    // 通过字符串匹配实现语法分析
    if (contains_of(line, "//"))
    {
        job_line_type = JOB_LINE_TYPE_COMMENT;
    }
    else if (contains_of(line, "#BOX_POSITION"))
    {
        job_line_type = JOB_LINE_TYPE_COMMENT;
    }
    else if (contains_of(line, "#PROGRAM"))
    {
        // const regex pattern("^#PROGRAM [a-zA-Z0-9_]+ ([A-Z]+)$");

        // std::smatch matches;
        // if (std::regex_match(line, matches, pattern) && matches.size() == 2)
        // {
        //     if (matches[1].str() == "BEGIN")
        //         job_line_type = JOB_LINE_TYPE_PROGRAM_BEGIN;
        //     else if (matches[1].str() == "END")
        //         job_line_type = JOB_LINE_TYPE_PROGRAM_END;
        // }

        if (contains_of(line, "BEGIN"))
            job_line_type = JOB_LINE_TYPE_PROGRAM_BEGIN;
        else if (contains_of(line, "END"))
            job_line_type = JOB_LINE_TYPE_PROGRAM_END;
    }
    else if (contains_of(line, "#SEGMENT"))
    {
        const regex pattern("^#SEGMENT [0-9]+ [a-zA-Z0-9_]+ ([A-Z]+)$");

        std::smatch matches;
        if (std::regex_match(line, matches, pattern) && matches.size() == 2)
        {
            if (matches[1].str() == "BEGIN")
                job_line_type = JOB_LINE_TYPE_SEGMENT_BEGIN;
            else if (matches[1].str() == "END")
                job_line_type = JOB_LINE_TYPE_SEGMENT_END;
        }
    }
    else if (contains_of(line, "#PARALLEL SEGMENT"))
    {
        const regex pattern("^#PARALLEL SEGMENT [0-9]+ ([A-Z]+)$");

        std::smatch matches;
        if (std::regex_match(line, matches, pattern) && matches.size() == 2)
        {
            if (matches[1].str() == "BEGIN")
                job_line_type = JOB_LINE_TYPE_PARALLEL_SEGMENT_BEGIN;
            else if (matches[1].str() == "END")
                job_line_type = JOB_LINE_TYPE_PARALLEL_SEGMENT_END;
        }
    }
    else if (contains_of(line, "="))
    {
        job_line_type = JOB_LINE_TYPE_VAR_OPERATOR_ASSIGNMENT;
    }
    else  // 这里处理未定义行
    {
        job_line_type = JOB_LINE_TYPE_UNKNOWN;
    }

    return job_line_type;
}

// 解析函数
// TODO: 细化错误类型
JobParser::Status JobParser::parse()
{
    string line;
    string content = job_content_;
    JobLineType job_line_type = JOB_LINE_TYPE_BLANK;
    
    // 用于控制语法解析流程
    bool parse_ok = true;
    bool program_close = true;
    bool segment_close = true;
    bool block_close = true;
    bool parallel_segment_close = true;
    Segment current_segment;
    CommandBlock current_block;
    CommandBlock current_parallel_block;

    // 逐行解析
    size_t line_no = 0;
    while (parse_ok && getline(content, line))
    {
        ++line_no;

        if (DEBUG) cout << "Parsing line " << line_no << ": " << line << endl;

        job_line_type = ParseJobLineType(line);

        if (JOB_LINE_TYPE_BLANK == job_line_type || JOB_LINE_TYPE_COMMENT == job_line_type) // 跳过空白行和注释行
            continue;

        if (!segment_close && job_line_type != JOB_LINE_TYPE_SEGMENT_END) // 段BEGIN，将内容存储到段内
        {
            current_segment.content_ += line + '\n';

            // 满足这里判断条件的是真正的命令行
            if (job_line_type != JOB_LINE_TYPE_PARALLEL_BEGIN && job_line_type != JOB_LINE_TYPE_PARALLEL_END
                && job_line_type != JOB_LINE_TYPE_CALCULATION_BEGIN && job_line_type != JOB_LINE_TYPE_CALCULATION_END)
            {
                // 段内容都归属某个块
                current_block.addCommandLine(CommandLine(line, line_no));

                // 解析变量
                if (parseVariables(line, job_line_type) != JOB_STATUS_OK)
                {
                    cerr << "ERROR: variable definition invalid" << endl;
                    parse_ok = false;
                    break;
                }
            }
        }
        if (!parallel_segment_close && job_line_type != JOB_LINE_TYPE_PARALLEL_SEGMENT_END) // 并行段BEGIN，将内容存储到段内
        {
            current_parallel_block.addCommandLine(CommandLine(line, line_no));
        }

        // 解析命令行
        switch (job_line_type)
        {
        case JOB_LINE_TYPE_PROGRAM_BEGIN:
            if (!program_close) // BEGIN END 不匹配
            {
                parse_ok = false;
                break;
            }

            program_close = false;
            try {
                istringstream iss(line);
                string _w; // 占位符，用来过滤不需要的输入
                iss >> _w >> program_name_;
            } catch(const exception&) {
                parse_ok = false;
            }
            break;
        
        case JOB_LINE_TYPE_PROGRAM_END:
            if (program_close) // BEGIN END 不匹配 
            {
                parse_ok = false;
                break;
            }

            try {   // 检查程序名是否匹配
                istringstream iss(line);
                string _w; // 占位符，用来过滤不需要的输入
                string name;
                iss >> _w >> name;
                if (name != program_name_)
                {
                    parse_ok = false;
                    break;
                }
            } catch(const exception&) {
                parse_ok = false;
            }
            // 成功解析程序结构
            program_close = true;
            break;
        
        case JOB_LINE_TYPE_SEGMENT_BEGIN:
            if (!segment_close || current_segment.no_ != -1 
                || !current_segment.name_.empty()) // BEGIN END 不匹配
            {
                parse_ok = false;
                break;
            }

            segment_close = false;
            try {
                istringstream iss(line);
                string _w; // 占位符，用来过滤不需要的输入
                iss >> _w >> current_segment.no_ >> current_segment.name_;

            } catch(const exception&) {
                parse_ok = false;
            }
            break;
        
        case JOB_LINE_TYPE_SEGMENT_END:
            if (segment_close) // BEGIN END 不匹配 
            {
                parse_ok = false;
                break;
            }
            try {   // 检查段名和段号是否匹配
                string segment_name;
                int segment_no;

                istringstream iss(line);
                string _w; // 占位符，用来过滤不需要的输入
                iss >> _w >> segment_no >> segment_name;
                if (segment_name != current_segment.name_ || segment_no != current_segment.no_)
                {
                    parse_ok = false;
                    break;
                }
            } catch(const exception&) {
                parse_ok = false;
            }
            // 成功解析段结构
            // 段尾包含顺序代码，在此处理
            if (!current_block.empty())
            {
                current_segment.blocks_.push_back(current_block);
                current_block.clear();
            }
            segments_.push_back(current_segment);

            segment_close = true;
            current_segment.clear();
            break;
        
        case JOB_LINE_TYPE_PARALLEL_BEGIN:
        case JOB_LINE_TYPE_CALCULATION_BEGIN:
            if (!block_close)
            {
                parse_ok = false;
                break;
            }
            block_close = false;
            // 读到块开始标志时，可能已经读入了顺序行，因此需要在此处理
            if (!current_block.empty())
            {
                current_segment.blocks_.push_back(current_block);
                current_block.clear();
            }
            break;
        
        case JOB_LINE_TYPE_PARALLEL_END:
        case JOB_LINE_TYPE_CALCULATION_END:
            if (block_close) // BEGIN END 不匹配 
            {
                parse_ok = false;
                break;
            }
            // 成功解析块结构
            current_block.setType(JOB_LINE_TYPE_CALCULATION_END == job_line_type ?
                                  CommandBlock::COMMAND_BLOCK_TYPE_CALCULATION :
                                  CommandBlock::COMMAND_BLOCK_TYPE_PARALLEL);
            current_segment.blocks_.push_back(current_block);

            block_close = true;
            current_block.clear();
            break;

        case JOB_LINE_TYPE_PARALLEL_SEGMENT_BEGIN:
            if (!parallel_segment_close)
            {
                parse_ok = false;
                break;
            }
            parallel_segment_close = false;
            break;
        
        case JOB_LINE_TYPE_PARALLEL_SEGMENT_END:
            if (parallel_segment_close) // BEGIN END 不匹配 
            {
                parse_ok = false;
                break;
            }
            // 成功解析并行段结构
            current_segment.parallel_blocks_.push_back(current_parallel_block);

            parallel_segment_close = true;
            current_parallel_block.clear();
            break;

        case JOB_LINE_TYPE_INVALID_FORMAT:
        case JOB_LINE_TYPE_UNKNOWN:
            parse_ok = false;
            break;

        default:
            break;
        }
    }

    // 存在只有BEGIN，没有END的情况
    if (!program_close && content.empty())
    {
        cerr << "ERROR: missing #RPOGRAM " << program_name_ << " END" << endl;
        parse_ok = false;
    }
    if (!segment_close && content.empty())
    {
        cerr << "ERROR: missing #SEGMENT " << current_segment.no_ << " " << current_segment.name_ << " END" << endl;
        parse_ok = false;
    }
    if (!block_close && content.empty())
    {
        if (CommandBlock::COMMAND_BLOCK_TYPE_CALCULATION == current_block.getType())
            cerr << "ERROR: missing #CALCULATION END" << endl; 
        else
            cerr << "ERROR: missing #PARALLEL END" << endl; 
        parse_ok = false;
    }
    if (!parallel_segment_close && content.empty())
    {
        cerr << "ERROR: missing #PARALLEL SEGMENT END" << endl;
        parse_ok = false;
    }

    if (!parse_ok)  // 作业文件内容有误
    {
        if (!line.empty())
        {
            cerr << "ERROR: failed to parse line " << line << endl;
        }
        return JOB_STATUS_INVALID;
    }
    return JOB_STATUS_OK;
}

JobParser::Status JobParser::parseVariables(std::string line, const JobLineType& job_line_type)
{
    // 一行可能有多个变量，循环处理
    for (string::size_type index = line.find("var_"); index != string::npos; index = line.find("var_"))
    {
        line = line.substr(index);

        // 提取变量定义字符串
        string var_define;
        char delim = ',';

        // 因为允许变量间进行操作，因此需要根据操作符分割不同变量的定义
        if (JOB_LINE_TYPE_VAR_OPERATOR_ASSIGNMENT == job_line_type)
        {
            delim = '=';
        }
        if (!getline(line, var_define, delim)) // 万一有BUG
        {
            cerr << "ERROR: parseVariables has inner error" << endl;
            return JOB_STATUS_INVALID;
        }

        // 提取变量字段
        string var_name, var_typename;
        if (!ParseVariableDefine(var_define, var_name, var_typename))
        {
            cerr << "ERROR: variable definition invalid: " << var_define << endl;
            return JOB_STATUS_INVALID;
        }

        // 检查变量类型
        const Variable::Type var_type = TypenameToType(var_typename);
        if (var_type == Variable::VARIABLE_TYPE_UNKNOWN)
        {
            cerr << "ERROR: unsupport variable type: " << var_typename << endl;
            return JOB_STATUS_INVALID;
        }

        // 若变量已创建，检查类型是否匹配；否则，创建它
        if (variable_table_.find(var_name) != variable_table_.end())
        {
            const VariablePointer vp = variable_table_[var_name];
            if (vp->getType() != var_type) // 变量重名但类型不同，认为是语法错误
            {
                cerr << "ERROR: variable " << var_name << " already defined, conflicts with:\n" 
                     << *vp << endl;
                return JOB_STATUS_INVALID;
            }
        }
        try {
            VariablePointer vp = VariablePointer(new Variable(var_name, var_typename, var_type));
            variable_table_[var_name] = vp;
        } catch (const std::exception& e) {
            cerr << "ERROR: failed to new Variable, skipped: " << e.what() << endl;
        }
    }

    return JOB_STATUS_OK;
}

// JobParser调试函数，格式化输出文件状态
void JobParser::outputStatus() const
{
    switch (job_status_)
    {
    case JOB_STATUS_OK:
        cout << "Parser: parse job file success" << endl;
        break;

    case JOB_STATUS_FILE_FOUND:
        cout << "Parser: read job file success" << endl;
        break;

    case JOB_STATUS_FILE_NOT_FOUND:
        cerr << "Parser: cannot find job file" << endl;
        break;

    case JOB_STATUS_INVALID:
        cerr << "Parser: job file invalid" << endl;
        break;

    default:
        cerr << "Parser: internal error, unrecognized job status" << endl;
        break;
    }
}

std::ostream& operator<<(std::ostream& os, const JobParser& jp)
{
    std::copy(jp.segments_.cbegin(), jp.segments_.cend(), ostream_iterator<Segment>(os, "\n"));
    // 输出解析到的变量
    os << jp.variable_table_;
    return os;
}

// 工具类指令
bool JobParser::IsToolRelatedCommand(const JobLineType job_line_type)
{
    static std::set<JobLineType> commands = {
        // 工具复位
        JOB_LINE_TYPE_TOOLRESET,

        // 剥线工具
        JOB_LINE_TYPE_STRIP_ONEKEY,
        JOB_LINE_TYPE_STRIP_INIT,
        JOB_LINE_TYPE_STRIP_ROTATE,
        JOB_LINE_TYPE_STRIP_BACKROTATE,
        JOB_LINE_TYPE_STRIP_UPWARD,
        JOB_LINE_TYPE_STRIP_LINKLEFT,
        JOB_LINE_TYPE_STRIP_LINKRIGHT,
        JOB_LINE_TYPE_STRIP_LINKUP,
        JOB_LINE_TYPE_STRIP_LINKDOWN,
        JOB_LINE_TYPE_STRIP_STOP,

        // 接线工具
        JOB_LINE_TYPE_CONNECT_TIGHTENSTART,
        JOB_LINE_TYPE_CONNECT_TIGHTENSTOP,
        JOB_LINE_TYPE_CONNECT_TIGHTENBACK,
        JOB_LINE_TYPE_CONNECT_TIGHTENINIT,
        JOB_LINE_TYPE_CONNECT_CLIPLOCK,
        JOB_LINE_TYPE_CONNECT_CLIPUNLOCK,
        JOB_LINE_TYPE_CONNECT_SLEEVELOCK,
        JOB_LINE_TYPE_CONNECT_SLEEVEUNLOCK,
        JOB_LINE_TYPE_CONNECT_SLEEVESTOP,
        JOB_LINE_TYPE_CONNECT_BRANCHLINELOCK,
        JOB_LINE_TYPE_CONNECT_BRANCHLINEUNLOCK,

        // 剪线工具
        JOB_LINE_TYPE_CUT_START,
        JOB_LINE_TYPE_CUT_INIT,

        // 夹爪工具
        JOB_LINE_TYPE_OPENGRIPPER,
        JOB_LINE_TYPE_CLOSEGRIPPER,

        // 滑台
        JOB_LINE_TYPE_SLIDER_MOVE,
        JOB_LINE_TYPE_SLIDER_GOHOME,
    };

    return commands.find(job_line_type) != commands.cend();
}

// 机械臂类指令
bool JobParser::IsArmRelatedCommand(const JobLineType job_line_type)
{
    static std::set<JobLineType> commands = {
        // 机械臂指令
        JOB_LINE_TYPE_MOVEJ,
        JOB_LINE_TYPE_MOVEL,

        JOB_LINE_TYPE_MOVEJ_1,
        JOB_LINE_TYPE_MOVEJ_2,
        JOB_LINE_TYPE_MOVEJ_3,

        JOB_LINE_TYPE_MOVEL_1,
        JOB_LINE_TYPE_MOVEL_2,
        JOB_LINE_TYPE_MOVEL_2_WAITSIGNAL,

        // 确认滑台位置指令
        //JOB_LINE_TYPE_RECORD_SLIDER_POS,

        // 激光雷达指令
        // JOB_LINE_TYPE_LIDAR_1,
        // JOB_LINE_TYPE_LIDAR_2,
        // JOB_LINE_TYPE_LIDARTOMA,
        // JOB_LINE_TYPE_LIDARTOMA_1,
        // JOB_LINE_TYPE_LIDARTOMA_2,
        // JOB_LINE_TYPE_LIDARTOMA_3,
    };

    return commands.find(job_line_type) != commands.cend();
}

// 过程类指令
bool JobParser::IsProcessCommand(const JobLineType job_line_type)
{
    static std::set<JobLineType> commands = {
        // 延时指令
        JOB_LINE_TYPE_WAITTIME,
    };

    return IsVarOpCommand(job_line_type) || commands.find(job_line_type) != commands.cend();
}

// 变量操作类指令
bool JobParser::IsVarOpCommand(const JobLineType job_line_type)
{
    static std::set<JobLineType> commands = {
        // 变量操作指令
        JOB_LINE_TYPE_VAR_OP,
        JOB_LINE_TYPE_VAR_OPERATOR_ASSIGNMENT,
        JOB_LINE_TYPE_OFFSET,
    };

    return commands.find(job_line_type) != commands.cend();
}

bool JobParser::IsLidarOpCommand(const JobLineType job_line_type)
{
    static std::set<JobLineType> commands = {
        // 变量操作指令
        JOB_LINE_TYPE_LIDAR_1,
        JOB_LINE_TYPE_LIDAR_2,
        JOB_LINE_TYPE_LIDARTOMA,
        JOB_LINE_TYPE_LIDARTOMA_1,
        JOB_LINE_TYPE_LIDARTOMA_2,
        JOB_LINE_TYPE_LIDARTOMA_3,
        JOB_LINE_TYPE_LIDARTOMA1_AT,
        JOB_LINE_TYPE_LIDARTOMA2_AT,
        JOB_LINE_TYPE_LIDARTOMA3_AT,

    };

    return commands.find(job_line_type) != commands.cend();
}


// 根据变量值创建变量赋值语句
std::string JobParser::variableToAssignmentCommand(const std::string& var_define) const
{
    string var_name, var_typename;
    if (!JobParser::ParseVariableDefine(var_define, var_name, var_typename)) return "";

    assert (variable_table_.find(var_name) != variable_table_.cend());
    VariablePointer vp = variable_table_.at(var_name);

    string value_str;
    switch (vp->getType())
    {
    case Variable::VARIABLE_TYPE_DOUBLE:
        value_str = std::to_string(vp->getValue<DoubleValue>());
        break;

    case Variable::VARIABLE_TYPE_INT:
        value_str = std::to_string(vp->getValue<IntValue>());
        break;

    case Variable::VARIABLE_TYPE_DOUBLE_6:
    case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
        {
            const DoubleValueArray& doubles = vp->getValue<DoubleValueArray>();
            for (size_t i = 0; i < doubles.size(); ++i)
            {
                if (i != 0)
                {
                    value_str += ", ";
                }
                value_str += std::to_string(doubles[i]);
            }
        }
        break;

    default:
        cerr << "Unsupported variable type: " << var_typename << endl;
        return "";
    }

    return format("#%s = %s", var_define.c_str(), value_str.c_str());
}
