#ifndef JOB_PARSER_H
#define JOB_PARSER_H

#include <string>
#include <map>
#include <iosfwd>
#include <vector>

#include "utils.h"
#include "Segment.h"
#include "Variable.h"

enum JobLineType
{
    JOB_LINE_TYPE_BLANK, // 0
    JOB_LINE_TYPE_UNKNOWN,
    JOB_LINE_TYPE_INVALID_FORMAT,

    // ------段落定义

    JOB_LINE_TYPE_PROGRAM_BEGIN,
    JOB_LINE_TYPE_PROGRAM_END,

    JOB_LINE_TYPE_SEGMENT_BEGIN,    // 5
    JOB_LINE_TYPE_SEGMENT_END,

    JOB_LINE_TYPE_PARALLEL_BEGIN,
    JOB_LINE_TYPE_PARALLEL_END,

    JOB_LINE_TYPE_PARALLEL_SEGMENT_BEGIN,
    JOB_LINE_TYPE_PARALLEL_SEGMENT_END, // 10

    JOB_LINE_TYPE_CALCULATION_BEGIN,
    JOB_LINE_TYPE_CALCULATION_END,

    // ------指令定义

    // 机械臂指令
    JOB_LINE_TYPE_MOVEJ,
    JOB_LINE_TYPE_MOVEL,

    JOB_LINE_TYPE_MOVEJ_1,
    JOB_LINE_TYPE_MOVEJ_2,
    JOB_LINE_TYPE_MOVEJ_3,

    JOB_LINE_TYPE_MOVEL_1,
    JOB_LINE_TYPE_MOVEL_2,
    JOB_LINE_TYPE_MOVEL_2_WAITSIGNAL,
	JOB_LINE_TYPE_GETMAPOS,

    // 确认滑台位置指令
    JOB_LINE_TYPE_RECORD_SLIDER_POS,

    // 激光雷达指令
    JOB_LINE_TYPE_LIDAR_1,
    JOB_LINE_TYPE_LIDAR_2,
    JOB_LINE_TYPE_LIDARTOMA,
    JOB_LINE_TYPE_LIDARTOMA_1,
    JOB_LINE_TYPE_LIDARTOMA_2,
    JOB_LINE_TYPE_LIDARTOMA_3,

    JOB_LINE_TYPE_LIDARTOMA1_AT,
    JOB_LINE_TYPE_LIDARTOMA2_AT,
    JOB_LINE_TYPE_LIDARTOMA3_AT,

    // 手工确认指令
    JOB_LINE_TYPE_MANNUALLY_CONFIRM,

    // 延时指令
    JOB_LINE_TYPE_WAITTIME,

    // 注释行
    JOB_LINE_TYPE_COMMENT,

    // 变量操作指令
    JOB_LINE_TYPE_VAR_OP,
    JOB_LINE_TYPE_VAR_OPERATOR_ASSIGNMENT,
    JOB_LINE_TYPE_OFFSET,

    // 工具复位
    JOB_LINE_TYPE_TOOLRESET,

    // 滑台指令
    JOB_LINE_TYPE_SLIDER_MOVE,
    JOB_LINE_TYPE_SLIDER_GOHOME,

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

    JOB_LINE_TYPE_FIXEDPP_BEGIN,
    JOB_LINE_TYPE_FIXEDPP_END,

    // 空指令，用来修复并行步进模式下，并行块指令数不一致问题
    JOB_LINE_TYPE_NO_OP,

      // 运算指令
    JOB_LINE_TYPE_ADD,
    JOB_LINE_TYPE_SUB,
    JOB_LINE_TYPE_MUL,
    JOB_LINE_TYPE_DIV,
};

// 作业文件解析器
struct JobParser
{
    enum Status
    {
        JOB_STATUS_OK,
        JOB_STATUS_FILE_FOUND,
        JOB_STATUS_FILE_NOT_FOUND,
        JOB_STATUS_INVALID,
    };

    JobParser(const std::string& job_path = "");

    // 作业解析函数
    JobParser::Status parse();
    JobParser::Status parseVariables(std::string line, const JobLineType& job_line_type);

    // 输出解析结果
    void outputStatus() const;

    // 判断是否成功解析
    inline bool succeedToParse() const { return job_status_ == JOB_STATUS_OK; }

    // 根据变量值创建变量赋值语句
    std::string variableToAssignmentCommand(const std::string& var_define) const;

    friend std::ostream& operator<<(std::ostream& os, const JobParser& jp);

    // 工具函数，包装解析相关内容

    // 将变量类型名称转为实际的编码类型
    static Variable::Type TypenameToType(const std::string& type_name)
    {
        if (typename_to_type_.find(type_name) == typename_to_type_.cend())
        {
            return Variable::VARIABLE_TYPE_UNKNOWN;
        }
        return typename_to_type_[type_name];
    }

    // 获取命令行字段
    inline static std::vector<std::string> GetCommandFields(const std::string& line)
    {
        return getlines(line, ',', true);
    }

    // 获取命令字段
    inline static std::string GetCommandField(const std::string& line)
    {
        using std::string;

        const string::size_type pos = line.find(',');
        return pos == string::npos ? line : line.substr(0, pos);
    }

    // 从变量定义中提取变量名和变量类型名
    static bool ParseVariableDefine(const std::string& var_define, std::string& var_name, std::string& var_typename)
    {
        const std::vector<std::string> var_fiedls = GetVariableFields(var_define);
        if (var_fiedls.size() != 3)
        {
            return false;
        }
        var_typename = var_fiedls[1];
        var_name =  var_fiedls[2];
        return true;
    }

    // 解析命令行，获取命令类型
    static JobLineType ParseJobLineType(std::string line);

    // 创建空指令
    static std::string CreateNoOpCommand() { return "#NOOP"; }

    // 指令类别判断
    // 工具类指令
    static bool IsToolRelatedCommand(const JobLineType job_line_type);
    static bool IsToolRelatedCommandLine(const std::string& line) { return IsToolRelatedCommand(ParseJobLineType(line)); }
    // 机械臂类指令
    static bool IsArmRelatedCommand(const JobLineType job_line_type);
    static bool IsArmRelatedCommandLine(const std::string& line) { return IsArmRelatedCommand(ParseJobLineType(line)); }
    // 过程类指令
    static bool IsProcessCommand(const JobLineType job_line_type);
    static bool IsProcessCommandLine(const std::string& line) { return IsProcessCommand(ParseJobLineType(line)); }

    // 变量操作类指令
    static bool IsVarOpCommand(const JobLineType job_line_type);
    static bool IsVarOpCommandLine(const std::string& line) { return IsVarOpCommand(ParseJobLineType(line)); }

    // 变量操作类指令
    static bool IsLidarOpCommand(const JobLineType job_line_type);
    static bool IsLidarOpCommandLine(const std::string& line) { return IsLidarOpCommand(ParseJobLineType(line)); }
public:
    std::string job_path_; // 作业文件路径
    std::string job_content_; // 作业文件内容
    JobParser::Status job_status_; // 作业文件状态

    std::string program_name_; // 程序名
    Segments segments_; // 段组
    VariableTable variable_table_; // 变量表

private:
    // 解析辅助结构
    static std::map<std::string, Variable::Type> typename_to_type_;

    static std::vector<std::string> GetVariableFields(const std::string& var_define)
    {
        std::vector<std::string> var_fiedls = getlines(var_define, '_', true);
        // 变量名可能包含下划线
        if (var_fiedls.size() > 3)
        {
            for (size_t i = 3; i < var_fiedls.size(); ++i)
            {
                var_fiedls[2] += '_' + var_fiedls[i];
            }
            var_fiedls.erase(var_fiedls.begin() + 3, var_fiedls.end());
        }
        return var_fiedls;
    }
};

#endif // JOB_PARSER_H
