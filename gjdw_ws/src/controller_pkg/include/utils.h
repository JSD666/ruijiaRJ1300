#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <algorithm>
#include <codecvt>
#include <cstdlib>
#include <memory>

#include "ros/ros.h"
#include <time.h>

#include "fstream"
#include "iostream"
#include <sys/stat.h>  
#include <sys/types.h>

#define DEBUG 1

// trim from start (in place)
static inline void ltrim(std::string &s) 
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

// 检查字符串是否包含指定的子串
static inline bool contains_of(const std::string& str, const char* substr)
{
    return str.find(substr) != std::string::npos;
}

// 读取指定路径的文本文件，返回string
std::string read_file(const char* path, const char* delim = "\n");

// 从字符串中读取行，将content视为流
bool getline(std::string& content, std::string& line, const char delim = '\n');

// 从字符串中读取行，将所有行存入容器并返回它
std::vector<std::string> getlines(std::string content, const char delim = '\n', const bool need_trim = true);

// 格式化生成string
std::string format(const char *fmt, ...);

// 解析cmdRsp
bool get_cmd_rsp_errcode(const std::string& json_str, int& err_code);

// 发送命令回复信息给地面站
void send_command_response_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                             const int value = 0, const int status = 0);
// 主控系统心跳发布
void send_heart_beat(ros::Publisher& pub);

// 发送命令信息给地面站
void send_command_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                    const int value = 0, const char* name = "");

// 发送命令信息给地面站
void send_info_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                    std::string value = "" , const char* name = "");

// 发送命令指令给接线工具
void send_command_to_clamp_wire_tool(ros::Publisher& pub, const int seq,
                                     const unsigned int value = 0, const char* name = "");

// 发送命令指令给剪线工具
void send_command_to_cut_wire_tool(ros::Publisher& pub, const int seq,
                                   const unsigned int value = 0, const char* name = "");

// 发送命令指令给夹爪工具
void send_command_to_claw_tool(ros::Publisher& pub, const int seq,
                                  const unsigned int value = 0, const char* name = "");

// 发送命令指令给剥线工具
void send_command_to_strip_wire_tool(ros::Publisher& pub, const int seq,
	const unsigned int value = 0, const char* name = "");

// 发送命令指令给电源管理结点
void send_command_to_power_manager(ros::Publisher& pub, const int seq,
	                               const unsigned int value = 0, const char* name = "");

// 发送命令给机械臂管理节点
void send_command_to_robot_script(ros::Publisher& pub, const unsigned int CtrType,
	                                std::string strUrScript,const int arm_id);

// 电源管理节点控制机械臂开机等服务
bool call_power_control_srv(ros::ServiceClient& service_clients,const int command_id, 
                                   const unsigned int arm_id = 0, const std::string&  name = "");
// 读取指定脚本并发送给机械臂执行
void exec_arm_script(ros::Publisher& pub, const std::string& script_path,
                        const unsigned int CtrType,const int arm_id);

int current_time(int ti = 0);

std::wstring s2ws(const std::string& str);
std::string ws2s(const std::wstring& w_str);
std::string UnicodeToUTF8(const std::wstring & wstr);

// 发送命令信息给地面站
//void send_info_to_ground_station(ros::Publisher& pub, const int seq,
  //                                  const char* value, const char* name);

void creat_logfile(const unsigned int type = 0, const std::string& logtxt = "" );
void write_content(const char* begin_name, const char* filepath, const char* msg_begin , const std::string& log_content);

#endif // UTILS_H