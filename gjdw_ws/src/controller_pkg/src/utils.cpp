#include "utils.h"
#include "config.h"

#include "std_msgs/String.h"
#include "public_pkg/cmd_msg.h"
#include "public_pkg/cmd_rsp_msg.h"
#include "public_pkg/status_digital_msg.h"
#include "public_pkg/heart_beat_msg.h"
#include "public_pkg/arms_power_srv.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h" 
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <iostream>
#include <fstream>
#include <cstdarg>
#include <vector>

using namespace std;

// 格式化生成string，内部实现
static std::string vformat(const char *fmt, va_list ap);

// 读取指定路径的文本文件，返回string
std::string read_file(const char* path, const char* delim)
{
    ifstream ifs(path);
    if (!ifs)
    {
        cerr << "Cannot open script file: " << path << endl;
        return "";
    }
    string content, line;
    while (getline(ifs, line)) // TODO: 有性能考虑的话，使用Buffer
    {
        content += line + delim;
    }
    return content;
}

// 从字符串中读取行
bool getline(std::string& content, std::string& line, const char delim)
{
    if (content.empty()) return false;

    string::size_type pos = content.find(delim);
    if (pos == string::npos)
    {
        line = content;
        content = "";
    }
    else
    {
        line = content.substr(0, pos);
        content = content.substr(pos + 1, content.size() - pos); // pos+1以去掉delim
    }
    return true;
}


// 从字符串中读取行，将所有行存入容器并返回它
std::vector<std::string> getlines(std::string content, const char delim, const bool need_trim)
{
    vector<string> lines;
    string line;
    while (getline(content, line, delim))
    {
        if (need_trim) trim(line);

        lines.push_back(line);
    }
    return lines;
}

// 格式化生成string，内部实现
// See: https://stackoverflow.com/questions/69738/c-how-to-get-fprintf-results-as-a-stdstring-w-o-sprintf#69911
static std::string vformat(const char *fmt, va_list ap)
{
    // Allocate a buffer on the stack that's big enough for us almost
    // all the time.  Be prepared to allocate dynamically if it doesn't fit.
    size_t size = 1024;
    char stackbuf[1024];
    std::vector<char> dynamicbuf;
    char *buf = &stackbuf[0];
    va_list ap_copy;

    while (1) {
        // Try to vsnprintf into our buffer.
        va_copy(ap_copy, ap);
        int needed = vsnprintf (buf, size, fmt, ap);
        va_end(ap_copy);

        // NB. C99 (which modern Linux and OS X follow) says vsnprintf
        // failure returns the length it would have needed.  But older
        // glibc and current Windows return -1 for failure, i.e., not
        // telling us how much was needed.

        if (needed <= (int)size && needed >= 0) {
            // It fit fine so we're done.
            return std::string (buf, (size_t) needed);
        }

        // vsnprintf reported that it wanted to write more characters
        // than we allotted.  So try again using a dynamic buffer.  This
        // doesn't happen very often if we chose our initial size well.
        size = (needed > 0) ? (needed+1) : (size*2);
        dynamicbuf.resize (size);
        buf = &dynamicbuf[0];
    }
}

// 格式化生成string
std::string format(const char *fmt, ...)
{
    va_list ap;
    va_start (ap, fmt);
    std::string buf = vformat (fmt, ap);
    va_end (ap);
    return buf;
}

// 解析cmdRsp
bool get_cmd_rsp_errcode(const std::string& json_str, int& err_code)
{
    rapidjson::Document d;
    d.Parse(json_str.c_str());

    if (!d.IsObject())
    {
        if (DEBUG)
            cerr << "Failed to parse ducument for " << json_str << endl;
        return false;
    }

    const rapidjson::Value& cmd_rsp = d["cmdRsp"];
    if (!cmd_rsp.IsObject())
    {
        if (DEBUG)
            cerr << "Failed to get field cmdRsp" << endl;
        return false;
    }

    const rapidjson::Value& code = cmd_rsp["errCode"];
    if (!code.IsInt())
    {
        if (DEBUG)
            cerr << "Failed to get field errCode" << endl;
        return false;
    }
    err_code = code.GetInt64();
    return true;
}

// 发送命令回复信息给地面站
void send_command_response_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                             const int value, const int status)
{
    public_pkg::cmd_rsp_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_NAME_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_GROUND_STATION;
    msg.header.dstNodeType = NODE_NAME_GROUND_STATION;
    msg.header.msgType = MSG_TYPE_CMD_RSP_MSG;
    msg.header.msgTime = ros::Time::now();

    
    //msg.object = CONTROL_OBJECT_GROUND_STATION;
    msg.object = CONTROL_OBJECT_MASTER_CONTROLLER;

    const string content = format("{\"cmdRspArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %d, \"statusCode\": %d}]}",
                                    cmd_id, "uint", value, status);
    msg.content = content;
    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content );
    //ROS_INFO_STREAM("send_command_response_to_ground_station: " << content);
}

// 发送遥信信息给地面站
void send_command_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                    const int value, const char* name)
{
    public_pkg::status_digital_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_NAME_CONTROLLER;
    msg.header.msgType = MSG_TYPE_STATUS_DIGITAL_MSG;
    
    const string content = format("{\"digitalArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %d, \"name\": \"%s\"}]}",
                                    cmd_id, "uint", value, name);
    msg.content = content;
    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content );
    //ROS_INFO_STREAM("Sent command content to ground station: " << content);
}

void send_info_to_ground_station(ros::Publisher& pub, const int cmd_id,
                                    std::string value, const char* name)
{
    public_pkg::status_digital_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_NAME_CONTROLLER;
    msg.header.msgType = MSG_TYPE_STATUS_DIGITAL_MSG;
    
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);
    writers.StartObject();
    writers.Key("digitalArray");
    writers.StartArray();

    writers.StartObject();

    writers.Key("seq");
    writers.Uint(cmd_id);
    writers.Key("type");
    writers.String("string");
    writers.Key("value");
    string tempstr =UnicodeToUTF8(s2ws(value));
    //string tempstr2 = UnicodeToUTF8(value);
    
    writers.String(tempstr.c_str());
    writers.Key("name");
    writers.String(name);

    writers.EndObject();

    writers.EndArray();
    writers.EndObject();
    msg.content=string(ss.GetString());


    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content);
    //ROS_INFO_STREAM("Sent command content to ground station: " << msg.content);
}

// 发送主控心跳信息
void send_heart_beat(ros::Publisher& pub)
{
    public_pkg::heart_beat_msg msg;
    msg.srcNodeName = NODE_NAME_CONTROLLER;
    msg.srcNodeType = NODE_NAME_CONTROLLER;
    msg.msgType = MSG_TYPE_HEART_BAET_MSG;
    msg.msgTime = ros::Time::now();
    msg.process_id = getpid();
    
    pub.publish(msg);
}

// 发送命令指令给接线工具
void send_command_to_clamp_wire_tool(ros::Publisher& pub, const int seq,
                                     const unsigned int value, const char* name)
{
    public_pkg::cmd_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_CLAMP_WIRE_TOOL;
    msg.header.dstNodeType = NODE_TYPE_CLAMP_WIRE_TOOL;
    msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    msg.object = CONTROL_OBJECT_CLAMP_WIRE_TOOL_CONNECT;

    const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
                                    seq, "uint", value, name);
    msg.content = content;
    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content );
    //ROS_INFO_STREAM("send_command_to_clamp_wire_tool: " << content);
}

// 发送命令指令给剪线工具
void send_command_to_cut_wire_tool(ros::Publisher& pub, const int seq,
                                   const unsigned int value, const char* name)
{
    public_pkg::cmd_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_CUT_WIRE_TOOL;
    msg.header.dstNodeType = NODE_TYPE_CUT_WIRE_TOOL;
    msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    msg.object = CONTROL_OBJECT_CLAMP_WIRE_TOOL_CUT;

    const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
                                    seq, "uint", value, name);
    msg.content = content;
    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content );
    //ROS_INFO_STREAM("send_command_to_cut_wire_tool: " << content);
}

// 发送命令指令给夹爪工具
void send_command_to_claw_tool(ros::Publisher& pub, const int seq,
                                  const unsigned int value, const char* name)
{
    public_pkg::cmd_msg msg;
    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
    msg.header.dstNodeName = NODE_NAME_CLAW_TOOL;
    msg.header.dstNodeType = NODE_TYPE_CLAW_TOOL;
    msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    msg.object = CONTROL_OBJECT_CLAMP_WIRE_TOOL_GRIPPER;

    const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
                                    seq, "uint", value, name);
    msg.content = content;
    pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content);
    //ROS_INFO_STREAM("send_command_to_claw_tool: " << content);
}

// 发送命令指令给剥线工具
void send_command_to_strip_wire_tool(ros::Publisher& pub, const int seq,
	const unsigned int value, const char* name)
{
	public_pkg::cmd_msg msg;
	msg.header.srcNodeName = NODE_NAME_CONTROLLER;
	msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
	msg.header.dstNodeName = NODE_NAME_STRIP_WIRE_TOOL;
	msg.header.dstNodeType = NODE_TYPE_STRIP_WIRE_TOOL;
	msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

	msg.object = CONTROL_OBJECT_CLAMP_WIRE_TOOL_STRIP;

	const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
		seq, "uint", value, name);
	msg.content = content;
	pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content );
    //ROS_INFO_STREAM("send_command_to_strip_wire_tool: " << content);
}



void send_command_to_robot_script(ros::Publisher& pub, const unsigned int CtrType,
	                             std::string strUrScript,const int arm_id)
{
    public_pkg::cmd_msg msg;
    static unsigned int CtrlNo = 1;
    
    msg.header.msgID = 1;
	msg.header.srcNodeName = NODE_NAME_CONTROLLER;
	msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
	msg.header.dstNodeName = NODE_NAME_ROBOT_Script;
	msg.header.dstNodeType = NODE_TYPE_ROBOT_Script;
	msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

    if(arm_id == ROBOT_ARM_LEFT_ARM)
    {
	    msg.object = CONTROL_OBJECT_LEFT_ARM;
    }
    else if(arm_id == ROBOT_ARM_RIGHT_ARM)
    {
        msg.object = CONTROL_OBJECT_RIGHT_ARM;
    } 

    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);
    writers.StartObject();
    writers.Key("UrScriptCtrl");
    writers.StartArray();

    writers.StartObject();

    writers.Key("CtrlType");
    writers.Uint(CtrType);
    writers.Key("CtrlNo");
    writers.Uint(CtrlNo);
    writers.Key("strUrScript");
    writers.String(strUrScript.c_str());

    writers.EndObject();

    writers.EndArray();
    writers.EndObject();
    msg.content=string(ss.GetString());
    
    if(CtrType == 2)
    {
        if(CtrlNo > 60000)
        CtrlNo = 0;
        ++CtrlNo;
    }
	pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, msg.content);
    //ROS_INFO_STREAM("send_command_to_robot_script: " << msg);
}

// 读取指定脚本并发送给机械臂执行
void exec_arm_script(ros::Publisher& pub, const std::string& script_path,
                        const unsigned int CtrType,const int arm_id)
{
    std::string strUrScript;
    strUrScript = read_file(script_path.c_str());
    if (strUrScript.empty())
    {
        cerr << "Failed to read file " << script_path << endl;
        return;
    }
    
    send_command_to_robot_script(pub,CtrType,strUrScript,arm_id);
}

//机械臂电源控制服务
bool call_power_control_srv(ros::ServiceClient& service_clients,
    const int command_id, const unsigned int arm_id, const std::string& control_command)
{
    //ros::Time call_time = ros::Time::now();
    //ROS_INFO_STREAM(" call_time: " <<  call_time << "  command_id: " << command_id 
              //<< " arm_id:" << arm_id << " control_command: " << control_command );

    public_pkg::arms_power_srv srv;

    srv.request.srvTime = ros::Time::now();
    srv.request.arm_id = arm_id;
    srv.request.control_command = control_command;
    if (!service_clients.call(srv))
    {
        ROS_ERROR("Failed to call service");
        return false;
    }

    const unsigned int control_result = srv.response.control_result;
    const string result = srv.response.result;
    /*//由于电源板物理开机需要一些时间，及时返回的执行状态不能作为执行结果
    if(control_result == 1)
    {
        ROS_INFO_STREAM("control [" << control_command << "] SUCCESS");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("control [" << control_command << "] FAILD]");
    }*/

    ROS_INFO_STREAM("Call_power_control_service ........");
    return false;
}


// 发送命令指令给电源管理结点
void send_command_to_power_manager(ros::Publisher& pub, const int seq,
	                               const unsigned int value, const char* name)
{
	public_pkg::cmd_msg msg;

    msg.header.msgID = 0;
	msg.header.srcNodeName = NODE_NAME_CONTROLLER;
	msg.header.srcNodeType = NODE_TYPE_CONTROLLER;
	msg.header.dstNodeName = NODE_NAME_POWER_MANAGER;
	msg.header.dstNodeType = NODE_TYPE_POWER_MANAGER;
	msg.header.msgType = MSG_TYPE_CMD_MSG;
    msg.header.msgTime = ros::Time::now();

	msg.object = CONTROL_OBJECT_POWER_MANAGER;
    msg.type = 0;
    
	const string content = format("{\"cmdArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %u, \"name\": \"%s\"}]}",
		seq, "uint", value, name);
	msg.content = content;
	pub.publish(msg);
    creat_logfile(LOG_MAIN_CONTROL, content);
    //ROS_INFO_STREAM("send_command_to_power_manager: " << content);
}

// 发送命令信息给地面站
/*void send_info_to_ground_station(ros::Publisher& pub, const int seq,
                                    const char* value, const char* name)
{
	public_pkg::status_digital_msg msg;


    msg.header.srcNodeName = NODE_NAME_CONTROLLER;
    msg.header.srcNodeType = NODE_NAME_CONTROLLER;
    msg.header.msgType = MSG_TYPE_STATUS_DIGITAL_MSG;
    std::wstring ws_value = s2ws(value);
    std::string s_value = ws2s(ws_value);
    string content = format("{\"digitalArray\": [{\"seq\": %d, \"type\": \"%s\", \"value\": %s, \"name\": \"%s\"}]}",
                                    seq, "std::string", value, name);
    msg.content = content;

    pub.publish(msg);

    ROS_INFO_STREAM("send_info_to_ground_station: " << content);
}*/



// string 转 utf-8
std::wstring s2ws(const std::string& str)
 {
  if (str.empty()) {
    return L"";
  }
  unsigned len = str.size() + 1;
  setlocale(LC_CTYPE, "en_US.UTF-8");
  std::unique_ptr<wchar_t[]> p(new wchar_t[len]);
  mbstowcs(p.get(), str.c_str(), len);
  std::wstring w_str(p.get());
  return w_str;
}
//  utf-8 转 string
std::string ws2s(const std::wstring& w_str)
{
    if (w_str.empty()) {
      return "";
    }
    unsigned len = w_str.size() * 4 + 1;
    setlocale(LC_CTYPE, "en_US.UTF-8");
    std::unique_ptr<char[]> p(new char[len]);
    wcstombs(p.get(), w_str.c_str(), len);
    std::string str(p.get());
    return str;
}

std::string UnicodeToUTF8(const std::wstring & wstr)
{
    std::string ret;
    try {
        std::wstring_convert< std::codecvt_utf8<wchar_t> > wcv;
        ret = wcv.to_bytes(wstr);
    } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
    }
    return ret;
}

// std::string string_To_UTF8(const std::string & str)
// {
// 	int nwLen = ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);
 
// 	wchar_t * pwBuf = new wchar_t[nwLen + 1];//一定要加1，不然会出现尾巴 
// 	ZeroMemory(pwBuf, nwLen * 2 + 2);
 
// 	::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.length(), pwBuf, nwLen);
 
// 	int nLen = ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, -1, NULL, NULL, NULL, NULL);
 
// 	char * pBuf = new char[nLen + 1];
// 	ZeroMemory(pBuf, nLen + 1);
 
// 	::WideCharToMultiByte(CP_UTF8, 0, pwBuf, nwLen, pBuf, nLen, NULL, NULL);
 
// 	std::string retStr(pBuf);
 
// 	delete[]pwBuf;
// 	delete[]pBuf;
 
// 	pwBuf = NULL;
// 	pBuf = NULL;
 
// 	return retStr;
// }

void creat_logfile(const unsigned int type, const string& logtxt)
{
    switch (type)
    {
    case LOG_DEBUG:
        write_content(BEGIN_DEBUG, FILE_DEBUG, MSG_BEGIN_DEBUG, logtxt);
        break;

    case LOG_INFO:
        write_content(BEGIN_INFO, FILE_INFO, MSG_BEGIN_INFO, logtxt);
        break;

    case LOG_WARN:
        write_content(BEGIN_WARN, FILE_WARN, MSG_BEGIN_WARN, logtxt);
        break;
    
    case LOG_ERROR:
        write_content(BEGIN_ERROR, FILE_ERROR, MSG_BEGIN_ERROR, logtxt);
        break;
    
    case LOG_FATAL:
        write_content(BEGIN_FATAL, FILE_FATAL, MSG_BEGIN_FATAL, logtxt);
        break;
    
    case LOG_REMOTE_CONTROL:
        write_content(BEGIN_REMOTE, FILE_REMOTE_CONTROL, MSG_BEGIN_INFO, logtxt);
        break;
    
    case LOG_MAIN_CONTROL:
        write_content(BEGIN_MAIN, FILE_MAIN_CONTROL, MSG_BEGIN_INFO, logtxt);
        break;
    

    default:
        break;
    }
}

void write_content(const char* begin_name, const char* filepath, const char* msg_begin , const string& log_content)
{
    ofstream infile;
    string filename = "";
    string ID = begin_name + format("%d", getpid());
    filename = filepath + ID;
    filename += END_NAME;
    string content = UnicodeToUTF8(s2ws(log_content)) ;
    mkdir(FP,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);//给文件权限
    mkdir(filepath,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    infile.open(filename, ios::app);
    infile <<  msg_begin
        << current_time(TM_YEAR) << "."
        << current_time(TM_MON) << "." 
        << current_time(TM_MDAY) << "-" 
        << current_time(TM_HOUR) << ":" 
        << current_time(TM_MIN) << ":" 
        << current_time(TM_SEC) << "]: " 
        << content << "\n" << endl;
    infile.close();

}