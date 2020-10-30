#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "public_pkg/cmd_rsp_msg.h"
#include "public_pkg/cmd_msg.h"
#include "public_pkg/status_digital_msg.h"
#include "public_pkg/status_analog_msg.h"
#include "public_pkg/heart_beat_msg.h"
#include "rapidjson/reader.h"
#include "rapidjson/document.h"

#include <string>
#include <vector>
#include <algorithm>
#include <codecvt>
#include <cstdlib>
#include <memory>
#include <csignal>
#include <thread>

using namespace std;

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

void current_time ()
{
    struct tm *local;
    time_t t; 
    t=time(NULL);
    local=localtime(&t);
    ROS_INFO_STREAM("Current Local Time: [" << local->tm_year + 1900 << "." << local->tm_mon + 1 << "." << local->tm_mday << "][" << local->tm_hour << ":" << local->tm_min << ":"<< local->tm_sec << "]"); 
}


void digital_call_back(const public_pkg::status_digital_msg& msg)
{
    // if (msg.header.srcNodeName == "MainControl")
    // {
    //     current_time();
    //     std::wstring ws = s2ws(msg.content);
    //     std::string s = ws2s(ws);
    //     ROS_INFO_STREAM( "main control sended digital(maincontrol/yx) msg: " << "\n" << msg.content << "\n");
    // }
}
//
void left_arm_call_back(const public_pkg::cmd_msg& msg)
{ 
        //current_time();
        //ROS_INFO_STREAM("main control sended left arm script msg: " << "\n" <<  msg.content << "\n");
}

void right_arm_call_back(const public_pkg::cmd_msg& msg)
{
        //current_time();
       // ROS_INFO_STREAM("main control sended right arm script msg: " << "\n" <<  msg.content << "\n");
}

void cmd_rsp_call_back(const public_pkg::cmd_rsp_msg& msg)
{
        //current_time();
        //ROS_INFO_STREAM("main control sended cmd **rsp** msg: " << "\n" <<  msg.content << "\n");
}

void cmd_call_back(const public_pkg::cmd_msg& msg)
{
        //current_time();
        //ROS_INFO_STREAM("main control sended cmd msg: " << "\n" <<  msg.content << "\n");
}

/*void heart_beat_call_back(const public_pkg::heart_beat_msg& msg)
{
    if (msg.header.srcNodeName == "MainControl")
    {
        current_time();
        ROS_INFO_STREAM("main control sended heart beat msg: " << "\n");// <<  msg<< "\n");
    }
}*/

void analog_call_back(const public_pkg::status_analog_msg& msg)
{
    if (msg.header.srcNodeName == "Slide")
    {
        current_time();
        std::wstring ws = s2ws(msg.content);
        std::string s = ws2s(ws);
        ROS_INFO_STREAM( "slide: " << "\n" << msg.content << "\n");
    }
}

// 按下Ctrl+C后调用
static void sigint_handler(int);

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "controller_check");

    ROS_INFO("============ main controller output check starting ============");

    ros::NodeHandle nh;

    ros::Subscriber analog_sub = nh.subscribe("/nari/szrd/dnrobot/yc", 1000, analog_call_back);
    ros::Subscriber digital_sub = nh.subscribe("/nari/szrd/dnrobot/maincontrol/yx", 1000, digital_call_back);      // 订阅信息
    ros::Subscriber left_arm_sub = nh.subscribe("/l_arm_controller/ur_driver/NariURScript", 1000, left_arm_call_back);     // 订阅命令消息
    ros::Subscriber right_arm_sub = nh.subscribe("/r_arm_controller/ur_driver/NariURScript", 1000, right_arm_call_back);    // 
    ros::Subscriber cmd_rsp_sub = nh.subscribe("/nari/szrd/dnrobot/cmdRsp", 1000, cmd_rsp_call_back); // 消息
    ros::Subscriber cmd_sub = nh.subscribe("/nari/szrd/dnrobot/cmd", 1000, cmd_call_back);   // 
    //ros::Subscriber heart_beat_sub = nh.subscribe("/nari/szrd/dnrobot/node/heartbeat", 1000, heart_beat_call_back);   
    // 注册SIGINT信号
    std::signal(SIGINT,  sigint_handler);

    ros::spin();

    return 0;
}

// 按下Ctrl+C后调用
static void sigint_handler(int)
{
    ROS_INFO("You have pressed Ctrl+C, so the controller output check will be stopped");
    // 调度器使用了rosconsole，延时一小会，确保退出信息能正常输出
    std::this_thread::sleep_for(chrono::milliseconds(100));

    ROS_INFO("============ main controller output check stopped ============");
    ros::shutdown();
}

