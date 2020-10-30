#include "config.h"
#include "SegmentScheduler.h"
#include "call_back.h"
#include "utils.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "public_pkg/cmd_rsp_msg.h"
#include "public_pkg/status_digital_msg.h"
#include "public_pkg/heart_beat_msg.h"
#include "public_pkg/write_srv.h"
#include "public_pkg/arms_power_srv.h"

#include <memory>
#include <csignal>
#include <thread>

using namespace std;

// 全局变量，在回调中使用
std::shared_ptr<SegmentScheduler> scheduler;

// 按下Ctrl+C后调用
static void sigint_handler(int);

int main(int argc, char* argv[])
{
#ifndef SEPERATE_TESTING
    ros::init(argc, argv, NODE_NAME_CONTROLLER);
#else
    ros::init(argc, argv, "controller_test");
#endif

    ROS_INFO("============ controller started ============");
    creat_logfile(LOG_INFO,"============ controller started ============");
    ros::NodeHandle nh;

    ros::Publisher left_arm_control_info_pub = nh.advertise<public_pkg::cmd_msg>(LEFT_ARM_CONTROL_TOPIC, 5);   // 左臂控制消息
    ros::Publisher right_arm_control_info_pub = nh.advertise<public_pkg::cmd_msg>(RIGHT_ARM_CONTROL_TOPIC, 5);  // 右臂控制消息
    ros::Publisher cmd_rsp_msg_pub = nh.advertise<public_pkg::cmd_rsp_msg>(COMMAND_RESPONSE_MESSAGE_TOPIC, 5);  // 命令回复消息
    ros::Publisher cmd_msg_pub = nh.advertise<public_pkg::cmd_msg>(COMMAND_MESSAGE_TOPIC, 5);  // 命令消息
  //ros::Publisher status_digital_msg_pub = nh.advertise<public_pkg::status_digital_msg>(GENERAL_DIGITAL_MESSAGE_TOPIC, 5); // 通用数字量消息
    ros::Publisher status_digital_msg_pub = nh.advertise<public_pkg::status_digital_msg>(GENERAL_DIGITAL_MAINCONTROL_MESSAGE_TOPIC, 5); // 给地面站的通用数字量消息
    ros::Publisher heart_beat_msg_pub = nh.advertise<public_pkg::heart_beat_msg>(HEART_BEAT_MESSAGE_TOPIC, 5); // 其他通用数字量消息

    // 自动选点相关话题
    ros::Publisher rangeparam_pub = nh.advertise<public_pkg::range_msg>(RANGEPARAM_TOPIC, 5);  // 右臂控制消息
    ros::Publisher ground_check_pub1 = nh.advertise<std_msgs::String>(GROUND_CHECK_TOPIC1, 5); // 其他通用数字量消息
    ros::Publisher ground_check_pub2 = nh.advertise<std_msgs::String>(GROUND_CHECK_TOPIC2, 5); // 其他通用数字量消息
    ros::Publisher ground_check_pub3 = nh.advertise<std_msgs::String>(GROUND_CHECK_TOPIC3, 5); // 其他通用数字量消息
    ros::Publisher ground_check_pub4 = nh.advertise<std_msgs::String>(GROUND_CHECK_TOPIC4, 5); // 其他通用数字量消息
    
    ros::ServiceClient write_service_client = nh.serviceClient<public_pkg::write_srv>(SLIDER_CONTROL_SERVICE_NAME); // 滑台写服务
    ros::ServiceClient arm_power_control_service_client = nh.serviceClient<public_pkg::arms_power_srv>(ARM_POWER_CONTROL_SERVICE_CLIENT); // 机械臂电源控制服务


    // 尽管sub用不上，还是需要声明，否则无法正常发布 (被编译器优化了？)
    ros::Subscriber cmd_msg_sub = nh.subscribe(COMMAND_MESSAGE_TOPIC, 1000, cmd_call_back);     // 订阅命令消息
    ros::Subscriber left_arm_io_state_sub = nh.subscribe(COMMAND_RESPONSE_LEFT_ARM_LISTEN_TOPIC, 1000, left_arm_io_state_call_back);    // 订阅左机械臂IO状态消息
    ros::Subscriber right_arm_io_state_sub = nh.subscribe(COMMAND_RESPONSE_RIGHT_ARM_LISTEN_TOPIC, 1000, right_arm_io_state_call_back); // 订阅右机械臂IO状态消息
    ros::Subscriber status_digital_sub = nh.subscribe(GENERAL_DIGITAL_MESSAGE_TOPIC, 1000, status_digital_call_back);   // 订阅遥信信息
    ros::Subscriber status_analog_sub = nh.subscribe(GENERAL_ANALOG_MESSAGE_TOPIC, 1000, status_analog_call_back);      // 订阅遥测信息
    ros::Subscriber remotecontrol_heartbeat_sub = nh.subscribe(REMOTECONTROL_HEARTBEAT_MESSAGE_TOPIC, 1000, remotecontrol_heartbeat_call_back);      // 订阅遥测信息
    
    //订阅lidar1消息对象 摄像头1
    ros::Subscriber status_lidar1_sub = nh.subscribe(LIDAR1_LISTEN_TOPIC, 1000, status_lidar1_call_back);
    //订阅lidar2消息对象 摄像头2
    ros::Subscriber status_lidar2_sub = nh.subscribe(LIDAR2_LISTEN_TOPIC, 1000, status_lidar2_call_back);
    //订阅lidar1消息对象 摄像头1b
    ros::Subscriber status_lidar1b_sub = nh.subscribe(LIDAR1B_LISTEN_TOPIC, 1000, status_lidar1b_call_back);
    //订阅lidar2消息对象 摄像头2b
    ros::Subscriber status_lidar2b_sub = nh.subscribe(LIDAR2B_LISTEN_TOPIC, 1000, status_lidar2b_call_back);

	// 订阅机械臂末端位姿数据
    ros::Subscriber left_arm_pos_sub = nh.subscribe(LEFT_ARM_TOOL_POINT_POSITION_TOPIC, 1000, left_arm_tool_position_call_back);    // 订阅左机械臂末端位姿数据消息
    ros::Subscriber right_arm_pos_sub = nh.subscribe(RIGHT_ARM_TOOL_POINT_POSITION_TOPIC, 1000, right_arm_tool_position_call_back); // 订阅右机械臂末端位姿数据消息

    ros::Rate loop_rate(1.0);

    // 设置指令函数表
    SegmentScheduler::SetFunctionMap();

    // 创建并启动调度器
    scheduler = std::make_shared<SegmentScheduler>(left_arm_control_info_pub, right_arm_control_info_pub, cmd_rsp_msg_pub, cmd_msg_pub,
                                                   status_digital_msg_pub, write_service_client,arm_power_control_service_client,
                                                   heart_beat_msg_pub,rangeparam_pub,ground_check_pub1,ground_check_pub2
                                                   ,ground_check_pub3,ground_check_pub4);

    // 注册SIGINT信号
    std::signal(SIGINT,  sigint_handler);

    scheduler->start();

    ros::spin();

    ros::shutdown();
    scheduler->requestToQuit();

    return 0;
}

// 按下Ctrl+C后调用
static void sigint_handler(int)
{
    ROS_INFO("You have pressed Ctrl+C, so the controller will be stopped");
    creat_logfile(LOG_INFO,"You have pressed Ctrl+C, so the controller will be stopped");
    if (scheduler)
    {
        // 请求调度器退出
        scheduler->requestToQuit();
        scheduler->notifyheartbeartEnd(true);
        auto& event = scheduler->remote_hearbeat_flag_;
        event.notify(false);
    }

    // 调度器使用了rosconsole，延时一小会，确保退出信息能正常输出
    std::this_thread::sleep_for(chrono::milliseconds(150));

    ROS_INFO("============ controller stopped ============");
    creat_logfile(LOG_INFO,"============ controller stopped ============");
    ros::shutdown();
}
