#ifndef SEGMENT_SCHEDULER_H
#define SEGMENT_SCHEDULER_h

#include <vector>
#include <stack>
#include <map>
#include <string>
#include <functional>
#include <mutex>
#include <memory>
#include <atomic>
#include <future>
#include <chrono>
#include <condition_variable>
#include <tuple>

#include "ros/ros.h"

#include "JobParser.h"
#include "cmd_line_func.h"
#include "Segment.h"
#include "CommandBlock.h"
#include "Event.h"
#include "config.h"

// 命令回复消息中的步态定义
enum CmdRspSegmentStatus
{
    CMD_RSP_SEGMENT_STATUS_NONE = 0,
    CMD_RSP_SEGMENT_STATUS_RUNNING,
    CMD_RSP_SEGMENT_STATUS_SUCCESS,
    CMD_RSP_SEGMENT_STATUS_STOP,
    CMD_RSP_SEGMENT_STATUS_ERROR,
};

// 命令调度结果定义，用来控制调度器行为
enum CmdScheduleResult
{
    CMD_SCHEDULE_RESULT_SUCCESS,                        // 调度成功
    CMD_SCHEDULE_RESULT_FAILED,                         // 调度失败
    CMD_SCHEDULE_RESULT_PLEASE_STOP,                    // 请求停止调度
    CMD_SCHEDULE_RESULT_PLEASE_QUIT,                    // 请求系统退出
    CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD,          // 请重新调度该命令
    CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_BLOCK,        // 请重新调度该块
    CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_SEGMENT,      // 请重新调度该段
};

class SegmentScheduler
{
public:
    SegmentScheduler(ros::Publisher& left_arm_control_pub,   ros::Publisher& right_arm_control_pub,
                     ros::Publisher& cmd_msg_pub,   ros::Publisher& cmd_rsp_msg_pub,
                     ros::Publisher& status_digital_msg_pub,   ros::ServiceClient& write_service_client,
                     ros::ServiceClient& arm_power_control_service_client,
                     ros::Publisher& heart_beat_msg_pub,
                     ros::Publisher& rangeparam_pub,ros::Publisher& ground_check_pub1
                     ,ros::Publisher& ground_check_pub2
                     ,ros::Publisher& ground_check_pub3
                     ,ros::Publisher& ground_check_pub4 );

    // 调度器状态定义
    enum Status
    {
        SCHEDULER_STATUS_NOT_STARTED,           // 调度器未启动
        SCHEDULER_STATUS_IDLE,                  // 空闲中
        SCHEDULER_STATUS_SCHEDULING,            // 调度指令中
        SCHEDULER_STATUS_PAUSED,                // 已暂停执行
    };

    // 调度请求定义
    enum ScheduleRequest
    {
        SCHEDULE_REQUEST_NONE,               // 无请求
        SCHEDULE_REQUEST_SKIP,               // 请求调步
        SCHEDULE_REQUEST_CONTINUE,           // 请求继续执行
        SCHEDULE_REQUEST_PAUSE,              // 请求暂停执行
        SCHEDULE_REQUEST_STOP,               // 请求停止执行
        SCHEDULE_REQUEST_QUIT,               // 请求退出系统
        SCHEDULE_REQUEST_RESCHEDULE_SEGMENT, // 请求重新执行段
        SCHEDULE_REQUEST_STEP_FORWARD,       // 请求步进前进
        SCHEDULE_REQUEST_STEP_BACKWARD,      // 请求步进后退
    };

    // 命令函数格式定义
    using CmdLineFunc = std::function<CmdLineFuncRsp(SegmentScheduler& scheduler, const std::string& line)>;

    // 绑定命令函数
    static void SetFunctionMap();

    // 手工操作作用域，RAII对象
    class ManuallyOperationScope
    {
    public:
        ManuallyOperationScope(SegmentScheduler& scheduler)
            : scheduler_(scheduler)
        {
            scheduler_.enableManuallyOperationMode(true);
        }

        ~ManuallyOperationScope()
        {
            scheduler_.enableManuallyOperationMode(false);
        }
    private:
        SegmentScheduler& scheduler_;
    };

    // 启动调度器
    void start();

    // 按照输入的文件和序号组调度段
    bool schedule(const std::string& path = "", const std::vector<int>& segment_idxs = std::vector<int>());
    
    // 通知服务到达
    void notifyServiceReach(const int result = 0)
    {
        auto& event = server_reach_flag_;
        if (event.isWaiting())
        {
            event.notify(result);
        }
    }
    
    void notifyPowerServiceQuitReach(const int result = 0)
    {
        //ROS_INFO("notifyPowerServiceQuitReach");
        //server_reach_flag_.notify(0);
        left_arm_protective_stop_flag_.notify(result);   //左臂保护性停止标志
        right_arm_protective_stop_flag_.notify(result);  //右臂保护性停止标志

        // 电源管理通信连接标志
        left_arm_connection_flag_.notify(0);                  //左臂建立连接标志
        left_arm_power_on_flag_.notify(0);                    //左臂已上电标志
        left_arm_enable_flag_.notify(0);                    //左臂已上电标志
        right_arm_enable_flag_.notify(0);                    //左臂已上电标志
        
        arm_emergency_stopped_flag_.notify(0);
        left_arm_program_running_flag_.notify(0);

        right_arm_connection_flag_.notify(0);                 //右臂建立连接标志
        right_arm_power_on_flag_.notify(0);                    //右臂已上电标志

        right_arm_program_running_flag_.notify(0);
    }

    // 释放所有等待中的信号
    void notifyAll(const bool result = false)
    {
        // 通知作业到达
        //notifySegmentReach();

        // 通知已手工确认
        notifyManuallyConfirm();

        // 通知激光雷达指令数据到达
        lidar_point_reach_flag_.notify(result);
        lidar_first_point_reach_flag_.notify(result);
        lidar_first_point_check_event.notify(result);
        lidar_second_point_check_event.notify(result);
        // 通知雷达选点完成
        lidar_point_confirm_flag_.notify(result);

        // 通知机械臂指令停止等待
        left_arm_reach_flag_.notify(result);
        right_arm_reach_flag_.notify(result);
		left_arm_pp_received_flag_.notify(result);
		right_arm_pp_received_flag_.notify(result);

		// 通知机械臂遥测数据到达
		arm_analog_info_received_flag_.notify(result);

        // 通知滑台指令停止等待
        vertical_slider_reach_flag_.notify(result);
        vertical_slider_go_home_flag_.notify(result);
        horizontal_slider_reach_flag_.notify(result);
        horizontal_slider_go_home_flag_.notify(result);
        slider_pos_confirm_flag_.notify(result);

        // 通知末端接线工具指令停止等待
        clamp_wire_tool_connect_tighten_result_flag_.notify(result);
        clamp_wire_tool_connect_tighten_init_flag_.notify(result);
        clamp_wire_tool_connect_clip_lock_flag_.notify(result);
        clamp_wire_tool_connect_clip_unlock_flag_.notify(result);
        clamp_wire_tool_connect_sleeve_lock_flag_.notify(result);
        clamp_wire_tool_connect_sleeve_unlock_flag_.notify(result);
        clamp_wire_tool_connect_branchline_lock_flag_.notify(result);
        clamp_wire_tool_connect_branchline_unlock_flag_.notify(result);

        // 通知末端剪线工具指令停止等待
        cut_wire_tool_cut_result_flag_.notify(result);
        cut_wire_tool_cut_init_flag_.notify(result);

        // 通知末端夹爪工具指令停止等待
        claw_tool_gripper_close_flag_.notify(result);
        claw_tool_gripper_open_flag_.notify(result);

        // 通知末端剥线工具指令停止等待
        strip_wire_tool_strip_onekey_result_flag_.notify(result);
        strip_wire_tool_strip_init_result_flag_.notify(result);
        strip_wire_tool_strip_rotate_result_flag_.notify(result);
        strip_wire_tool_strip_back_rotate_result_flag_.notify(result);
        strip_wire_tool_strip_upward_result_flag_.notify(result);
        strip_wire_tool_strip_link_left_result_flag_.notify(result);
        strip_wire_tool_strip_link_right_result_flag_.notify(result);
        strip_wire_tool_strip_link_up_result_flag_.notify(result);
        strip_wire_tool_strip_link_down_result_flag_.notify(result);
        strip_wire_tool_strip_stop_result_flag_.notify(result);

        // 通知末端工具连接状态
        clamp_wire_tool_connection_state_flag_.notify(result);
        cut_wire_tool_connection_state_flag_.notify(result);
        claw_tool_connection_state_flag_.notify(result);
        strip_wire_tool_connection_state_flag_.notify(result);

        // 电源管理服务通知到位信号
        server_reach_flag_.notify(0);

        // 机械臂状态
        left_arm_protective_stop_flag_.notify(result);   //左臂保护性停止标志
        right_arm_protective_stop_flag_.notify(result);  //右臂保护性停止标志

        // 电源管理通信连接标志
        left_arm_connection_flag_.notify(0);                  //左臂建立连接标志
        left_arm_power_on_flag_.notify(0);                    //左臂已上电标志
        left_arm_enable_flag_.notify(0);                    //左臂已上电标志
        right_arm_enable_flag_.notify(0);                    //左臂已上电标志
        
        arm_emergency_stopped_flag_.notify(0);
        left_arm_program_running_flag_.notify(0);

        right_arm_connection_flag_.notify(0);                 //右臂建立连接标志
        right_arm_power_on_flag_.notify(0);                    //右臂已上电标志

        right_arm_program_running_flag_.notify(0);

        wait_quit_system_flag_.notify(result);

        wait_yx_signal_arrive.notify(-100);
    }

    // ----------------
    // -- 调度器状态管理
    // ----------------
    inline Status getStatus() const { return status_; }
    inline bool isIdle() const { return status_ == SCHEDULER_STATUS_IDLE; }

    // ----------------
    // -- 调度请求管理
    // ----------------
    inline void resetRequest() { request_ = SCHEDULE_REQUEST_NONE; }

    // 发送继续调度请求
    inline void requestToContinue()
    {
        request_ = SCHEDULE_REQUEST_CONTINUE;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::None;
        retry_mode_ = RetryMode::None;
    }

    inline void requestToSkip()
    {
        request_ = SCHEDULE_REQUEST_SKIP;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::None;
        retry_mode_ = RetryMode::None;
    }

    // 发送重新调度段请求
    inline void requestToRescheduleSegment()
    {
        request_ = SCHEDULE_REQUEST_RESCHEDULE_SEGMENT;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::Backward;
        retry_mode_ = RetryMode::RetrySegment;
    }

    // 发送步进调度请求
    inline void requestToStepForward()
    {
        request_ = SCHEDULE_REQUEST_STEP_FORWARD;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::Forward;
        retry_mode_ = RetryMode::None;
    }

    // 发送步退调度请求
    inline void requestToStepBackward()
    {
        request_ = SCHEDULE_REQUEST_STEP_BACKWARD;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::Backward;
        retry_mode_ = RetryMode::None;
    }

    // 暂停、停止、退出请求可以直接打断系统执行流程
    // 发送暂停调度请求
    inline void requestToPause()
    {
        request_ = SCHEDULE_REQUEST_PAUSE;
        stopArmMotion();
        notifyAll();
    }

    // 发送停止调度请求
    inline void requestToStop()
    {
        request_ = SCHEDULE_REQUEST_STOP;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::None;
        retry_mode_ = RetryMode::None;
        stopArmMotion();
        notifyAll();
    }

    // 发送退出调度请求
    inline void requestToQuit()
    {
        request_ = SCHEDULE_REQUEST_QUIT;
        last_step_mode_ = step_mode_;
        step_mode_ = CmdStepMode::None;
        retry_mode_ = RetryMode::None;
        stopArmMotion();
        notifyAll();
        notifySegmentReach();

    }

// ----------------
// -- 心跳退出
// ----------------

    inline bool notifyheartbeartEnd(const bool result)
    {
        auto& event = heart_beat_flag_;
        // (event.isWaiting())
        //{
            event.notify(result);
        //}
    }

    inline bool waitheartbeartEnd()
    {
        auto& event = heart_beat_flag_;
        if ( event.wait_for(std::chrono::seconds(4)) ) return false;
        
        return true;
    }

    // 向机械臂发送停止运动指令
    void stopArmMotion();


    // 查询调度请求
    inline bool hasRequest() const { return request_ != SCHEDULE_REQUEST_NONE; }
    inline bool isRequestedToContinue() const { return request_ == SCHEDULE_REQUEST_CONTINUE; }
    inline bool isRequestedToRescheduleSegment() const { return request_ == SCHEDULE_REQUEST_RESCHEDULE_SEGMENT; }
    inline bool isRequestedToStepForward() const { return request_ == SCHEDULE_REQUEST_STEP_FORWARD; }
    inline bool isRequestedToStepBackward() const { return request_ == SCHEDULE_REQUEST_STEP_BACKWARD; }
    inline bool isRequestedToPause() const { return request_ == SCHEDULE_REQUEST_PAUSE; }
    inline bool isRequestedToStop() const { return request_ == SCHEDULE_REQUEST_STOP; }
    inline bool isRequestedToQuit() const { return request_ == SCHEDULE_REQUEST_QUIT; }

    // ----------------
    // -- 数据成员获取接口
    // ----------------
    inline VariableTable& getVariableTable() { return jp_.variable_table_; }
    inline const VariableTable& getVariableTable() const { return jp_.variable_table_; }
    inline ros::Publisher& getCmdRspMsgPub() { return cmd_rsp_msg_pub_; }
    inline ros::Publisher& getCmdMsgPub() { return cmd_msg_pub_; }
    inline ros::Publisher& getLeftArmControlPub() { return left_arm_control_pub_; }
    inline ros::Publisher& getRightArmControlPub() { return right_arm_control_pub_; }
    inline ros::ServiceClient& getWriteServiceClient() { return write_service_client_; }
    inline ros::Publisher& getStatusDigitalMsgPub() { return status_digital_msg_pub_; }
    inline ros::Publisher& getHeartBeatPub() { return heart_beat_msg_pub_; }
    inline ros::ServiceClient& getPowerControlServiceClient() { return arm_power_control_service_client_; }

    // 自动选点
    inline ros::Publisher& getRangeParam_Pub() { return rangeparam_pub_;  }
    inline ros::Publisher& getGround_Check_Pub() { return ground_check_pub1_; }
    inline ros::Publisher& getGround_Check_Pub1() { return ground_check_pub1_; }
    inline ros::Publisher& getGround_Check_Pub2() { return ground_check_pub2_; }
    inline ros::Publisher& getGround_Check_Pub3() { return ground_check_pub3_; }
    inline ros::Publisher& getGround_Check_Pub4() { return ground_check_pub4_; }


    // ----------------
    // -- 滑台位置获取/设置接口
    // ----------------
    inline double getHorizontalSliderPos() const { return horizontal_slider_pos_; }
    inline double getVerticalSliderPos() const { return vertical_slider_pos_; }
    inline void setHorizontalSliderPos(const double pos) { horizontal_slider_pos_ = pos; }
    inline void setVerticalSliderPos(const double pos) { vertical_slider_pos_ = pos; }
	// 机械臂位置信息
	inline DoubleValueArray getArm1PP() const { return arm1_pp_; }
	inline void setArm1PP(DoubleValueArray point) { arm1_pp_ = point; }
	inline DoubleValueArray getArm1Joint() const { return arm1_joint_; }
	inline void setArm1Joint(DoubleValueArray point) { arm1_joint_ = point; }

	inline DoubleValueArray getArm2PP() const { return arm2_pp_; }
	inline void setArm2PP(DoubleValueArray point) { arm2_pp_ = point; }
	inline DoubleValueArray getArm2Joint() const { return arm2_joint_; }
	inline void setArm2Joint(DoubleValueArray point) { arm2_joint_ = point; }

    // movel2_waitsingal指令
    /*
    inline std::string getSingalDevice() const { return SingalDevice_; }
    inline void setSingalDevice(std::string SingalDevice) { SingalDevice_ = SingalDevice; }
    inline int getSignalSeq() const { return SingalSeq_; }
    inline void setSignalSeq(int SingalSeq) { SingalSeq_ = SingalSeq; }
    inline int getSignalValue() const { return SingalValue_; }
    inline void setSignalValue(int SingalValue) { SingalValue_ = SingalValue; }*/

    // ----------------
    // -- 手工确认信号管理
    // ----------------
    // 通知已手工确认
    inline void notifyManuallyConfirm()
    {
        auto& event = manually_confirm_flag_;
        if (event.isWaiting())
        {
            event.notify(true);
        }
    }

    // 等待手工确认
    inline bool waitManuallyConfirm()
    {
        auto& event = manually_confirm_flag_;
        return event.wait_for(std::chrono::seconds(DEFAULT_MANUALLY_CONFIRM_RESPONSE_WAIT_TIME_SECONDS));
    }

    // 查询是否在等待手工确认
    inline bool isWaitingManuallyConfirm() const
    {
        auto& event = manually_confirm_flag_;
        return event.isWaiting();
    }

    // 检查是否处于手工操作模式
    bool isInManuallyOperationMode() const { return in_manually_operation_mode_; }

    // ----------------
    // -- 同步事件定义
    // ----------------
    // 激光雷达
    BoolEvent lidar_point_reach_flag_;          // 选点数据到达
    BoolEvent lidar_first_point_reach_flag_;    // 第一个点选点数据到达
    BoolEvent lidar_first_point_check_event;    // 第一个点选点数据到达
    BoolEvent lidar_second_point_check_event;    // 第一个点选点数据到达
    BoolEvent lidar_point_confirm_flag_;        // 选点数据到达

    // 机械臂
    BoolEvent left_arm_reach_flag_;             // 左机械臂到位信号
    BoolEvent right_arm_reach_flag_;            // 右机械臂到位信号
    BoolEvent arm_analog_info_received_flag_;   // 机械臂遥测数据到达
    BoolEvent left_arm_pp_received_flag_;       // 左机械臂末端位姿数据信号
    BoolEvent right_arm_pp_received_flag_;      // 右机械臂末端位姿数据信号

    // 滑台
    BoolEvent vertical_slider_reach_flag_;          // 垂直滑台到位标志
    BoolEvent vertical_slider_go_home_flag_;        // 垂直滑台归零标志
    BoolEvent horizontal_slider_reach_flag_;        // 水平滑台到位标志
    BoolEvent horizontal_slider_go_home_flag_;      // 水平滑台归零标志
    BoolEvent slider_pos_confirm_flag_;             // 滑台位置确认标志

    // 末端工具连接状态
    BoolEvent clamp_wire_tool_connection_state_flag_;   // 接线工具连接状态标志
    BoolEvent cut_wire_tool_connection_state_flag_;     // 剪线工具连接状态标志
    BoolEvent claw_tool_connection_state_flag_;         // 夹爪工具连接状态标志
    BoolEvent strip_wire_tool_connection_state_flag_;   // 剥线工具连接状态标志
    IntEvent wait_yx_signal_arrive;   // 剥线工具连接状态标志

    // 末端接线工具
    BoolEvent clamp_wire_tool_connect_tighten_result_flag_;     // 拧断结束标志    
    BoolEvent clamp_wire_tool_connect_tighten_init_flag_;       // 拧断电机复位标志    
    BoolEvent clamp_wire_tool_connect_clip_lock_flag_;          // 线夹锁定标志    
    BoolEvent clamp_wire_tool_connect_clip_unlock_flag_;        // 线夹解锁标志    
    BoolEvent clamp_wire_tool_connect_sleeve_lock_flag_;        // 套筒锁定标志    
    BoolEvent clamp_wire_tool_connect_sleeve_unlock_flag_;      // 套筒解锁标志    
    BoolEvent clamp_wire_tool_connect_branchline_lock_flag_;    // 支线锁定标志    
    BoolEvent clamp_wire_tool_connect_branchline_unlock_flag_;  // 支线解锁标志    

    // 末端剪线工具
    BoolEvent cut_wire_tool_cut_result_flag_;     // 剪线结束标志
    BoolEvent cut_wire_tool_cut_init_flag_;       // 剪线复位标志

    // 末端夹爪工具
    BoolEvent claw_tool_gripper_open_flag_;       // 夹爪复位标志
    BoolEvent claw_tool_gripper_close_flag_;      // 夹爪闭合标志

    // 末端剥线工具
    BoolEvent strip_wire_tool_strip_onekey_result_flag_;        // 一键剥皮结束标志位
    BoolEvent strip_wire_tool_strip_init_result_flag_;          // 初始化结束标志位
    BoolEvent strip_wire_tool_strip_rotate_result_flag_;        // 旋转结束标志位
    BoolEvent strip_wire_tool_strip_back_rotate_result_flag_;   // 反转结束标志位
    BoolEvent strip_wire_tool_strip_upward_result_flag_;        // 开口向上结束标志位
    BoolEvent strip_wire_tool_strip_link_left_result_flag_;       // 推杆向左结束标志位
    BoolEvent strip_wire_tool_strip_link_right_result_flag_;     // 推杆向右结束标志位
    BoolEvent strip_wire_tool_strip_link_up_result_flag_;       // 推杆向上结束标志位
    BoolEvent strip_wire_tool_strip_link_down_result_flag_;     // 推杆向下结束标志位
    BoolEvent strip_wire_tool_strip_stop_result_flag_;          // 停止结束标志位

    BoolEvent left_arm_protective_stop_flag_;   //左臂紧急停止标志
    BoolEvent right_arm_protective_stop_flag_;  //右臂紧急停止标志

    // 电源管理通信连接标志
    IntEvent left_arm_connection_flag_;                  //左臂建立连接标志
    IntEvent left_arm_power_on_flag_;                    //左臂已上电标志
    IntEvent arm_emergency_stopped_flag_;
    IntEvent left_arm_program_running_flag_;

    IntEvent right_arm_connection_flag_;                 //右臂建立连接标志
    IntEvent right_arm_power_on_flag_;                    //右臂已上电标志

    IntEvent right_arm_enable_flag_;                 
    IntEvent left_arm_enable_flag_;                    
    
    //IntEvent right_arm_emergency_stopped_flag_;
    IntEvent right_arm_program_running_flag_;
    BoolEvent wait_quit_system_flag_;
    BoolEvent heart_beat_flag_;

    IntEvent wait_continue_flag_;

    // 地面站心跳超时标志
    BoolEvent remote_hearbeat_flag_;

private:
    // ----------------
    // -- 作业到达信号管理接口
    // ----------------
    // 通知作业已到达
    inline void notifySegmentReach()
    {
        auto& event = segment_reach_flag_;
        if (event.isWaiting())
        {
            event.notify(true);
        }
    }

    // 等待作业到达
    inline bool waitSegmentReach()
    {
        auto& event = segment_reach_flag_;
        return event.wait_for(std::chrono::seconds(DEFAULT_SEGMENT_RESPONSE_WAIT_TIME_SECONDS));
    }


    // ----------------
    // -- 电源管理连接
    // ----------------

    // 左臂建立连接标志    
    inline int wait_left_arm_connection_SingleReach()
    {
        ROS_INFO("wait_left_arm_connection_SingleReach");
        auto& event = left_arm_connection_flag_;
        if (!event.wait_for(std::chrono::seconds(120)))
        { 
            ROS_ERROR("left_arm_not_connection!");
            send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-主臂连接超时，请人工检测后继续运行！- ","info"); 
            send_info_to_ground_station(getStatusDigitalMsgPub(),13,"主臂连接失败，请检查电源板故障","info");
                         
            return 0;
        }
        else if (event.get() == 0)
        {
            return 0;
            ROS_INFO("left_arm_connection_flag_ = 0");
        }
        //event.wait();
        return event.get();
    }
    // 左臂已上电标志
    inline int wait_left_arm_power_on_SingleReach(int waittime)
    {
        auto& event = left_arm_power_on_flag_;//1
        if (!event.wait_for(std::chrono::seconds(waittime)))
        { 
            ROS_ERROR("left_arm_not_power_on!");
            //send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-主臂上电超时！- ","info");  
            return WAIT_POWER_ON_TIME_OUT; 
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        //event.wait();
        return event.get();
    }

    inline int wait_left_arm_enable_SingleReach(int waittime)
    {
        auto& event = left_arm_enable_flag_;//1
        if (!event.wait_for(std::chrono::seconds(waittime)))
        { 
            return 1; 
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        return event.get();
    }

    inline int wait_arm_emergency_stopped_SingleReach(int waittime)
    {
        auto& event = arm_emergency_stopped_flag_;
        if (!event.wait_for(std::chrono::seconds(waittime)))
        { 
            return 1; 
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        return event.get();
    }

    inline int wait_left_arm_program_running_SingleReach()
    {
        auto& event = left_arm_program_running_flag_;
        event.wait();
        if (event.get() == 0)
        {
            return 0;
        }
        
        return event.get();
    }



    // 臂建立连接标志    
    inline int wait_right_arm_connection_SingleReach()
    {
        auto& event = right_arm_connection_flag_;
        if (!event.wait_for(std::chrono::seconds(20)))
        { 
            ROS_ERROR("right_arm_not_connection!");
            send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-从臂连接超时，请人工检测后继续运行！-","info"); 
            send_info_to_ground_station(getStatusDigitalMsgPub(),13,"从臂连接失败，请检查电源板故障","info");
             
            return 0;
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        //event.wait();
        return event.get();
    }

    inline int wait_right_arm_enable_SingleReach(int waittime)
    {
        auto& event = right_arm_enable_flag_;//1
        if (!event.wait_for(std::chrono::seconds(waittime)))
        { 
            //ROS_ERROR("left_arm_not_power_on!");
            //send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-主臂上电超时！- ","info");  
            return 1; 
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        //event.wait();
        return event.get();
    }
    // 右臂已上电标志
    inline int wait_right_arm_power_on_SingleReach(int waittime)
    {
        auto& event = right_arm_power_on_flag_;//101
        if (!event.wait_for(std::chrono::seconds(waittime)))
        { 
            ROS_ERROR("right_arm_not_power_on!");
            //send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-从臂上电超时！-","info");  
            return WAIT_POWER_ON_TIME_OUT; 
        }
        else if (event.get() == 0)
        {
            return 0;
        }
        //event.wait();
        return event.get();
    }

    

    inline int wait_right_arm_program_running_SingleReach()
    {
        auto& event = right_arm_program_running_flag_; 
        if (!event.wait_for(std::chrono::seconds(30)))
        { 
            ROS_ERROR("right_arm_not_connection!");
            send_info_to_ground_station(getStatusDigitalMsgPub(),15,"警告-从臂运行超时，请人工检测后继续运行！-","info");  
            return 0;
        }
        else if (event.get() == 0)
        {
            return 0;
        }
       // event.wait();
        return event.get();
    }


    // ----------------
    // -- 服务到达信号管理接口
    // ----------------

    // 等待服务到达left_arm_protective_stop_flag_
    inline int waitServiceReach()
    {
        auto& event = server_reach_flag_;
        event.wait();
        return event.get();
    }

    // ----------------
    // -- 执行功能函数定义
    // ----------------
    void run(); // 运行调度器
    void arm_power_run();
    void heart_beat();
    void remote_heart_beat();

    CmdScheduleResult runCommandLine(const CommandLine& line,       // 运行命令行
                                     const bool in_parallel = false,
                                     const int thread_id = 0);
    CmdScheduleResult runSequentailBlock(const CommandBlock& block,      // 运行顺序命令块
                                         const bool in_parallel = false,
                                         const int thread_id = 0);
    CmdScheduleResult runParallelBlock(const CommandBlocks& blocks);     // 运行并行命令块
    CmdScheduleResult runCalculationBlock(const CommandBlock& block);    // 运行计算命令块

    // 发送步态信息
    inline void sendCurrentSegmentStatus(const CmdRspSegmentStatus status);

    // ----------------
    // -- 手工模式接口
    // ----------------
    // 发送允许手工操作信号
    void sendManuallyOperationSignal(const int value);

    // 设置手工模式
    inline void enableManuallyOperationMode(const bool enable)
    {
        in_manually_operation_mode_ = enable;
        sendManuallyOperationSignal(enable ? 1 : 0);
    }

    

    // 命令步进模式定义
    enum class CmdStepMode
    {
        None,       // 不处于步进模式
        Forward,    // 处于步进前进模式
        Backward,   // 处于步进后退模式
    };

    // 重试模式定义
    enum class RetryMode
    {
        None,               // 不处于重试模式
        RetrySegment,       // 重试当前段
    };

    // ----------------
    // -- 并行模式线程同步接口
    // ----------------
    // 设置需要同步调度请求的并行线程数
    inline void setRequestWaiterNum(const unsigned short num)
    {
        num_parallel_threads_ = num;
        num_waiting_parallel_threads_ = 0;
    }

    // 同步所有并行线程的调度请求
    ScheduleRequest synRequest();

    // 同步所有并行线程的执行结果
    CmdLineFuncRsp synParallelCommandResults(const CmdLineFuncRsp rsp);

    // ----------------
    // -- 值命令行变量值缓存接口
    // ----------------
    // 从命令行变量值缓存表中加载该命令行的变量值
    void loadCachedVariables(const CommandLine& cmd_line);

    // 缓存命令行使用到的变量值
    void cacheVariables(const CommandLine& cmd_line);

private:
    JobParser jp_; // Job文件解析器

    // 调度段定义
    Segments scheduled_segments_;   // 根据请求调度的段组

    // 内部同步信号
    BoolEvent segment_reach_flag_;       // 作业到达标志
    BoolEvent manually_confirm_flag_;    // 手工确认标志
    IntEvent server_reach_flag_;         // 电源管理命令到达标志
    
    std::mutex scheduled_segment_mutex_; // 防止并发修改调度段表

    // 信息发布话题及服务
    ros::Publisher& cmd_msg_pub_;
    ros::Publisher& cmd_rsp_msg_pub_;
    ros::Publisher& left_arm_control_pub_;
    ros::Publisher& right_arm_control_pub_;
    ros::ServiceClient& write_service_client_;
    ros::Publisher& status_digital_msg_pub_;
    ros::Publisher& heart_beat_msg_pub_;
    ros::ServiceClient& arm_power_control_service_client_;

    // 自动选点
    ros::Publisher& rangeparam_pub_;
    ros::Publisher& ground_check_pub1_;
    ros::Publisher& ground_check_pub2_;  
    ros::Publisher& ground_check_pub3_;
    ros::Publisher& ground_check_pub4_;   
    
    // 调度器状态管理
    std::atomic<Status> status_ { SCHEDULER_STATUS_NOT_STARTED };
    std::atomic<ScheduleRequest> request_ { SCHEDULE_REQUEST_NONE };
    std::atomic<CmdStepMode> step_mode_ { CmdStepMode::None };
    CmdStepMode last_step_mode_ { CmdStepMode::None };
    RetryMode retry_mode_ { RetryMode::None };

    // 并行主线程ID号,用于设置并行模式下响应手工确认的代表线程
    int parallel_master_thread_id_ { 0 };

    // 数据成员
    std::atomic<double> horizontal_slider_pos_ { 0.0 };
    std::atomic<double> vertical_slider_pos_ { 0.0 };
	DoubleValueArray arm1_pp_ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	DoubleValueArray arm1_joint_ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	DoubleValueArray arm2_pp_ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	DoubleValueArray arm2_joint_ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    // 
    std::string SingalDevice_ {""};
    int SingalSeq_ { 0 };
    int SingalValue_ { 0 };

    // 当前调度段号
    unsigned int current_scheduling_segment_no_ { 0 };

    // 当前机械臂指令Ctrlno
    unsigned int current_robot_arm_cmd_pkg_seq { 0 };
    

    // 标志系统是否处于手工操作模式
    std::atomic_bool in_manually_operation_mode_ { false };

    // ----------------
    // -- 用于同步并行模式下的工作线程
    // ----------------
    std::mutex parallel_threads_mutex_;
    std::condition_variable parallel_threads_cv_;
    std::atomic_ushort num_parallel_threads_ { 1 };
    std::atomic_ushort num_waiting_parallel_threads_ { 0 };
    bool parallel_threads_all_ready_ { false };
    bool parallel_request_threads_all_ready_ { false };

    // 同步并行指令的执行结果
    std::atomic_bool parallel_commands_all_ok_ { true };

    // 同步手工确认请求
    ScheduleRequest parallel_request_ { SCHEDULE_REQUEST_NONE };

    // 设置函数表
    static std::map<JobLineType, CmdLineFunc> func_map_; // 指令函数表

    // 指令变量值缓存表，步进后退模式时使用
    // 将指令使用到的变量转为变量赋值语句缓存起来
    std::map<CommandLine, CommandLines> variable_assignment_cache;
};

#endif // SEGMENT_SCHEDULER_H
