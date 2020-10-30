#include "SegmentScheduler.h"
#include "cmd_line_func.h"
#include "utils.h"
#include "JobParser.h"
#include "config.h"

#include "ros/console.h"
#include "std_msgs/String.h"

#include <iostream>
#include <thread>
#include <algorithm>
#include <chrono>
#include <map>

using namespace std;

extern std::shared_ptr<SegmentScheduler> scheduler;


// 指令函数表
std::map<JobLineType, SegmentScheduler::CmdLineFunc> SegmentScheduler::func_map_;

SegmentScheduler::SegmentScheduler(ros::Publisher& left_arm_control_pub,   ros::Publisher& right_arm_control_pub,
                                   ros::Publisher& cmd_rsp_msg_pub,        ros::Publisher& cmd_msg_pub,
                                   ros::Publisher& status_digital_msg_pub, ros::ServiceClient& write_service_client,
                                   ros::ServiceClient& arm_power_control_service_client,
                                   ros::Publisher& heart_beat_msg_pub,
                                   ros::Publisher& rangeparam_pub, ros::Publisher& ground_check_pub1,
                                   ros::Publisher& ground_check_pub2,
                                   ros::Publisher& ground_check_pub3,
                                   ros::Publisher& ground_check_pub4 )
    :   left_arm_control_pub_(left_arm_control_pub),     right_arm_control_pub_(right_arm_control_pub),
        cmd_rsp_msg_pub_(cmd_rsp_msg_pub),               cmd_msg_pub_(cmd_msg_pub),
        status_digital_msg_pub_(status_digital_msg_pub), write_service_client_(write_service_client),
        arm_power_control_service_client_(arm_power_control_service_client),heart_beat_msg_pub_(heart_beat_msg_pub),
        rangeparam_pub_(rangeparam_pub),ground_check_pub1_(ground_check_pub1),ground_check_pub2_(ground_check_pub2)
        ,ground_check_pub3_(ground_check_pub3),ground_check_pub4_(ground_check_pub4)
{

}

// 绑定命令函数
void SegmentScheduler::SetFunctionMap()
{
    // 机械臂命令函数
    func_map_[JOB_LINE_TYPE_MOVEJ] = movej_line_func;
    func_map_[JOB_LINE_TYPE_MOVEJ_1] = movej1_line_func;
    func_map_[JOB_LINE_TYPE_MOVEJ_2] = movej2_line_func;
    func_map_[JOB_LINE_TYPE_MOVEJ_3] = movej3_line_func;

    func_map_[JOB_LINE_TYPE_MOVEL] = movel_line_func;
    func_map_[JOB_LINE_TYPE_MOVEL_1] = movel1_line_func;
    func_map_[JOB_LINE_TYPE_MOVEL_2] = movel2_line_func;
    func_map_[JOB_LINE_TYPE_MOVEL_2_WAITSIGNAL] = movel2_waitsinal_line_func;
    func_map_[JOB_LINE_TYPE_GETMAPOS] = get_ma_pos_func;

    // 滑台命令函数
    func_map_[JOB_LINE_TYPE_SLIDER_MOVE] = slider_move_line_func;
    func_map_[JOB_LINE_TYPE_SLIDER_GOHOME] = slider_go_home_line_func;
    func_map_[JOB_LINE_TYPE_RECORD_SLIDER_POS] = slider_record_pos_line_func;

    // 杂项指令
    func_map_[JOB_LINE_TYPE_WAITTIME] = waittime_line_func;
    func_map_[JOB_LINE_TYPE_MANNUALLY_CONFIRM] = manually_confirm_line_func;
    func_map_[JOB_LINE_TYPE_TOOLRESET] = tool_reset_line_func;

    // 变量操作指令
    func_map_[JOB_LINE_TYPE_VAR_OPERATOR_ASSIGNMENT] = var_operator_assignment_line_func;
    func_map_[JOB_LINE_TYPE_OFFSET] = offset_line_func;

    // 激光雷达指令
    func_map_[JOB_LINE_TYPE_LIDAR_1] = makepoint_lidar1_func;
    func_map_[JOB_LINE_TYPE_LIDAR_2] = makepoint_lidar2_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA] = makepoint_lidartoma_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA_1] = makepoint_lidartoma1_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA_2] = makepoint_lidartoma2_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA_3] = makepoint_lidartoma3_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA1_AT] = makepoint_lidartoma1_AT_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA2_AT] = makepoint_lidartoma2_AT_func;
    func_map_[JOB_LINE_TYPE_LIDARTOMA3_AT] = makepoint_lidartoma3_AT_func;

    // 末端接线工具指令
    func_map_[JOB_LINE_TYPE_CONNECT_TIGHTENSTART] = connect_tightenstart_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_TIGHTENSTOP] = connect_tightenstop_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_TIGHTENBACK] = connect_tightenback_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_TIGHTENINIT] = connect_tighteninit_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_CLIPLOCK] = connect_clip_lock_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_CLIPUNLOCK] = connect_clipunlock_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_SLEEVELOCK] = connect_sleevelock_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_SLEEVEUNLOCK] = connect_sleeveunlock_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_SLEEVESTOP] = connect_sleevestop_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_BRANCHLINELOCK] = connect_branchlinelock_line_func;
    func_map_[JOB_LINE_TYPE_CONNECT_BRANCHLINEUNLOCK] = connect_branchlineunlock_line_func;

    // 末端剪线工具指令
    func_map_[JOB_LINE_TYPE_CUT_START] = cut_start_line_func;
    func_map_[JOB_LINE_TYPE_CUT_INIT] = cut_init_line_func;

    // 末端夹爪工具指令
    func_map_[JOB_LINE_TYPE_OPENGRIPPER] = open_gripper_line_func;
    func_map_[JOB_LINE_TYPE_CLOSEGRIPPER] = close_gripper_line_func;

	// 末端剥线工具指令
	func_map_[JOB_LINE_TYPE_STRIP_ONEKEY] = strip_onekey_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_INIT] = strip_init_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_ROTATE] = strip_rotate_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_BACKROTATE] = strip_backrotate_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_UPWARD] = strip_upward_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_LINKLEFT] = strip_linkleft_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_LINKRIGHT] = strip_linkright_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_LINKUP] = strip_linkup_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_LINKDOWN] = strip_linkdown_line_func;
	func_map_[JOB_LINE_TYPE_STRIP_STOP] = strip_stop_line_func;

     // 运算指令
    func_map_[JOB_LINE_TYPE_ADD] = operator_var;
    func_map_[JOB_LINE_TYPE_SUB] = operator_var;
    func_map_[JOB_LINE_TYPE_MUL] = operator_var;
    func_map_[JOB_LINE_TYPE_DIV] = operator_var;

    // 空指令，直接返回成功
	func_map_[JOB_LINE_TYPE_NO_OP] = [] (SegmentScheduler&, const std::string&) {
        return CMD_LINE_FUNC_RSP_SUCCESS;
    };
}

// 启动调度器
void SegmentScheduler::start()
{
    if (status_ == SCHEDULER_STATUS_NOT_STARTED)
    {
        status_ = SCHEDULER_STATUS_IDLE;

        // 调度器启动后会发布话题数据，需要延时等ROS先启动
        ROS_INFO("Starting segment scheduler...");
        std::this_thread::sleep_for(chrono::seconds(1));

        thread t(&SegmentScheduler::run, this);
        // thread arm_power_t(&SegmentScheduler::arm_power_run, this);
        // thread heart_beat_t(&SegmentScheduler::heart_beat, this);
        // thread remote_heart_beat_t(&SegmentScheduler::remote_heart_beat, this);
        
        t.detach();  // 分离子线程
        // arm_power_t.detach();
        // heart_beat_t.detach();
        // remote_heart_beat_t.detach();
    }
}

// 发送允许手工操作信号
void SegmentScheduler::sendManuallyOperationSignal(const int value)
{
    // 发送允许手工操作信号给地面站
    send_command_to_ground_station(getStatusDigitalMsgPub(), CONTROL_COMMAND_ID_MANUALLY_CONTROL, value);
}

// 按照输入的序号组调度段
// TODO: 细化错误类型
bool SegmentScheduler::schedule(const std::string& path, const std::vector<int>& segment_idxs)
{
    if (SCHEDULER_STATUS_NOT_STARTED == status_) { start(); }

    if (path.empty())
    {
        ROS_ERROR("job path empty");
        send_info_to_ground_station(getStatusDigitalMsgPub(),16,"错误-作业文件路径不存在！","info");
        return false;
    }

    // 保存上次作业
    JobParser last_job = std::move(jp_);

    // 读取并解析作业文件
    JobParser new_job = JobParser(path);
    if (new_job.job_status_ != JobParser::JOB_STATUS_OK)
    {
        ROS_ERROR("Failed to parse job file: %s", path.c_str());
        send_info_to_ground_station(getStatusDigitalMsgPub(),16,"错误-作业文件解析失败！","info");
        return false;
    }
    ROS_INFO_STREAM("JobParser: [OK]");
    ROS_DEBUG_STREAM(new_job);

    jp_ = std::move(new_job);

    // 如果当前作业名和上次作业名相同，则继承上次作业的变量值
    if (last_job.job_path_ == path)
    {
        VariableTable& current_variable_table = jp_.variable_table_;

        // 不能直接move，因为作业文件可能临时被修改
        for (auto& p : last_job.variable_table_)
        {
            if (current_variable_table.find(p.first) != current_variable_table.cend())
            {
                // 如果继承的变量与当前作业同名变量的类型不同，则不继承
                const VariablePointer vp = current_variable_table.at(p.first);
                if (vp->getTypeName() != p.second->getTypeName())
                {
                    continue;
                }
            }
            current_variable_table[p.first] = p.second;
        }
        ROS_INFO_STREAM("Reloaded last variable table:\n" << jp_.variable_table_);
    }

    if (segment_idxs.empty()) // 按照默认顺序调度
    {
        {
            std::lock_guard<std::mutex> lk(scheduled_segment_mutex_);
            scheduled_segments_ = jp_.segments_;
        }
        ROS_INFO_STREAM("Segments to schedule:\n" << scheduled_segments_);

        notifySegmentReach();
        return true;
    }

    // 检查idxs是否合法
    bool segment_idx_invalid = any_of(segment_idxs.cbegin(), segment_idxs.cend(), 
        [&] (const int idx) { 
            return idx < 1 || idx > jp_.segments_.size(); // 注意段号是从1开始的
        });
    if (segment_idx_invalid)
    {
        send_info_to_ground_station(getStatusDigitalMsgPub(),14,"作业请求失败，需要工作步序超过范围","info");
    
        ROS_ERROR("Job begin request invalid, segment index out of range!");
        return false;
    }

    // 将请求作业存入调度器
    {
        std::lock_guard<std::mutex> lk(scheduled_segment_mutex_);

        // scheduled_segments_.clear();
        for_each(segment_idxs.cbegin(), segment_idxs.cend(), 
            [&](const int idx) {
                scheduled_segments_.push_back(jp_.segments_[idx - 1]); // 注意段号是从1开始的
            });

        ROS_INFO_STREAM("Segments to schedule:\n" << scheduled_segments_);
    }

    notifySegmentReach();
    return true;
}

// 运行调度器
void SegmentScheduler::run()
{
    ROS_INFO("Segment scheduler thread started");
    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info"); 
            

    while (true)
    {
        // 清空上次的调度请求
        resetRequest();

        status_ = SCHEDULER_STATUS_IDLE;

        // 等待作业时允许手工
        enableManuallyOperationMode(true);
        
        // 阻塞等待调度
        ROS_INFO("Waiting for segment to arrive");  
        //creat_logfile(LOG_INFO, "Waiting for segment to arrive");
        //send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");

        waitSegmentReach();

        // 在等待作业到来的过程中，可能有人请求我们退出
        if (isRequestedToQuit())
        {
            ROS_INFO("System quiting");
            resetRequest();
            break;
        }
        
        Segments segments_to_run_; // 需要执行的段
        {
            std::lock_guard<std::mutex> lk(scheduled_segment_mutex_);
            if (scheduled_segments_.empty())
            {
                continue;
            }

            // 数据已到来，交换缓冲区
            swap(scheduled_segments_, segments_to_run_);
        }

        // //判断是否机械臂在正常范围
        // DoubleValueArray jointlarm(6);
        // DoubleValueArray jointrarm(6);
        // get_Joint_value(*this,1,jointlarm);
        // get_Joint_value(*this,2,jointrarm);
        // ROS_INFO_STREAM("JOINT1: "<< jointlarm);
        // ROS_INFO_STREAM("JOINT2: "<< jointrarm);
        // if ((jointlarm[0]> (1.628 + 5*M_PI/180))||(jointlarm[0]<(1.628 - 5*M_PI/180))
        //     ||(jointlarm[1]> (0.055 + 5*M_PI/180))||(jointlarm[1]<(0.055 - 5*M_PI/180))
        //     ||(jointlarm[2]> (-2.831 + 5*M_PI/180))||(jointlarm[2]<(-2.831 - 5*M_PI/180))
        //     ||(jointlarm[3]> (-1.888 + 5*M_PI/180))||(jointlarm[3]<(-1.888 - 5*M_PI/180))
        //     ||(jointlarm[4]> (-1.548 + 5*M_PI/180))||(jointlarm[4]<(-1.548 - 5*M_PI/180))
        //     ||(jointlarm[5]> (-0.168 + 5*M_PI/180))||(jointlarm[5]<(-0.168 - 5*M_PI/180))
        //     ||(jointrarm[0]> (-1.57 + 5*M_PI/180))||(jointrarm[0]<(-1.57 - 5*M_PI/180))
        //     ||(jointrarm[1]> (0 + 5*M_PI/180))||(jointrarm[1]<(0 - 5*M_PI/180))
        //     ||(jointrarm[2]> (-2.875 + 5*M_PI/180))||(jointrarm[2]<(-2.875 - 5*M_PI/180))
        //     ||(jointrarm[3]> (-1.853 + 5*M_PI/180))||(jointrarm[3]<(-1.853 - 5*M_PI/180))
        //     ||(jointrarm[4]> (-1.566 + 5*M_PI/180))||(jointrarm[4]<(-1.566 - 5*M_PI/180))
        //     ||(jointrarm[5]> (-0.729 + 5*M_PI/180))||(jointrarm[5]<(-0.729 - 5*M_PI/180)))
        // {
        //     send_info_to_ground_station(getStatusDigitalMsgPub(),15,IF_CONTINUE_JOB,"WARNNING");
        //     //读取地面站继续信号 
        //     auto& event = wait_continue_flag_;
        //     if(!event.wait_for(chrono::seconds(20)))
        //     {
        //         send_info_to_ground_station(getStatusDigitalMsgPub(),14,"不在安全位置，等待继续作业超时，停止作业!!!","info");
        //         send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");
                     
        //         continue;
        //     }
        //     else if(event.get() == 0)
        //     {
        //         continue;
        //     }
        // }
 
        // 调度时禁止手工操作
        enableManuallyOperationMode(false);

        status_ = SCHEDULER_STATUS_SCHEDULING;
        step_mode_ = CmdStepMode::None;

        // 检查是否请求退出系统
        bool request_to_quit_system = false;

        // 逐段执行
        for (size_t segment_index = 0; segment_index < segments_to_run_.size() && segment_index >= 0;)
        {
            const Segment& segment = segments_to_run_[segment_index];
            current_scheduling_segment_no_ = segment.no_;

            // 发送步态信息
            sendCurrentSegmentStatus(CMD_RSP_SEGMENT_STATUS_RUNNING);
            ROS_INFO_STREAM("Running segment " << segment.no_ << " " << segment.name_);

            // 逐块执行
            CmdScheduleResult schedule_result = CMD_SCHEDULE_RESULT_SUCCESS;
            size_t block_index = step_mode_ == CmdStepMode::Backward ? segment.blocks_.size()-1 : 0;
            while (block_index < segment.blocks_.size() && block_index >= 0)
            {
                const CommandBlock& block = segment.blocks_[block_index];
                const CommandBlocks& parallel_blocks = segment.parallel_blocks_;

                if (block.empty()) continue;

                switch (block.getType())
                {
                case CommandBlock::COMMAND_BLOCK_TYPE_SEQUENTIAL:
                    ROS_INFO_STREAM("Running sequentail block:\n" << block);
                    schedule_result = runSequentailBlock(block);   
                    break;

                case CommandBlock::COMMAND_BLOCK_TYPE_PARALLEL:
                    ROS_INFO_STREAM("Running parallel block:\n" << parallel_blocks);
                    schedule_result = runParallelBlock(parallel_blocks);   
                    break;

                case CommandBlock::COMMAND_BLOCK_TYPE_CALCULATION:
                    ROS_INFO_STREAM("Running calculation block:\n" << block);
                    schedule_result = runCalculationBlock(block);   
                    break;

                default:
                    ROS_ERROR_STREAM("Unknown block type: " << to_string(block.getType()));
                    break;
                }

                bool need_break = false;//?
                switch (schedule_result)
                {
                case CMD_SCHEDULE_RESULT_SUCCESS:
                    if (retry_mode_ == RetryMode::None)
                        ROS_INFO("Run block: [SUCCESS]");
                    else if (retry_mode_ == RetryMode::RetrySegment)
                        ROS_INFO("Run block: [RESCHEDULE SEGMENT]");
                    block_index += step_mode_ == CmdStepMode::Backward ? -1 : 1;
                    break;

                case CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_BLOCK:
                    ROS_INFO("Run block: [RESCHEDULE]");
                    break;

                default:
                    need_break = true;
                    break;
                }
                if (need_break) break;
            }

            // 如果处于重置模式，那设置调度结果为”重新调度当前段“
            if (retry_mode_ == RetryMode::RetrySegment)
            {
                schedule_result = CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_SEGMENT;
            }

            // 检查段执行结果
            switch (schedule_result)
            {
            // 执行下一个段
            case CMD_SCHEDULE_RESULT_SUCCESS:
                ROS_INFO("Run segment: [SUCCESS]");
                (CMD_RSP_SEGMENT_STATUS_SUCCESS);
                segment_index += step_mode_ == CmdStepMode::Backward ? -1 : 1;
                break;

            // 重新执行当前段
            case CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_SEGMENT:
                ROS_INFO("Run segment: [RESCHEDULE]");
                retry_mode_ = RetryMode::None;
                step_mode_ = CmdStepMode::None;
                break;

            // 停止当前段执行
            case CMD_SCHEDULE_RESULT_PLEASE_STOP:
                ROS_INFO("Run segment: [STOP]");
                break;

            // 系统退出
            case CMD_SCHEDULE_RESULT_PLEASE_QUIT:
                ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_INFO("!!!!!QUITING SYSTEM!!!!!");
                ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!");
                request_to_quit_system = true;
                break;

            default:
                ROS_ERROR_STREAM("Unhandled schedule result: " << schedule_result);
                break;
            }

            // 如果请求停止调度
            if (schedule_result == CMD_SCHEDULE_RESULT_PLEASE_STOP
                || schedule_result == CMD_SCHEDULE_RESULT_PLEASE_QUIT)
            {
                ROS_INFO("Stopping scheduling");
                sendCurrentSegmentStatus(CMD_RSP_SEGMENT_STATUS_STOP);
                break;
            }
        }

        // 调度结束允许手工操作
        enableManuallyOperationMode(true);
        send_info_to_ground_station(getStatusDigitalMsgPub(),14,"通知-作业文件正常完成","info");
        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");


        // 请求退出系统
        if (request_to_quit_system)
        {
            ROS_INFO("System quiting");
            break;
        }
    }
    status_ = SCHEDULER_STATUS_NOT_STARTED;
    resetRequest();

    ROS_INFO("Segment scheduler thread ended");
}

// 运行命令行
CmdScheduleResult SegmentScheduler::runCommandLine(const CommandLine& line, const bool in_parallel, const int thread_id)
{
    // 获取命令行类型
    const JobLineType job_line_type = JobParser::ParseJobLineType(line.getContent());
    if (func_map_.find(job_line_type) == func_map_.end())
    {
        ROS_ERROR_STREAM("Unsupported job line type: " << job_line_type);
        return CMD_SCHEDULE_RESULT_PLEASE_STOP;
    }

    // 步进模式下忽略工具类指令 改为后退忽略工具相关指令
    if (step_mode_ == CmdStepMode::Backward
            && (JobParser::IsToolRelatedCommand(job_line_type) || job_line_type == JOB_LINE_TYPE_MANNUALLY_CONFIRM))
    {
        ROS_INFO_STREAM("Skipped tool related command (step mode): " << line);
        return CMD_SCHEDULE_RESULT_SUCCESS;
    }

    if (step_mode_ == CmdStepMode::Backward)
    {
        // 步进后退模式下忽略变量操作类指令
        if ((JobParser::IsVarOpCommand(job_line_type)) || (JobParser::IsLidarOpCommand(job_line_type)))
        {
            ROS_INFO_STREAM("Skipped var op command (step backward mode): " << line);
            return CMD_SCHEDULE_RESULT_SUCCESS;
        }
        // 步进后退时，为防止作业文件靠后的变量赋值污染靠前的指令，使用缓存的变量值
        else if((JobParser::IsArmRelatedCommand(job_line_type)))//改工具相关LOAD 190917
        {
            loadCachedVariables(line);
        }
    }

    if (job_line_type != JOB_LINE_TYPE_NO_OP)
    {
        ROS_INFO_STREAM("Thread " << thread_id << ": Executing line: " << line);
    }

    // 执行命令
    CmdLineFuncRsp cmd_rsp = func_map_[job_line_type](*this, line.getContent());

    // 检查执行结果
    if (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS)
    {
        if (job_line_type != JOB_LINE_TYPE_NO_OP)
        {
            ROS_INFO_STREAM("Thread " << thread_id << ": (" << line.getLineNo()
                            << "): Command line response: [" << to_string(cmd_rsp) << "]");
        }

        // 没有外部请求且是过程类指令, 不需要停下
        if (!hasRequest() && JobParser::IsProcessCommand(job_line_type))
        {
            return CMD_SCHEDULE_RESULT_SUCCESS;
        }

        // 将命令行使用的变量缓存下来，以待步进后退时使用 
        if (JobParser::IsArmRelatedCommand(job_line_type) && step_mode_ != CmdStepMode::Backward)
        {
            cacheVariables(line);
        }
    }
    else
    {
        if (job_line_type != JOB_LINE_TYPE_NO_OP)
        {
            ROS_ERROR_STREAM("Thread " << thread_id << ": (" << line.getLineNo()
                            << "): Command line response: [" << to_string(cmd_rsp) << "]");
        }
        sendCurrentSegmentStatus(CMD_RSP_SEGMENT_STATUS_ERROR);
    }

    // 指令有语法错误时，直接停止作业
    if (cmd_rsp == CMD_LINE_FUNC_RSP_SYNTAX_ERROR)
    {
        send_info_to_ground_station(getStatusDigitalMsgPub(),16,"作业文件语法错误，停止作业","info");
    
        ROS_ERROR("There are syntax error in your job file. For saftty, this job will be stop");
        requestToStop();
    }

    // 并行模式下需要同步并行并行指令的执行结果，以确定是否需要手工确认
    if (in_parallel)
    {
        ROS_INFO_STREAM("Thread " << thread_id << ": Waiting for other parallel threads to complete command");
        
        cmd_rsp = synParallelCommandResults(cmd_rsp);
        if (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS)
        {
            ROS_INFO_STREAM("Thread " << thread_id << ": Grant total command result: [ALL SUCCESS]");
        }
        else
        {
            ROS_ERROR_STREAM("Thread " << thread_id << ": Grant total command result: [SOME FAILED]");
        }
    }

    // 在执行命令期间，地面站可能发来了操作请求（如手工确认指令），优先响应外部请求
    // 让代表线程请求手工确认，防止并行模式下重复请求导致程序崩溃
    if (thread_id == parallel_master_thread_id_)
    {
        // 命令执行出错/步进模式(非重试模式)/请求暂停需要手工确认
        if (cmd_rsp != CMD_LINE_FUNC_RSP_SUCCESS || (step_mode_ != CmdStepMode::None && retry_mode_ == RetryMode::None)
            || request_ == SCHEDULE_REQUEST_PAUSE)
        {
            if (!hasRequest())
            {
                // 主动暂停,等待手工确认
                request_ = SCHEDULE_REQUEST_PAUSE;
            }

            // 循环查询是否要求暂停系统
            while (isRequestedToPause())
            {
                ManuallyOperationScope scope(*this);
                ROS_INFO("Thread %d: System pause! Waiting for manually confirm", thread_id);
                send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统请求暂停，等待人工处理","info");
    
                if (!waitManuallyConfirm())
                {
                    ROS_ERROR("Thread %d: No response from ground station. For safety, the remaining job will be drop", thread_id);
                    return CMD_SCHEDULE_RESULT_PLEASE_STOP;
                }
            }
        }
        parallel_request_ = request_;   // 只有一个代表线程，因此这里不需要加锁
        resetRequest();
    }

    ScheduleRequest request = parallel_request_;
    if (in_parallel) // 同步调度请求
    {
        ROS_INFO_STREAM("Thread " << thread_id << ": Synchronizing schedule request from thread " << parallel_master_thread_id_);
        request = synRequest();
    }

    // 如果没有请求,则静默执行
    if (request == SCHEDULE_REQUEST_NONE)
    {
        ROS_INFO_STREAM("Thread " << thread_id << ": No schedule request, continue running");
        return CMD_SCHEDULE_RESULT_SUCCESS;
    }

    // 处理调度请求
    CmdScheduleResult new_result = CMD_SCHEDULE_RESULT_PLEASE_STOP;
    switch (request)
    {
    case SCHEDULE_REQUEST_CONTINUE:
        ROS_INFO("Thread %d: Manually confirmed: continue scheduling", thread_id);
        // 防止当前是手工确认指令导致永久继续
        new_result = job_line_type != JOB_LINE_TYPE_MANNUALLY_CONFIRM ?
                        CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD : CMD_SCHEDULE_RESULT_SUCCESS;
        break;

    case SCHEDULE_REQUEST_SKIP:
        ROS_INFO("Thread %d: Manually confirmed: skip scheduling", thread_id);
        new_result = CMD_SCHEDULE_RESULT_SUCCESS;
        break;

    case SCHEDULE_REQUEST_STOP:
        ROS_INFO("Thread %d: Manually confirmed: stop scheduling", thread_id);
        new_result = CMD_SCHEDULE_RESULT_PLEASE_STOP;
        break;

    case SCHEDULE_REQUEST_QUIT:
        ROS_INFO("Thread %d: Manually confirmed: quit system", thread_id);
        new_result = CMD_SCHEDULE_RESULT_PLEASE_QUIT;
        break;

    case SCHEDULE_REQUEST_RESCHEDULE_SEGMENT:
        ROS_INFO("Thread %d: Manually confirmed: retry segment", thread_id);
        // 借助步进后退实现重试
        if (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS || last_step_mode_ != CmdStepMode::Backward)
            new_result = CMD_SCHEDULE_RESULT_SUCCESS;
        else
            new_result = CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD;
        break;

    case SCHEDULE_REQUEST_STEP_FORWARD:
        ROS_INFO("Thread %d: Manually confirmed: step forward", thread_id);
        // 在步进后退模式下，若要求前进，则不该继续执行当前指令（即退不到过去的位置，就该放弃）
        if (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS || last_step_mode_ == CmdStepMode::Backward)
            new_result = CMD_SCHEDULE_RESULT_SUCCESS;
        else
            new_result = CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD;
        break;

    case SCHEDULE_REQUEST_STEP_BACKWARD:
        ROS_INFO("Thread %d: Manually confirmed: step backward", thread_id);
        // 在步进前进/正常前进执行模式下，若要求后退，则不该继续执行当前指令（即去不了想去的位置，就该放弃）
        if (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS || last_step_mode_ != CmdStepMode::Backward)
            new_result = CMD_SCHEDULE_RESULT_SUCCESS;
        else
            new_result = CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD;
        break;

    default:
        ROS_ERROR("Thread %d: Response from ground station didn't specify next action. "
                  "For safety, the remaining job will be drop", thread_id);
        new_result = CMD_SCHEDULE_RESULT_PLEASE_STOP;
        break;
    }

    return new_result;
}

// 运行顺序命令块
CmdScheduleResult SegmentScheduler::runSequentailBlock(const CommandBlock& block, const bool in_parallel, const int thread_id)
{
    if (in_parallel) { ROS_DEBUG_STREAM("Thread " << thread_id << ": Started"); }

    assert (block.getType() == CommandBlock::COMMAND_BLOCK_TYPE_SEQUENTIAL);

    CmdScheduleResult schedule_result = CMD_SCHEDULE_RESULT_SUCCESS;

    const CommandLines& cmd_lines = block.getCommandLines();

    bool need_break = false;
    size_t line_index = step_mode_ == CmdStepMode::Backward ? cmd_lines.size()-1 : 0;
    while (line_index < cmd_lines.size() && line_index >= 0)
    {
        const CommandLine& cmd_line = cmd_lines[line_index];
        
        schedule_result = runCommandLine(cmd_line, in_parallel, thread_id);

        switch (schedule_result)
        {
        // 执行下一条指令
        case CMD_SCHEDULE_RESULT_SUCCESS:
            line_index += step_mode_ == CmdStepMode::Backward ? -1 : 1;
            break;

        // 继续执行当前指令 
        case CMD_SCHEDULE_RESULT_PLEASE_RESCHEDULE_CMD:
            break;

        // 其他结果委托给上级处理
        default:
            need_break = true;
        }
        if (need_break) break;
    }

    if (in_parallel) { ROS_DEBUG_STREAM("Thread " << thread_id << ": Ended"); }

    return schedule_result;
}

// 运行并行命令块
CmdScheduleResult SegmentScheduler::runParallelBlock(const CommandBlocks& blocks)
{
    // 设置并行工作线程数
    setRequestWaiterNum(blocks.size());

    // 计算块中与机械臂相关的命令数 改成非变量操作类补空指令
    auto count_arm_commands = [] (const CommandBlock& block) {
        return count_if(block.getCommandLines().cbegin(), block.getCommandLines().cend(),
                        [] (const CommandLine& line) {
                            return !JobParser::IsVarOpCommandLine(line.getContent());
                        });
    };

    // 计算各个块中的机械臂相关指令数，求其最大者
    size_t max_num_arm_commands = 0;
    int index_of_block_that_has_most_arm_commmands = 0;
    for (size_t i = 0; i < blocks.size(); ++i)
    {
        const size_t num_arm_commands = count_arm_commands(blocks[i]);
        if (num_arm_commands > max_num_arm_commands)
        {
            max_num_arm_commands = num_arm_commands;
            index_of_block_that_has_most_arm_commmands = i;
        }
    }
    parallel_master_thread_id_ = index_of_block_that_has_most_arm_commmands;

    ROS_INFO_STREAM("Max num arm commands in parallel blocks: " << max_num_arm_commands);
    ROS_INFO_STREAM("Parallel master thread ID set as: " << parallel_master_thread_id_);

    // 在并行块中填充空指令，对齐机械臂类指令数
    auto pad_noop_cmds = [&] (CommandBlock block) {
        const string noop_cmd = JobParser::CreateNoOpCommand();
        for (size_t i = count_arm_commands(block); i < max_num_arm_commands; ++i)
        {
            block.addCommandLine(CommandLine(noop_cmd, 0));
        }
        return block;
    };

    // 存储各个并行块的执行结果
    std::vector<std::future<CmdScheduleResult>> future_results(blocks.size());
    for (size_t i = 0; i < blocks.size(); ++i)
    {
        future_results[i] =
            std::async(std::launch::async,
                       std::bind(&SegmentScheduler::runSequentailBlock, this,
                                 pad_noop_cmds(blocks[i]), true, i));
    }

    // 等待并存储并行块的执行结果
    CmdScheduleResult total_result = CMD_SCHEDULE_RESULT_SUCCESS;
    for (auto& future_result : future_results)
    {
        const CmdScheduleResult one_result = future_result.get();
        if (one_result != CMD_SCHEDULE_RESULT_SUCCESS)
        {
            total_result = one_result;
        }
    } 

    parallel_master_thread_id_ = 0;
    num_parallel_threads_ = 1;

    return total_result;
}

// 运行计算命令块
CmdScheduleResult SegmentScheduler::runCalculationBlock(const CommandBlock& block)
{
    assert (block.getType() == CommandBlock::COMMAND_BLOCK_TYPE_CALCULATION);

    const CommandBlock sequentail_block(CommandBlock::COMMAND_BLOCK_TYPE_SEQUENTIAL, block.getCommandLines());
    
    return runSequentailBlock(sequentail_block);
}

// 同步所有并行线程的调度请求
SegmentScheduler::ScheduleRequest SegmentScheduler::synRequest()
{
    unsigned int current_num_waiting_parallel_threads = 0;

    {
        std::lock_guard<std::mutex> lk(parallel_threads_mutex_);
        ++num_waiting_parallel_threads_;
        ROS_INFO_STREAM("num_waiting_parallel_threads_: "<< num_waiting_parallel_threads_);
        current_num_waiting_parallel_threads = num_waiting_parallel_threads_;
    }

    // 第一个进来的线程负责复位标志位
    if (current_num_waiting_parallel_threads == 1)
    {
        {
            //std::lock_guard<std::mutex> lk(parallel_threads_mutex_);

            parallel_request_threads_all_ready_ = false;
        }
    }

    // 最后一个进来的线程负责通知其他线程全员就绪
    if (current_num_waiting_parallel_threads == num_parallel_threads_)
    {
        {
            //std::unique_lock<std::mutex> lk(parallel_threads_mutex_);
            num_waiting_parallel_threads_ = 0;
            parallel_request_threads_all_ready_ = true;
        }
        ROS_INFO("LAST Thread notify all");
        parallel_threads_cv_.notify_all();
    }
    else  // 不是最后进来的线程需要等待
    {
        std::unique_lock<std::mutex> lk(parallel_threads_mutex_);
        parallel_threads_cv_.wait(lk, [&] { return parallel_request_threads_all_ready_; });
    }

    return parallel_request_;
}

// 同步所有并行线程的执行结果
CmdLineFuncRsp SegmentScheduler::synParallelCommandResults(const CmdLineFuncRsp rsp)
{
    unsigned int current_num_waiting_parallel_threads = 0;

    {
        std::lock_guard<std::mutex> lk(parallel_threads_mutex_);
        ++num_waiting_parallel_threads_;
        current_num_waiting_parallel_threads = num_waiting_parallel_threads_;
        ROS_INFO_STREAM("When enter thread is :"<< num_waiting_parallel_threads_);
    }

    // 第一个进来的线程负责复位标志位
    if (current_num_waiting_parallel_threads == 1)
    {
        {
            std::lock_guard<std::mutex> lk(parallel_threads_mutex_);
            parallel_threads_all_ready_ = false;
            parallel_commands_all_ok_ = true;
        }
    }

    // 并发写入执行结果
    {
        std::lock_guard<std::mutex> lk(parallel_threads_mutex_);
        if (parallel_commands_all_ok_ && rsp != CMD_LINE_FUNC_RSP_SUCCESS)
        {
            parallel_commands_all_ok_ = false;
        }
    }

    // 最后一个进来的线程负责通知其他线程全员就绪
    if (current_num_waiting_parallel_threads == num_parallel_threads_)
    {
        {
            //std::unique_lock<std::mutex> lk(parallel_threads_mutex_);
            num_waiting_parallel_threads_ = 0;
            ROS_INFO_STREAM("When start unlock num is :"<< num_waiting_parallel_threads_);
    
            parallel_threads_all_ready_ = true;

        }
        parallel_threads_cv_.notify_all();
    }
    else  // 不是最后进来的线程需要等待
    {
        std::unique_lock<std::mutex> lk(parallel_threads_mutex_);
        parallel_threads_cv_.wait(lk, [&] { return parallel_threads_all_ready_; });
    }
    ROS_INFO_STREAM("When leave thread is :"<< num_waiting_parallel_threads_);
    

    return parallel_commands_all_ok_ ? CMD_LINE_FUNC_RSP_SUCCESS : CMD_LINE_FUNC_RSP_FAILED;
}

// 从命令行变量值缓存表中加载该命令行的变量值
void SegmentScheduler::loadCachedVariables(const CommandLine& cmd_line)
{
    auto iter = variable_assignment_cache.find(cmd_line);

    // 没有缓存该指令，不需要加载，直接返回
    if (iter == variable_assignment_cache.cend()) return;

    // 执行该命令行缓存的所有变量赋值语句
    for (auto& line : iter->second)
    {
        ROS_DEBUG_STREAM("(" << cmd_line.getLineNo() << ") Load cached variable: " << line.getContent());

        const JobLineType job_line_type = JobParser::ParseJobLineType(line.getContent());
        assert (func_map_.find(job_line_type) != func_map_.cend());

        const CmdLineFuncRsp cmd_rsp = func_map_[job_line_type](*this, line.getContent());
        assert (cmd_rsp == CMD_LINE_FUNC_RSP_SUCCESS);
    }
}

// 缓存命令行使用到的变量值
void SegmentScheduler::cacheVariables(const CommandLine& cmd_line)
{
    const vector<string> fields = JobParser::GetCommandFields(cmd_line.getContent());
    CommandLines cmd_lines;

    // 将变量字段转为变量，读取变量的值，并将其转为变量赋值语句
    size_t no = 0;
    for (auto& field : fields)
    {
        const string line_content = jp_.variableToAssignmentCommand(field);
        if (!line_content.empty())
        {
            ++no;
            cmd_lines.push_back(CommandLine(line_content, no));
            ROS_DEBUG_STREAM("(" << cmd_line.getLineNo() << ") Write variable to cache: " << line_content);
        }
    }

    if (!cmd_lines.empty())
    {
        variable_assignment_cache[cmd_line] = cmd_lines;
    }
}

// 发送步态信息
// @segment_index: 步序
// @status: 步态
inline void SegmentScheduler::sendCurrentSegmentStatus(const CmdRspSegmentStatus status)
{
    send_command_response_to_ground_station(
        cmd_rsp_msg_pub_, CONTROL_COMMAND_ID_BEGIN_JOB, current_scheduling_segment_no_, status);
}

// 向机械臂发送停止运动指令
void SegmentScheduler::stopArmMotion()
{
    std::string msg;
    msg = "stopj(0.5)\n";

    ROS_INFO("Sending stop command to robot arm motion");
    send_command_to_robot_script(left_arm_control_pub_,1,msg,ROBOT_ARM_LEFT_ARM);
    // sleep(10);
    send_command_to_robot_script(left_arm_control_pub_,2,msg,ROBOT_ARM_LEFT_ARM);
    send_command_to_robot_script(right_arm_control_pub_,1,msg,ROBOT_ARM_RIGHT_ARM);
    // sleep(10);
    send_command_to_robot_script(right_arm_control_pub_,2,msg,ROBOT_ARM_RIGHT_ARM);  
}


// 运行机械臂电源状态控制线程
void SegmentScheduler::arm_power_run()
{
    ROS_INFO("Service scheduler starting.....");
    while(true)
    {
        const int control_arm_power_flag = waitServiceReach();
        ROS_INFO_STREAM("control_arm_power_flag: " << control_arm_power_flag);
        //ROS_INFO("ServiceReached..........");

        switch (control_arm_power_flag)
        {
            case POWER_MANAGER_ARM_POWER_ON:
            {    
                // 左臂连接成功，等待上电标志
                if (wait_left_arm_connection_SingleReach() == LEFT_ARM_CONNECTED )  
                {
                    //主臂未上电，发送上电服务
                    if (wait_left_arm_power_on_SingleReach(20) == WAIT_POWER_ON_TIME_OUT)
                    {
                        ROS_INFO("left_arm_connected , starting power on......");
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"主臂连接成功，开始上电","info");
                        call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_POWER_ON,  CMD_FUNC_ARM_ID_LEFT_ARM, ARM_POWER_ON_COMMAND);
                    }
                }
                else
                {
                    break;
                }
                // 右臂连接成功，发送上电服务
                if (wait_right_arm_connection_SingleReach() == RIGHT_ARM_CONNECTED )//&& wait_right_arm_power_on_SingleReach() != RIGHT_ARM_POWER_ON )  
                {
                    if (wait_right_arm_power_on_SingleReach(20) == WAIT_POWER_ON_TIME_OUT)
                    {
                        ROS_INFO("right_arm_connected , starting power on.....");
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"从臂连接成功，开始上电","info");
                        call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_POWER_ON,  CMD_FUNC_ARM_ID_RIGHT_ARM, ARM_POWER_ON_COMMAND);
                    }
                }
                else
                {
                    break;
                }

                if(wait_left_arm_power_on_SingleReach(60) == 1)
                {
                    // 检left伺服打开标志
                    if (wait_left_arm_enable_SingleReach(5) == 2) 
                    {
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"主臂已打开抱闸","info");
                               
                    }
                    else
                    {
                        std::this_thread::sleep_for(chrono::milliseconds(5000));//500ms

                        ROS_INFO("left_arm_already_power_on , starting brake release.....");
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"主臂已上电，打开抱闸","info");
                        call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_BRAKE_RELEASE,  CMD_FUNC_ARM_ID_LEFT_ARM, ARM_BRAKE_RELEASE_COMMAND);
                    

                    }
                }
                else
                {
                   send_info_to_ground_station(getStatusDigitalMsgPub(),15,"主臂发送了上电服务，等待60S未检测到上电标志，检测电源板故障","info");
                         
                    break;
                }
                
                if(wait_right_arm_power_on_SingleReach(60) == 101)
                {
                // 检查伺服打开标志
                    if (wait_right_arm_enable_SingleReach(5) == 102 ) 
                    {
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");                        
                                 
                    }
                    else
                    {
                        std::this_thread::sleep_for(chrono::milliseconds(5000));//500ms

                        ROS_INFO("right_arm_already_power_on , starting brake release.....");
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"从臂已上电，打开抱闸","info");
                        call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_BRAKE_RELEASE,  CMD_FUNC_ARM_ID_RIGHT_ARM, ARM_BRAKE_RELEASE_COMMAND);
                        std::this_thread::sleep_for(chrono::milliseconds(2000));//500ms
                        send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");
                        
                         }
                    
                }
                else
                {
                   send_info_to_ground_station(getStatusDigitalMsgPub(),15,"从臂发送了上电服务，等待60S未检测到上电标志，检测电源板故障","info");
                         
                    break;
                }

                
                break;
            }
            
            

            case ARM_EMERGENCY_STOP_RELIEVE:
            {
                ROS_INFO("arm emergency stop releave now......");
                // 左臂急停状态，发送左臂上电服务,// 右臂急停状态，发送右臂上电服务
                if (wait_arm_emergency_stopped_SingleReach(10) == LEFT_ARM_EMERGENCY_STOPPED)
                {
                    std::this_thread::sleep_for(chrono::milliseconds(2000));//500ms

                    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"通知机械臂上电","info"); 
            
                    //ROS_INFO("arm emergency stop releave now......");
                    call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_POWER_ON,  CMD_FUNC_ARM_ID_LEFT_ARM, ARM_POWER_ON_COMMAND);
                
                    call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_POWER_ON,  CMD_FUNC_ARM_ID_RIGHT_ARM, ARM_POWER_ON_COMMAND);
                }else
                {
                    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"没有取消掉急停标志，解除急停失败","info");
                     

                    break;
                }

                
                
                // 左臂上电成功，发送打开抱闸服务
                if (wait_left_arm_power_on_SingleReach(20) == LEFT_ARM_POWER_ON)
                {
                    std::this_thread::sleep_for(chrono::milliseconds(5000));//500ms
                    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"通知主臂解除抱闸","info");             

                    call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_BRAKE_RELEASE,  CMD_FUNC_ARM_ID_LEFT_ARM, ARM_BRAKE_RELEASE_COMMAND);
                }
                // 右臂上电成功，发送打开抱闸服务
                if (wait_right_arm_power_on_SingleReach(20) == RIGHT_ARM_POWER_ON)
                {
                
                    std::this_thread::sleep_for(chrono::milliseconds(5000));//500ms
                    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"通知从臂解除抱闸","info");             

                        
                    call_power_control_srv(getPowerControlServiceClient(), CMD_CONTROL_POWER_MANAGER_ARM_BRAKE_RELEASE,  CMD_FUNC_ARM_ID_RIGHT_ARM, ARM_BRAKE_RELEASE_COMMAND);
                    send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");
                    
                }

               

                break;
            }

            case UNLOCK_ARM_PROTECTIVE_STOP:
            {
                ROS_INFO("starting unlock arm protective stop .......");
                call_power_control_srv(getPowerControlServiceClient(), 0,  CMD_FUNC_ARM_ID_LEFT_ARM, ARM_UNLOCK_PROTECTIVE_STOP_COMMAND);
                call_power_control_srv(getPowerControlServiceClient(), 0,  CMD_FUNC_ARM_ID_RIGHT_ARM, ARM_UNLOCK_PROTECTIVE_STOP_COMMAND);
                send_info_to_ground_station(getStatusDigitalMsgPub(),13,"系统空闲","info");
                    
                break;
            }


            default:
                ROS_INFO("Service==***==Stoped.....");
                break;
        }
    } 
}


// 主控运行心跳状态发布线程
void SegmentScheduler::heart_beat()
{
    while(true)
    {
       send_heart_beat(getHeartBeatPub()); 
       if (!waitheartbeartEnd()) // 延时4s
       {
           ROS_INFO("heart beat end!");
           break;
       }   
    }
}

void SegmentScheduler::remote_heart_beat()
{
    while(true)
    {
        auto& event = remote_hearbeat_flag_;
        if (!event.wait_for(chrono::seconds(5)))// 与地面站通信中断暂停作业
        {
            //ROS_INFO("remote station heart beat shutdown");
            creat_logfile(LOG_REMOTE_CONTROL,"地面站心跳中断，主控暂停当前作业");
            if (!scheduler->isIdle()) 
            {
                scheduler->requestToPause();
                if (scheduler->isWaitingManuallyConfirm())
                {
                    scheduler->notifyManuallyConfirm();
                }
            }
            continue ;
        }  
        else if (event.get() == false) 
        ROS_INFO("remote_heart_beat check quiting!");
        return ;
    }
}
