#ifndef CMD_FUNC_H
#define CMD_FUNC_H

#include <string>

#include "Variable.h"
#include "ros/ros.h"

// 调用命令行返回值定义
enum CmdLineFuncRsp
{
    CMD_LINE_FUNC_RSP_SUCCESS,                      // 调用成功
    CMD_LINE_FUNC_RSP_FAILED,                       // 调用失败
    CMD_LINE_FUNC_RSP_SYNTAX_ERROR,                 // 调用参数不合法，如参数个数或类型不符合定义
    CMD_LINE_FUNC_RSP_TIMEOUT,                      // 调用超时
};

enum SingalDevice
{
    RemoteControl,                      // 地面站
    Left_arm,                       
  
};


// 将命令行返回值类型转为字符串
std::string to_string(const CmdLineFuncRsp rsp);

class SegmentScheduler;

// 命令执行函数

// 机械臂指令
CmdLineFuncRsp movej_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movej1_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movej2_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movej3_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movel_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movel1_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movel2_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp movel2_waitsinal_line_func(SegmentScheduler& scheduler, const std::string& line);

bool wait_device_signal(SegmentScheduler& scheduler, const std::string& SingalDevice, const int& SingalSeq,const int& SingalValue);

// 变量操作指令
CmdLineFuncRsp var_operator_assignment_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp offset_line_func(SegmentScheduler& scheduler, const std::string& line);

// 滑台指令
CmdLineFuncRsp slider_move_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp slider_go_home_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp slider_record_pos_line_func(SegmentScheduler& scheduler, const std::string& line);

CmdLineFuncRsp Slider_moveabs_func(SegmentScheduler& scheduler,int id,double pos);
// 杂项指令
CmdLineFuncRsp manually_confirm_line_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp waittime_line_func(SegmentScheduler& scheduler, const std::string& line);

// 激光雷达指令
CmdLineFuncRsp makepoint_lidar1_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidar2_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma1_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma2_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma3_func(SegmentScheduler& scheduler, const std::string& line);

CmdLineFuncRsp makepoint_lidartoma1_AT_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma2_AT_func(SegmentScheduler& scheduler, const std::string& line);
CmdLineFuncRsp makepoint_lidartoma3_AT_func(SegmentScheduler& scheduler, const std::string& line);

// 末端接线工具指令
//
// 接线拧断开始指令功能函数
CmdLineFuncRsp connect_tightenstart_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线拧断停止指令功能函数
CmdLineFuncRsp connect_tightenstop_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线拧断反转指令功能函数
CmdLineFuncRsp connect_tightenback_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线拧断初始化指令功能函数
CmdLineFuncRsp connect_tighteninit_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线线夹锁定指令功能函数
CmdLineFuncRsp connect_clip_lock_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线线夹解锁指令功能函数
CmdLineFuncRsp connect_clipunlock_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线套筒锁定指令功能函数
CmdLineFuncRsp connect_sleevelock_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线套筒解锁指令功能函数
CmdLineFuncRsp connect_sleeveunlock_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线套筒停止指令功能函数
CmdLineFuncRsp connect_sleevestop_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线支线夹紧指令功能函数
CmdLineFuncRsp connect_branchlinelock_line_func(SegmentScheduler& scheduler, const std::string& line);
// 接线支线解锁指令功能函数
CmdLineFuncRsp connect_branchlineunlock_line_func(SegmentScheduler& scheduler, const std::string& line);

// 末端剪线工具指令
//
// 剪线开始指令功能函数
CmdLineFuncRsp cut_start_line_func(SegmentScheduler& scheduler, const std::string& line);
// 剪线复位指令功能函数
CmdLineFuncRsp cut_init_line_func(SegmentScheduler& scheduler, const std::string& line);

// 末端夹爪工具指令
//
// 打开夹爪指令功能函数
CmdLineFuncRsp open_gripper_line_func(SegmentScheduler& scheduler, const std::string& line);
// 关闭夹爪指令功能函数
CmdLineFuncRsp close_gripper_line_func(SegmentScheduler& scheduler, const std::string& line);

// 剥线工具指令
// 一键剥皮指令功能函数
CmdLineFuncRsp strip_onekey_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮初始化指令功能函数
CmdLineFuncRsp strip_init_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮正转功能函数
CmdLineFuncRsp strip_rotate_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮反转功能函数
CmdLineFuncRsp strip_backrotate_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮开口向上指令功能函数
CmdLineFuncRsp strip_upward_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮推杆左指令功能函数
CmdLineFuncRsp strip_linkleft_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮推杆右指令功能函数
CmdLineFuncRsp strip_linkright_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮推杆上指令功能函数
CmdLineFuncRsp strip_linkup_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮推杆下指令功能函数
CmdLineFuncRsp strip_linkdown_line_func(SegmentScheduler &scheduler, const std::string &line);
// 剥皮停止指令功能函数
CmdLineFuncRsp strip_stop_line_func(SegmentScheduler &scheduler, const std::string &line);

// 复位工具
CmdLineFuncRsp tool_reset_line_func(SegmentScheduler& scheduler, const std::string& line);

//机械臂电源控制
//CmdLineFuncRsp power_control_func(SegmentScheduler& scheduler,const int seq);

// 运算指令
CmdLineFuncRsp operator_var(SegmentScheduler& scheduler, const std::string& line);

// 获取机械臂当前位置
CmdLineFuncRsp get_ma_pos_func(SegmentScheduler& scheduler, const std::string& line);

CmdLineFuncRsp get_Joint_value(SegmentScheduler& scheduler, int MaID, DoubleValueArray& point);


#endif // CMD_FUNC_H
