#ifndef CONFIG_H
#define CONFIG_H

// 是否隔离真实地面站结点进行测试
// #define SEPERATE_TESTING

// 作业文件目录
#define JOB_FILE_DIRECTORY "./doc/job_file"

// 机械臂控制脚本目录
#define ARM_CONTROL_SCRIPT_DIRECTORY "./doc/arm_control_script"

// -----------------
// 设备ID定义
// -----------------
// 机械臂ID设置
#define CMD_FUNC_ARM_ID_LEFT_ARM 1
#define CMD_FUNC_ARM_ID_RIGHT_ARM 2

// 滑台ID设置
#define CMD_FUNC_SLIDER_ID_VERTICAL_SLIDER 1
#define CMD_FUNC_SLIDER_ID_HORIZONTAL_SLIDER 2

// -----------------
// 话题定义
// -----------------

// 主控心跳话题
#define HEART_BEAT_MESSAGE_TOPIC "/nari/szrd/dnrobot/node/heartbeat"

// 地面站心跳话题
#define REMOTECONTROL_HEARTBEAT_MESSAGE_TOPIC "/nari/szrd/dnrobot/remotecontrol/heartbeat"

// 通用数字量话题
#define GENERAL_DIGITAL_MESSAGE_TOPIC "/nari/szrd/dnrobot/yx"

// 其他通用数字量话题，给地面站
#define GENERAL_DIGITAL_MAINCONTROL_MESSAGE_TOPIC "/nari/szrd/dnrobot/maincontrol/yx"

// 通用模拟量话题
#define GENERAL_ANALOG_MESSAGE_TOPIC "/nari/szrd/dnrobot/yc"

// 左右机械臂控制话题
#define LEFT_ARM_CONTROL_TOPIC  "/l_arm_controller/ur_driver/NariURScript"
#define RIGHT_ARM_CONTROL_TOPIC "/r_arm_controller/ur_driver/NariURScript"

// 机械臂末端位姿数据话题
#define LEFT_ARM_TOOL_POINT_POSITION_TOPIC  "left_arm/actual_tcp_pose"
#define RIGHT_ARM_TOOL_POINT_POSITION_TOPIC "right_arm/actual_tcp_pose"

// 机械臂到位信息监听话题
#define COMMAND_RESPONSE_LEFT_ARM_LISTEN_TOPIC "left_arm/ur_driver/io_states"
#define COMMAND_RESPONSE_RIGHT_ARM_LISTEN_TOPIC "right_arm/ur_driver/io_states"

// 命令消息话题
#define COMMAND_MESSAGE_TOPIC "/nari/szrd/dnrobot/cmd"

// 命令消息回复话题
#define COMMAND_RESPONSE_MESSAGE_TOPIC "/nari/szrd/dnrobot/cmdRsp"

// 激光雷达通信话题
#define LIDAR1_LISTEN_TOPIC "/sys_0/server_position"
#define LIDAR2_LISTEN_TOPIC "/sys_1/server_position"

#define LIDAR1B_LISTEN_TOPIC "/sys_3/server_position"
#define LIDAR2B_LISTEN_TOPIC "/sys_4/server_position"

// 自动选点
#define RANGEPARAM_TOPIC "/rangeparam"
#define GROUND_INTERACT_0_TOPIC "/sys_0/ground_interact"
#define GROUND_INTERACT_1_TOPIC "/sys_1/ground_interact"
#define GROUND_INTERACT_3_TOPIC "/sys_3/ground_interact"
#define GROUND_INTERACT_4_TOPIC "/sys_4/ground_interact"
#define GROUND_CHECK_TOPIC1 "sys_0/ground_check"
#define GROUND_CHECK_TOPIC2 "sys_1/ground_check"
#define GROUND_CHECK_TOPIC3 "sys_2/ground_check"
#define GROUND_CHECK_TOPIC4 "sys_3/ground_check"

// -----------------
// 服务定义
// -----------------
// 滑台写服务
#define SLIDER_CONTROL_SERVICE_NAME "/pkg_slider/srv_slider_control"

// 机械臂电源控制服务
//#define  ARM_POWER_CONTROL_SERVICE_CLIENT   "/nari/szrd/demo_pkg/cmd"
#define  ARM_POWER_CONTROL_SERVICE_CLIENT     "/nari/szrd/dnrobot/arm_power_control"

// -----------------
// 滑台写服务序号定义
// -----------------
#define SERVICE_SEQ_HORIZONTAL_SLIDER_GO_HOME 1
#define SERVICE_SEQ_HORIZONTAL_SLIDER_REACH 2
#define SERVICE_SEQ_VERTICAL_SLIDER_GO_HOME 4
#define SERVICE_SEQ_VERTICAL_SLIDER_REACH 5

// -----------------
// 控制对象定义
// -----------------
#define CONTROL_OBJECT_POWER_MANAGER 100
#define CONTROL_OBJECT_MASTER_CONTROLLER 200
#define CONTROL_OBJECT_GROUND_STATION 201
#define CONTROL_OBJECT_LEFT_ARM 300
#define CONTROL_OBJECT_RIGHT_ARM 301
#define CONTROL_OBJECT_VERTICAL_SLIDER 400
#define CONTROL_OBJECT_HORIZONTAL_SLIDER 401
#define LIDAR_POINT_CONFIRM 600

// 末端工具对象
#define CONTROL_OBJECT_CLAMP_WIRE_TOOL_STRIP 500        // 剥线工具
#define CONTROL_OBJECT_CLAMP_WIRE_TOOL_CONNECT 501      // 接线工具
#define CONTROL_OBJECT_CLAMP_WIRE_TOOL_CUT 502          // 剪线工具
#define CONTROL_OBJECT_CLAMP_WIRE_TOOL_GRIPPER 503      // 手爪工具

// -----------------
// 控制指令ID定义
// -----------------
// 主控到地面站
#define CONTROL_COMMAND_ID_MANUALLY_CONTROL 1
#define CONTROL_COMMAND_ID_FAST_RECALL 2
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_POS 6
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_POS 7

// -----------------
// 地面站发给主控的控制指令消息序号定义
// -----------------
// 地面站到主控
#define CONTROL_COMMAND_ID_BEGIN_JOB 1                          // 开始作业
#define CONTROL_COMMAND_ID_STOP_JOB 2                           // 停止作业
#define CONTROL_COMMAND_ID_WITHDARW_ARM 3                       // 机械臂收回
#define CONTROL_COMMAND_ID_PAUSE_JOB 4                          // 暂停作业
#define CONTROL_COMMAND_ID_CONTINUE_JOB 5                       // 继续作业
#define CONTROL_COMMAND_ID_LIFT_EXCEPTION 6                     // 解除异常（故障）
#define CONTROL_COMMAND_ID_ASK_HEARTBEAT 7                      // 心跳请求
#define CONTROL_COMMAND_ID_CONFIRM_SLIDER_POS_BUTTON  8         // 确认滑台位置按钮
#define CONTROL_COMMAND_ID_REQUEST_HORIZONTAL_SLIDER_POS  9     // 请求水平滑台位置
#define CONTROL_COMMAND_ID_REQUEST_VERTICAL_SLIDER_POS  10      // 请求垂直滑台位置
#define CONTROL_COMMAND_ID_RESCHEDULE_JOB 11                    // 重新执行作业
#define CONTROL_COMMAND_ID_OPEN_ARM_TO_CHANGE_BATTERY 12        // 机械臂张开换电池
#define CONTROL_COMMAND_ID_CLOSE_ARM_AFTER_CHANGE_BATTERY 13    // 机械臂换电池回位
#define CONTROL_COMMAND_ID_UNLOCK_ARM_PROTECTIVE_STOP 14        // 解除机械臂保护性停止
#define CONTROL_COMMAND_ID_STEP_EXECUTION_FORWARD 15            // 步进执行（前进）
#define CONTROL_COMMAND_ID_STEP_EXECUTION_BACKWARD 16           // 步进执行（后退）

// 地面站->主控->主臂
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_BASE_DIRECTION_1_MOTION 1                    // 主臂基座方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_BASE_DIRECTION_2_MOTION 2                    // 主臂基座方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_SHOULDER_DIRECTION_1_MOTION 3                // 主臂肩部方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_SHOULDER_DIRECTION_2_MOTION 4                // 主臂肩部方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_ELBOW_DIRECTION_1_MOTION 5                   // 主臂肘部方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_ELBOW_DIRECTION_2_MOTION 6                   // 主臂肘部方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_1_DIRECTION_1_MOTION 7                 // 主臂手腕1方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_1_DIRECTION_2_MOTION 8                 // 主臂手腕1方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_2_DIRECTION_1_MOTION 9                 // 主臂手腕2方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_2_DIRECTION_2_MOTION 10                // 主臂手腕2方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_3_DIRECTION_1_MOTION 11                // 主臂手腕3方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TO_WRIST_3_DIRECTION_2_MOTION 12                // 主臂手腕3方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION 13     // （基座坐标系）主臂末端位移方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION 14     // （基座坐标系）主臂末端位移方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION 15     // （基座坐标系）主臂末端位移方向3运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION 16     // （基座坐标系）主臂末端位移方向4运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION 17     // （基座坐标系）主臂末端位移方向5运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION 18     // （基座坐标系）主臂末端位移方向6运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_1_MOTION 19            // （基座坐标系）主臂末端姿态方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_2_MOTION 20            // （基座坐标系）主臂末端姿态方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_3_MOTION 21            // （基座坐标系）主臂末端姿态方向3运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_4_MOTION 22            // （基座坐标系）主臂末端姿态方向4运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_5_MOTION 23            // （基座坐标系）主臂末端姿态方向5运动
#define CONTROL_COMMAND_ID_LEFT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_6_MOTION 24            // （基座坐标系）主臂末端姿态方向6运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION 25     // （工具坐标系）主臂末端位移方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION 26     // （工具坐标系）主臂末端位移方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION 27     // （工具坐标系）主臂末端位移方向3运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION 28     // （工具坐标系）主臂末端位移方向4运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION 29     // （工具坐标系）主臂末端位移方向5运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION 30     // （工具坐标系）主臂末端位移方向6运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_1_MOTION 31            // （工具坐标系）主臂末端姿态方向1运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_2_MOTION 32            // （工具坐标系）主臂末端姿态方向2运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_3_MOTION 33            // （工具坐标系）主臂末端姿态方向3运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_4_MOTION 34            // （工具坐标系）主臂末端姿态方向4运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_5_MOTION 35            // （工具坐标系）主臂末端姿态方向5运动
#define CONTROL_COMMAND_ID_LEFT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_6_MOTION 36            // （工具坐标系）主臂末端姿态方向6运动
#define CONTROL_COMMAND_ID_LEFT_ARM_STOP_MOTION 37                                  // 主臂停止运动

// 地面站->主控->从臂
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_BASE_DIRECTION_1_MOTION 1                     // 从臂基座方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_BASE_DIRECTION_2_MOTION 2                    // 从臂基座方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_SHOULDER_DIRECTION_1_MOTION 3                // 从臂肩部方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_SHOULDER_DIRECTION_2_MOTION 4                // 从臂肩部方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_ELBOW_DIRECTION_1_MOTION 5                   // 从臂肘部方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_ELBOW_DIRECTION_2_MOTION 6                   // 从臂肘部方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_1_DIRECTION_1_MOTION 7                 // 从臂手腕1方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_1_DIRECTION_2_MOTION 8                 // 从臂手腕1方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_2_DIRECTION_1_MOTION 9                 // 从臂手腕2方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_2_DIRECTION_2_MOTION 10                // 从臂手腕2方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_3_DIRECTION_1_MOTION 11                // 从臂手腕3方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TO_WRIST_3_DIRECTION_2_MOTION 12                // 从臂手腕3方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION 13     // （基座坐标系）从臂末端位移方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION 14     // （基座坐标系）从臂末端位移方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION 15     // （基座坐标系）从臂末端位移方向3运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION 16     // （基座坐标系）从臂末端位移方向4运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION 17     // （基座坐标系）从臂末端位移方向5运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION 18     // （基座坐标系）从臂末端位移方向6运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_1_MOTION 19            // （基座坐标系）从臂末端姿态方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_2_MOTION 20            // （基座坐标系）从臂末端姿态方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_3_MOTION 21            // （基座坐标系）从臂末端姿态方向3运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_4_MOTION 22            // （基座坐标系）从臂末端姿态方向4运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_5_MOTION 23            // （基座坐标系）从臂末端姿态方向5运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_BASE_COOR_END_POINT_POSE_DIRECTION_6_MOTION 24            // （基座坐标系）从臂末端姿态方向6运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_1_MOTION 25     // （工具坐标系）从臂末端位移方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_2_MOTION 26     // （工具坐标系）从臂末端位移方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_3_MOTION 27     // （工具坐标系）从臂末端位移方向3运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_4_MOTION 28     // （工具坐标系）从臂末端位移方向4运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_5_MOTION 29     // （工具坐标系）从臂末端位移方向5运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_TRANSLATION_DIRECTION_6_MOTION 30     // （工具坐标系）从臂末端位移方向6运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_1_MOTION 31            // （工具坐标系）从臂末端姿态方向1运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_2_MOTION 32            // （工具坐标系）从臂末端姿态方向2运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_3_MOTION 33            // （工具坐标系）从臂末端姿态方向3运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_4_MOTION 34            // （工具坐标系）从臂末端姿态方向4运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_5_MOTION 35            // （工具坐标系）从臂末端姿态方向5运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_TOOL_COOR_END_POINT_POSE_DIRECTION_6_MOTION 36            // （工具坐标系）从臂末端姿态方向6运动
#define CONTROL_COMMAND_ID_RIGHT_ARM_STOP_MOTION 37                                  // 从臂停止运动

// 地面站->主控->垂直滑台
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_POSITIVE_DIRECTION_MOVE 1        // 滑台1向正方向移动
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_NEGATIVE_DIRECTION_MOVE 2        // 滑台1向负方向移动
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_MOVE_TO_POSITION 3               // 滑台1向指定方向移动
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_GO_HOME 4                        // 滑台1归零
#define CONTROL_COMMAND_ID_VERTICAL_SLIDER_STOP_MOTION 5                    // 滑台1停止运动

// 地面站->主控->水平滑台
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_POSITIVE_DIRECTION_MOVE 1      // 滑台2向正方向移动
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_NEGATIVE_DIRECTION_MOVE 2      // 滑台2向负方向移动
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_MOVE_TO_POSITION 3             // 滑台2向指定方向移动
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_GO_HOME 4                      // 滑台2归零
#define CONTROL_COMMAND_ID_HORIZONTAL_SLIDER_STOP_MOTION 5                  // 滑台2停止运动

// 地面站->主控->电源板
/*
#define CONTROL_COMMAND_ID_POWER_MANAGER_SYSTEM_POWER_ON 1                 // （电源管理）系统开机
#define CONTROL_COMMAND_ID_POWER_MANAGER_SYSTEM_POWER_OFF 2                // （电源管理）系统关机
#define CONTROL_COMMAND_ID_POWER_MANAGER_SYSTEM_EMERGENCY_STOP 3           // （电源管理）系统急停
#define CONTROL_COMMAND_ID_POWER_MANAGER_SYSTEM_RELEASE_EMERGENCY_STOP 4   // （电源管理）系统解除急停
*/

// -----------------
// 主控发给设备的控制消息序号定义
// -----------------
// 主控到滑台
#define COMMAND_CONTROL_SEQ_HORIZONTAL_SLIDER_MOVE 3
#define COMMAND_CONTROL_SEQ_HORIZONTAL_SLIDER_GO_HOME 4
#define COMMAND_CONTROL_SEQ_VERTICAL_SLIDER_MOVE 3
#define COMMAND_CONTROL_SEQ_VERTICAL_SLIDER_GO_HOME 4

// 主控到末端接线工具
#define CMD_MSG_CLAMP_WIRE_TOOL_SUDDEN_STOP_SEQ 1                   // （接线）急停
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTART_SEQ 2          // （接线）拧断开始
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENSTOP_SEQ 3           // （接线）拧断停止
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENBACK_SEQ 4           // （接线）拧断反转
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_TIGHTENINIT_SEQ 5           // （接线）拧断初始
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_CLIPUNLOCK_SEQ 6            // （接线）线夹解锁
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_CLIPLOCK_SEQ 7              // （接线）线夹锁定
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVELOCK_SEQ 8            // （接线）套筒锁定
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVEUNLOCK_SEQ 9          // （接线）套筒解锁
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_SLEEVESTOP_SEQ 10           // （接线）套筒停止
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINELOCK_SEQ 11       // （接线）支线夹紧
#define CMD_MSG_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINEUNLOCK_SEQ 12     // （接线）支线解锁

// 主控到末端剪线工具
#define CMD_MSG_CUT_WIRE_TOOL_CUT_START 1               // （断线）剪线开始
#define CMD_MSG_CUT_WIRE_TOOL_CUT_STOP 2                // （断线）剪线停止
#define CMD_MSG_CUT_WIRE_TOOL_CUT_RESET 3               // （断线）剪线复位

// 主控到末端夹爪工具
#define CMD_MSG_CLAW_TOOL_CLOSE_GRIPPER 1            // （夹爪）夹爪闭合
#define CMD_MSG_CLAW_TOOL_RESET_GRIPPER 2            // （夹爪）夹爪复位

// 主控到末端剥线工具
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_STOP_SEQ 2        // 停止
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_ONEKEY_SEQ 3      // 一键剥皮
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_INIT_SEQ 4        // 初始化
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKLEFT_SEQ 12     // 剥皮推杆向左
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKRIGHT_SEQ 13   // 剥皮推杆向右
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKUP_SEQ 14     // 剥皮推杆向上
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_LINKDOWM_SEQ 15   // 剥皮推杆向下
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_ROTATE_SEQ 16     // 剥皮正转
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_BACKROTATE_SEQ 17 // 剥皮反转
#define CMD_MSG_CLAMP_WIRE_TOOL_STRIP_UPWARD_SEQ 18     // 剥皮开口向上

// -----------------
// 通用数字量信息（遥信）
// -----------------
// 滑台遥信消息定义
#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_REACH_FLAG_SEQ 1
#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_REACH_FLAG_TYPE "uint"
#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_REACH_FLAG_NAME "VerReachFlag"

#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_GO_HOME_FLAG_SEQ 2
#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_GO_HOME_FLAG_TYPE "uint"
#define GENERAL_DIGITAL_MSG_VERTICAL_SLIDER_GO_HOME_FLAG_NAME "verHommingFlag"

#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_REACH_FLAG_SEQ 24
#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_REACH_FLAG_TYPE "uint"
#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_REACH_FLAG_NAME "HorReachFlag"

#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_GO_HOME_FLAG_SEQ 25
#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_GO_HOME_FLAG_TYPE "uint"
#define GENERAL_DIGITAL_MSG_HORIZONTAL_SLIDER_GO_HOME_FLAG_NAME "HorHommingFlag"

// 末端接线工具
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG 1            // （接线）通信连接状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_ACTION_UNEXECUTED_FLAG 2                // （接线）拧断动作未执行标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_IN_TIGHTENING_FLAG 3                    // （接线）拧断过程中标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGNTHEN_FAILED_FLAG 4                  // （接线）拧断失败标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGHTNEN_SUCCESS_FLAG 5                 // （接线）拧断成功标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_CLIP_LOCK_STATE_FLAG 7                  // （接线）线夹锁定状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_IN_SLEEVE_LOCKING_FLAG 8                // （接线）套筒锁定动作过程中标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_SLEEVE_LOCK_INITIAL_POS_FLAG 9          // （接线）套筒锁定初始位置标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_SLEEVE_LOCK_STATE_FLAG 11               // （接线）套筒锁定标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_IN_BRANCHLINE_LOCK_FLAG 12              // （接线）支线夹紧动作中标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_BRANCHLINE_LOCK_INITIAL_POS_FLAG 13     // （接线）支线夹紧初始位置标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_BRANCHLINE_LOCK_STATE_FLAG 14           // （接线）支线锁定标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_BRANCHLINE_REACH_FLAG 15                // （接线）支线到位状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGHTEN_MOTOR_STATE 16                  // （接线）拧断电机状态遥信点
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_TIGHTEN_MOTOR_INIT 17                   // （接线）拧断电机复位遥信点
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_CLIP_LOCK_STATE 18                      // （接线）线夹锁定状态遥信点
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_SLEEVE_LOCK_STATE 19                    // （接线）套筒锁定状态遥信点
#define GENERAL_DIGITAL_MSG_SEQ_CLAMP_WIRE_TOOL_IN_BRANCHLINE_LOCK_STATE 20             // （接线）支线夹紧状态遥信点

// 末端断线工具
#define GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_COMMUNICATION_CONNECT_FLAG 1     // （断线）通信连接状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_CUT_STATE_FLAG 2                 // （断线）断线状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_INITIAL_STATE_FLAG 3             // （断线）断线电机初始状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_CUT_STATE 8                      // （断线）断线状态遥信点
#define GENERAL_DIGITAL_MSG_SEQ_CUT_WIRE_TOOL_CUT_MOTOR_INIT_STATE 9           // （断线）剪线电机初始状态遥信点

// 末端夹爪工具
#define GENERAL_DIGITAL_MSG_SEQ_CLAW_TOOL_COMMUNICATION_CONNECT_FLAG 1    // （夹爪）通信连接状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAW_TOOL_GRIPPER_STATE_FLAG 2            // （夹爪）夹爪状态标志
#define GENERAL_DIGITAL_MSG_SEQ_CLAW_TOOL_GRIPPER_STATE 3                 // （夹爪）夹爪状态遥信点

// 末端剥线工具
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_COMMUNICATION_CONNECT_FLAG 1  // （剥线）通信连接状态标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_STOP_FLAG 3                   // （剥线）停止标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ONEKEY_FLAG 4                 // （剥线）一键剥皮标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_INIT_FLAG 5                   // （剥线）初始化标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_LINKLEFT_FLAG 13              // （剥线）推杆左标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_LINKRIGHT_FLAG 14             // （剥线）推杆右标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_LINKUP_FLAG 15                // （剥线）推杆上标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_LINKDOWN_FLAG 16              // （剥线）推杆下标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ROTATE_FLAG 17                // （剥线）剥皮正转标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_BACKROTATE_FLAG 18            // （剥线）剥皮反转标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_UPWARD_FLAG 19                // （剥线）开口向上标志
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_EXECUTED_ACTION_FLAG 27       // （剥线）剥线执行动作遥信点
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_ACTION_STATE_FLAG 28          // （剥线）剥线动作状态遥信点
#define GENERAL_DIGITAL_MSG_SEQ_STRIP_TOOL_MAXIMUM_SEQ 29                // （剥线）剥线最大通信点

// 剥线消息 执行动作MotionType
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_NO_MOTION 0           // 无动作
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_STOP 1                // 停止
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_ONEKEY 2              // 一键剥皮
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_INIT 3                // 初始化
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_MAINLINE_CLAMP 4      // 主线夹紧
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_CUTTER_CLAMP 5        // 刀具夹紧
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_ROTATE_CUT 6          // 旋转剥皮
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_CUT_INSULATION 7      // 切断绝缘皮
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_UNLOCK 8              // 解锁
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_OPEN_CUTTER 9         // 刀具打开
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_CLOSE_CUTTER 10       // 刀具关闭
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKLEFT 11           // 推杆左
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKRIGHT 12          // 推杆右
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKUP 13             // 推杆上
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_LINKDOWN 14           // 推杆下
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_ROTATE 15             // 剥皮正转
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_BACKROTATE 16         // 剥皮反转
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_TYPE_UPWARD 17             // 开口向上

// 剥线消息 动作状态MotionStatus
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_NO_MOTION 0         // 无动作
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_COMPLETED 1         // 动作完成
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_IN_PROGRESS 2       // 动作进行中
#define GENERAL_DIGITAL_MSG_STRIP_TOOL_MOTION_STATUS_FAILED 3            // 动作失败

// -----------------
// 通用模拟量信息（遥测）
// -----------------
// 滑台遥测消息定义
#define GENERAL_ANALOG_MSG_VERTICAL_SLIDER_POS_SEQ 1
#define GENERAL_ANALOG_MSG_HORIZONTAL_SLIDER_POS_SEQ 2
// 机械臂遥测消息定义
#define GENERAL_ANALOG_MSG_l_shoulder_pan_joint 1
#define GENERAL_ANALOG_MSG_l_shoulder_lift_joint 2
#define GENERAL_ANALOG_MSG_l_elbow_joint 3
#define GENERAL_ANALOG_MSG_l_wrist_1_joint 4
#define GENERAL_ANALOG_MSG_l_wrist_2_joint 5
#define GENERAL_ANALOG_MSG_l_wrist_3_joint 6
#define GENERAL_ANALOG_MSG_l_shoulder_pan_joint_temp 7
#define GENERAL_ANALOG_MSG_l_shoulder_lift_joint_temp 8
#define GENERAL_ANALOG_MSG_l_elbow_joint_temp 9
#define GENERAL_ANALOG_MSG_l_wrist_1_joint_temp 10
#define GENERAL_ANALOG_MSG_l_wrist_2_joint_temp 11
#define GENERAL_ANALOG_MSG_l_wrist_3_joint_temp 12
#define GENERAL_ANALOG_MSG_r_shoulder_pan_joint 13
#define GENERAL_ANALOG_MSG_r_shoulder_lift_joint 14
#define GENERAL_ANALOG_MSG_r_elbow_joint 15
#define GENERAL_ANALOG_MSG_r_wrist_1_joint 16
#define GENERAL_ANALOG_MSG_r_wrist_2_joint 17
#define GENERAL_ANALOG_MSG_r_wrist_3_joint 18
#define GENERAL_ANALOG_MSG_r_shoulder_pan_joint_temp 19
#define GENERAL_ANALOG_MSG_r_shoulder_lift_joint_temp 20
#define GENERAL_ANALOG_MSG_r_elbow_joint_temp 21
#define GENERAL_ANALOG_MSG_r_wrist_1_joint_temp 22
#define GENERAL_ANALOG_MSG_r_wrist_2_joint_temp 23
#define GENERAL_ANALOG_MSG_r_wrist_3_joint_temp 24

// -----------------
// 等待回复时间
// -----------------
// 机械臂
#define DEFAULT_ARM_RESPONSE_WAIT_TIME_SECONDS 60

// 段等待
#define DEFAULT_SEGMENT_RESPONSE_WAIT_TIME_SECONDS 10

// 滑台：运动速度慢，等待时间设长一点
#define DEFAULT_SLIDER_RESPONSE_WAIT_TIME_SECONDS 60

// 滑台位置确认按钮
#define DEFAULT_SLIDER_POS_CONFIRM_RESPONSE_WAIT_TIME_SECONDS 300

// 人工确认：不设置超时
#define DEFAULT_MANUALLY_CONFIRM_RESPONSE_WAIT_TIME_SECONDS 3600*24*365

// 激光雷达数据
#define DEFAULT_LIDAR_RESPONSE_WAIT_TIME_SECONDS 300

// 末端接线工具
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_TIGHTEN_START_WAIT_TIME_SECONDS 4*60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_TIGHTEN_INIT_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_CLIP_LOCK_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_CLIP_UNLOCK_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_SLEEVE_LOCK_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_SLEEVE_UNLOCK_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINE_LOCK_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAMP_WIRE_TOOL_CONNECT_BRANCHLINE_UNLOCK_WAIT_TIME_SECONDS 60

// 末端剪线工具
#define DEFAULT_CUT_WIRE_TOOL_CUT_START_WAIT_TIME_SECONDS 5*60
#define DEFAULT_CUT_WIRE_TOOL_CUT_INIT_WAIT_TIME_SECONDS 2*60

// 末端夹爪工具
#define DEFAULT_CLAW_TOOL_GRIPPER_OPEN_WAIT_TIME_SECONDS 60
#define DEFAULT_CLAW_TOOL_GRIPPER_CLOSE_WAIT_TIME_SECONDS 60

// 末端剥线工具
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_ONEKEY_WAIT_TIME_SECONDS 6*60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_INIT_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_ROTATE_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_BACK_ROTATE_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_UPWARD_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_LEFT_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_RIGHT_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_UP_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_LINK_DOWN_WAIT_TIME_SECONDS 60
#define DEFAULT_STRIP_WIRE_TOOL_STRIP_STOP_WAIT_TIME_SECONDS 60

// -----------------
// 结点名和类型定义
// -----------------
// 主控结点
#define NODE_NAME_CONTROLLER "MainControl"
#define NODE_TYPE_CONTROLLER "MainControl"

// 地面站（远程控指端）
#ifndef SEPERATE_TESTING
#define NODE_NAME_GROUND_STATION "RemoteControl"
#define NODE_TYPE_GROUND_STATION "RemoteControl"
#else
// 测试用
#define NODE_NAME_GROUND_STATION "RemoteControlTest"
#define NODE_TYPE_GROUND_STATION "RemoteControlTest"
#endif

// 机械臂结点NEW
#define NODE_NAME_ROBOT_Script "URScriptControl"
#define NODE_TYPE_ROBOT_Script "URScriptControl"

// 滑台结点
#define NODE_NAME_SLIDER "Slide"
#define NODE_TYPE_SLIDER "Slide"

// 末端接线工具结点
#define NODE_NAME_CLAMP_WIRE_TOOL "ClampWireTool"
#define NODE_TYPE_CLAMP_WIRE_TOOL "ClampWireTool"

// 末端剪线工具结点
#define NODE_NAME_CUT_WIRE_TOOL "CutWireTool"
#define NODE_TYPE_CUT_WIRE_TOOL "CutWireTool"

// 末端夹爪工具结点
#define NODE_NAME_CLAW_TOOL "ClawTool"
#define NODE_TYPE_CLAW_TOOL "ClawTool"

// 末端剥线工具结点
#define NODE_NAME_STRIP_WIRE_TOOL "StripWireTool"
#define NODE_TYPE_STRIP_WIRE_TOOL "StripWireTool"

// 电源管理结点
#define NODE_NAME_POWER_MANAGER  "PwrMng"
#define NODE_TYPE_POWER_MANAGER  "PwrMng"

//机械臂状态节点名与节点类型
#define NODE_NAME_ARM_status_yx "armsInfoDigital"
#define NODE_TYPE_ARM_status_yx "arm"

// 机械臂结点
#define NODE_NAME_ARM_INFO_ANALOG "armsInfoAnalog"
#define NODE_TYPE_ARM "arm"

// 左机械臂状态
#define L_ARM_CONNECTED  "l_arm_connected"                   //主臂连接
#define L_ARM_POWER_ON  "l_arm_power_on"                        //主臂上电 
#define L_ARM_ENABLED   "l_arm_enabled"                         //主臂启用
#define L_ARM_EMERGENCY_STOPPED   "l_arm_emergency_stopped"     //主臂紧急停止
#define L_ARM_PROTECTIVE_STOPPED   "l_arm_protective_stopped"   //主臂保护性停止
#define L_ARM_PROGRAM_RUNNING   "l_arm_program_running"         //主臂程序运行
#define L_ARM_PROGRAM_PAUSED   "l_arm_program_paused"           //主臂程序暂停

// 右机械臂状态
#define R_ARM_CONNECTED  "r_arm_connected"                  //从臂连接
#define R_ARM_POWER_ON   "r_arm_power_on"                       //从臂上电
#define R_ARM_ENABLED   "r_arm_enabled"                         //从臂启用
#define R_ARM_EMERGENCY_STOPPED   "r_arm_emergency_stopped"     //从臂紧急停止
#define R_ARM_PROTECTIVE_STOPPED   "r_arm_protective_stopped"   //从臂保护性停止
#define R_ARM_PROGRAM_RUNNING   "r_arm_program_running"         //从臂程序运行
#define R_ARM_PROGRAM_PAUSED   "r_arm_program_paused"           //从臂程序暂停

// -----------------
// 消息类型定义
// -----------------
#define MSG_TYPE_CMD_MSG "public_pkg/cmd_msg"
#define MSG_TYPE_CMD_RSP_MSG "public_pkg/cmd_rsp_msg"
#define MSG_TYPE_STATUS_DIGITAL_MSG "public_pkg/status_digital_msg"
#define MSG_TYPE_STATUS_ANALOG_MSG "public_pkg/status_analog_msg"
#define MSG_TYPE_HEART_BAET_MSG "public_pkg/heart_beat_msg"

// -----------------
// 坐标偏移操作类型定义
// -----------------
#define OFFSET_FEATURE_BASE_COORDINATE 0
#define OFFSET_FEATURE_TOOL_COORDINATE 1

#define OFFSET_DIRECTION_X 1
#define OFFSET_DIRECTION_Y 2
#define OFFSET_DIRECTION_Z 3
#define OFFSET_DIRECTION_RX 4
#define OFFSET_DIRECTION_RY 5
#define OFFSET_DIRECTION_RZ 6

//机械臂ID
#define ROBOT_ARM_LEFT_ARM 1
#define ROBOT_ARM_RIGHT_ARM 2

// 地面站--主控--电源控制结点
#define CMD_CONTROL_POWER_MANAGER_ARM_POWER_ON 1                         // （电源管理）机械臂上电
#define CMD_CONTROL_POWER_MANAGER_ARM_BRAKE_RELEASE 2                    // （电源管理）打开电机抱闸
#define CMD_CONTROL_POWER_MANAGER_ARM_POWER_OFF 4                        // （电源管理）机械臂断电

// 电源管理-机械臂
#define POWER_MANAGER_ARM_POWER_ON 1                         // （电源管理）机械臂开机
#define POWER_MANAGER_ARM_POWER_OFF 2                        // （电源管理）机械臂关机
#define ARM_EMERGENCY_STOP  3                                // （电源管理）机械臂急停
#define ARM_EMERGENCY_STOP_RELIEVE  4                        //（电源管理）机械臂解除急停
#define UNLOCK_ARM_PROTECTIVE_STOP  5                        // 解除保护性停止命令到达

#define ARM_POWER_ON_TOPIC "RobotPowerOn"                         // （电源管理）机械臂开机
#define ARM_POWER_OFF_TOPIC "RobotPowerOff"                        // （电源管理）机械臂关机
#define ARM_EMERGENCY_STOP_TOPIC  "RobotEmergencyStop"                               // （电源管理）机械臂急停
#define ARM_EMERGENCY_STOP_RELIEVE_TOPIC  "RobotEmergencyStopRelieve"                        //（电源管理）机械臂解除急停



// 机械臂电源控制指令
#define ARM_POWER_ON_COMMAND    "power on"                                    //机械臂上电指令
#define ARM_POWER_OFF_COMMAND   "power off"                                   //机械臂断电指令
#define ARM_UNLOCK_PROTECTIVE_STOP_COMMAND   "unlock protective stop"         //解除保护性停止指令
#define ARM_BRAKE_RELEASE_COMMAND    "brake release"                          //打开电机抱闸指令

//-------------------//
// 电源管理通信连接标志 //
//------------------//

// 主臂-机械臂状态
#define LEFT_ARM_CONNECTED 8//1             // （电源管理）左臂连接标志
#define LEFT_ARM_ENABLED   2                // 主臂启用-伺服打开
#define LEFT_ARM_POWER_ON   1//3            // 主臂上电
#define LEFT_ARM_ERROR   3               // 主臂报错
#define LEFT_ARM_EMERGENCY_STOPPED  4       // 主臂紧急停止
#define LEFT_ARM_PROTECTIVE_STOPPED   5     // 主臂保护性停止
#define LEFT_ARM_PROGRAM_RUNNING    6       // 主臂程序运行
#define LEFT_ARM_PROGRAM_PAUSED    7        // 主臂程序暂停

// 从臂-机械臂状态
#define RIGHT_ARM_CONNECTED 108//8                        // （电源管理）右臂连接标志
#define RIGHT_ARM_ENABLED   102//9                        // 从臂启用
#define RIGHT_ARM_POWER_ON   101//10                      // 从臂上电
#define RIGHT_ARM_ERROR   103                             // 从臂报错
#define RIGHT_ARM_EMERGENCY_STOPPED  104//11              // 从臂紧急停止
#define RIGHT_ARM_PROTECTIVE_STOPPED   105//12            // 从臂保护性停止
#define RIGHT_ARM_PROGRAM_RUNNING    106//13              // 从臂程序运行
#define RIGHT_ARM_PROGRAM_PAUSED    107//14               // 从臂程序暂停

#define WAIT_POWER_ON_TIME_OUT   2                  // 机械臂上电超时
#define NOT_POWER_ON   0                            // 机械臂未上电
#define WAIT_LEFT_POWER_ON_TIME_FLAG   15                 // 机械臂上电超时
#define WAIT_RIGHT_POWER_ON_TIME_FLAG  15                 // 机械臂上电超时

#define WAIT_LIDAR_POINT_CONFIRM 15                 //雷达选点完成确认
#define NODE_NAME_LIDAR_POING_CONFIRM "VisionNode"
#define NODE_TYPE_LIDAR_POING_CONFIRM "VisionNode"

//地面站通知报警信息
#define LIDARCHOOSEFAILURE "警告-选点错误，请重新点选！-继续运行|转人工控制"
#define LIDARCHOOSEFAILURETOMANUAL "警告-自动选点找不到点，切换到人工模式"
#define CLAMP_TIGHTEN_START "接线拧断开始动作"
#define CLAMP_TIGHTEN_TIMEOUT "警告-接线拧断超时，请检查！-继续运行|转人工控制"
#define CLAMP_TIGHTENBACK_START "接线拧断反转开始动作"
#define CLAMP_TIGHTENINIT_START "接线拧断初始化开始动作"
#define CLAMP_TIGHTENINIT_TIMEOUT "警告-接线拧断初始化超时，请检查！-继续运行|转人工控制"
#define CLIP_LOCK_START "接线线夹锁定开始动作"
#define CLIP_LOCK_TIMEOUT "警告-接线线夹锁定超时，请检查！-继续运行|转人工控制"
#define CLIP_UNLOCK_START "接线线夹解锁开始动作"
#define CLIP_UNLOCK_TIMEOUT "警告-接线线夹解锁超时，请检查！-继续运行|转人工控制"
#define SLEEVE_LOCK_START "接线套筒锁定开始动作"
#define SLEEVE_LOCK_TIMEOUT "警告-接线套筒锁定超时，请检查！-继续运行|转人工控制"
#define SLEEVE_UNLOCK_START "接线套筒解锁开始动作"
#define SLEEVE_UNLOCK_TIMEOUT "警告-接线套筒解锁超时，请检查！-继续运行|转人工控制"
#define SLEEVE_STOP_START "接线套筒停止指令开始动作"
#define BRANCH_LOCK_START "接线支线夹紧开始动作"
#define BRANCH_LOCK_TIMEOUT "警告-接线支线夹紧超时，请检查！-继续运行|转人工控制"
#define BRANCH_UNLOCK_START "接线支线松开开始动作"
#define BRANCH_UNLOCK_TIMEOUT "警告-接线支线松开超时，请检查！-继续运行|转人工控制"
#define CUT_WIRE_START "剪线开始动作"
#define CUT_WIRE_TIMEOUT "警告-剪线超时，请检查！-继续运行|转人工控制"
#define CUT_INIT_START "剪线初始化开始动作"
#define CUT_INIT_TIMEOUT "警告-剪线初始化超时，请检查！-继续运行|转人工控制"
#define LEFTARM_GETPP_TIMEOUT "警告-读取主臂PP位姿数据超时，请检查！-继续运行|转人工控制"
#define RIGHTARM_GETPP_TIMEOUT "警告-读取从臂PP位姿数据超时，请检查！-继续运行|转人工控制"
#define ARM_GETJOINT_TIMEOUT "警告-读取机械臂JOINT数据超时，请检查！-继续运行|转人工控制"
#define GRIPPER_OPEN_START "夹爪打开开始动作"
#define GRIPPER_OPEN_TIMEOUT "警告-夹爪打开超时，请检查！-继续运行|转人工控制"
#define GRIPPER_CLOSE_START "夹爪关闭开始动作"
#define GRIPPER_CLOSE_TIMEOUT "警告-夹爪关闭超时，请检查！-继续运行|转人工控制"
#define LEFTARM_ARRIVE_TIMEOUT "警告-主臂到达位置超时"
#define RIGHTARM_ARRIVE_TIMEOUT "警告-从臂到达位置超时"
#define VSLIDER_MOVE_START "垂直滑台移动开始动作"
#define VSLIDER_MOVE_TIMEOUT "警告-垂直滑台移动超时，请检查！-继续运行|转人工控制"
#define HSLIDER_MOVE_START "水平滑台移动开始动作"
#define HSLIDER_MOVE_TIMEOUT "警告-水平滑台移动超时，请检查！-继续运行|转人工控制"
#define VSLIDER_HOME_START "垂直滑台回原点开始动作"
#define VSLIDER_HOME_TIMEOUT "警告-垂直滑台回原点超时，请检查！-继续运行|转人工控制"
#define HSLIDER_HOME_START "水平滑台回原点开始动作"
#define HSLIDER_HOME_TIMEOUT "警告-水平滑台回原点超时，请检查！-继续运行|转人工控制"
#define SLIDER_RECORDPOS_TIMEOUT "警告-滑台记录位置超时，请重试！-继续运行|转人工控制"

#define STRIP_ONEKEY_START "一键剥皮开始动作"
#define STRIP_ONEKEY_TIMEOUT "警告-一键剥皮超时，请检查！-继续运行|转人工控制"
#define STRIP_INIT_START "剥皮初始化开始动作"
#define STRIP_INIT_TIMEOUT "警告-剥皮初始化超时，请检查！-继续运行|转人工控制"
#define STRIP_ROTATE_START "剥皮正转开始动作"
#define STRIP_ROTATE_TIMEOUT "警告-剥皮正转超时，请检查！-继续运行|转人工控制"
#define STRIP_BACKROTATE_START "剥皮反转开始动作"
#define STRIP_BACKROTATE_TIMEOUT "警告-剥皮反转超时，请检查！-继续运行|转人工控制"
#define STRIP_UPWARD_START "剥皮开口向上开始动作"
#define STRIP_UPWARD_TIMEOUT "警告-剥皮开口向上超时，请检查！-继续运行|转人工控制"
#define STRIP_LINKLEFT_START "剥皮推杆向左开始动作"
#define STRIP_LINKLEFT_TIMEOUT "警告-剥皮推杆向左超时，请检查！-继续运行|转人工控制"
#define STRIP_LINKRIGHT_START "剥皮推杆向右开始动作"
#define STRIP_LINKRIGHT_TIMEOUT "警告-剥皮推杆向右超时，请检查！-继续运行|转人工控制"
#define STRIP_LINKUP_START "剥皮推杆向上开始动作"
#define STRIP_LINKUP_TIMEOUT "警告-剥皮推杆向上超时，请检查！-继续运行|转人工控制"
#define STRIP_LINKDOWN_START "剥皮推杆向下开始动作"
#define STRIP_LINKDOWN_TIMEOUT "警告-剥皮推杆向下超时，请检查！-继续运行|转人工控制"
#define STRIP_STOP_START "剥皮停止指令开始动作"
#define STRIP_STOP_TIMEOUT "警告-剥皮停止超时，请检查！-继续运行|转人工控制"
#define IF_CONTINUE_JOB "警告-不在安全位置，是否继续作业？-继续运行|停止作业"



//垂直雷达相对主臂
#define Lidar1ToMa1X 0.319
#define Lidar1ToMa1Y -0.209
#define Lidar1ToMa1Z -0.502
//垂直雷达相对从臂
#define Lidar1ToMa2X 0.419
#define Lidar1ToMa2Y 0.241
#define Lidar1ToMa2Z -0.502

//水平雷达相对主臂
#define Lidar2ToMa1X 0.297
#define Lidar2ToMa1Y -0.233
#define Lidar2ToMa1Z -0.129
//水平雷达相对从臂
#define Lidar2ToMa2X 0.397
#define Lidar2ToMa2Y 0.217
#define Lidar2ToMa2Z -0.129


/*//垂直雷达相对主臂
#define Lidar1ToMa1X 0.3075
#define Lidar1ToMa1Y -0.23996
#define Lidar1ToMa1Z -0.49963
//垂直雷达相对从臂
#define Lidar1ToMa2X 0.4075
#define Lidar1ToMa2Y 0.21004
#define Lidar1ToMa2Z -0.49963

//水平雷达相对主臂
#define Lidar2ToMa1X 0.30493
#define Lidar2ToMa1Y -0.24223
#define Lidar2ToMa1Z -0.0687
//水平雷达相对从臂
#define Lidar2ToMa2X 0.40493
#define Lidar2ToMa2Y 0.20777
#define Lidar2ToMa2Z -0.0687*/

// time
#define TM_YEAR 0 
#define TM_MON 1  
#define TM_MDAY 2 
#define TM_HOUR 3 
#define TM_MIN 4 
#define TM_SEC 5 

// log type/object
#define LOG_DEBUG 0 
#define LOG_INFO 1  
#define LOG_WARN 2 
#define LOG_ERROR 3 
#define LOG_FATAL 4 
#define LOG_REMOTE_CONTROL 5
#define LOG_MAIN_CONTROL 6

#define FP  "./log/" 
#define FILE_DEBUG  "./log/debug/" 
#define FILE_INFO  "./log/info/"  
#define FILE_WARN  "./log/warn/" 
#define FILE_ERROR  "./log/error/" 
#define FILE_FATAL "./log/fatal/"
#define FILE_REMOTE_CONTROL "./log/remote_control/"
#define FILE_MAIN_CONTROL "./log/main_control/"

#define BEGIN_DEBUG "debug_"
#define BEGIN_INFO "info_"
#define BEGIN_WARN "warn_"
#define BEGIN_ERROR "error_"
#define BEGIN_FATAL "fatal_"
#define BEGIN_REMOTE "remote_"
#define BEGIN_MAIN "main_"
#define END_NAME "_log.txt"

#define MSG_BEGIN_DEBUG "[ DEBUG ] ["
#define MSG_BEGIN_INFO "[ INFO ] ["
#define MSG_BEGIN_WARN "[ WARN ] ["
#define MSG_BEGIN_ERROR "[ ERROR ] ["
#define MSG_BEGIN_FATAL "[ FATAL ] ["


#endif // CONFIG_H
