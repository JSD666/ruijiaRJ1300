#ifndef LIDAR_FUNC_H
#define LIDAR_FUNC_H

#include <Eigen/Eigen>
#include <string>

#include "cmd_line_func.h"
#include "config.h"

#define LIDAR_TYPE_VERTICAL_LIDAR 1
#define LIDAR_TYPE_HORIZONTAL_LIDAR 2



#define LIDARTOMA1_ID 1
#define LIDARTOMA2_ID 2
#define LIDARTOMA3_ID 3



// 将三维点从激光坐标系变换到机械臂坐标系
//  @point: 三维点
//  @dist: 滑台移动距离
//  @lidar_id: 激光雷达类型ID
//  @arm_id: 机械臂类型ID
Eigen::Vector3d lidar_to_arm_xyz(const Eigen::Vector3d& point, const double& dist, 
                                const int& lidar_id, const int& arm_id);


// 计算两点表示的向量的旋转姿态，以RPY表示
//  @point1: 三维点
//  @point2: 三维点
//  @lidartoma_id: 坐标系类型
Eigen::Vector3d lidar_to_arm_rpy(const Eigen::Vector3d& toolsX,const int lidartoma_id);

Eigen::Vector3d lidar_to_arm_rpyNew(const Eigen::Vector3d& toolsX,const int& lidartoma_id,const int& posetype); 

std::vector<double>  calculate_offset_tool_coorNew(const std::vector<double>& tcp,double value,const int direction);

CmdLineFuncRsp lidartoma1_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         int MaID,int subId);
CmdLineFuncRsp liar_variable_output(SegmentScheduler& scheduler,const DoubleValueArray& pp,
                                    const std::string& var_define);

CmdLineFuncRsp lidartoma2_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         int MaID,int LineType);

CmdLineFuncRsp lidartoma3_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         double sliderpos_value,int MaID,int LineType);

bool DoubleValueArray_IsNumer(const DoubleValueArray& pp);

#endif // LIDAR_FUNC_H