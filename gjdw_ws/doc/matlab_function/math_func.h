#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include "Eigen/Eigen"

#include <vector>

// 旋转指定的轴向量
//  @dist: 轴向量距离原点的距离
//  @axis: 坐标轴
//  @rpy: 以欧拉角表示的旋转变换
Eigen::Vector3d tcp_move(const int dist, const std::string& axis, const Eigen::Vector3d& rpy);

// 计算指定轴的偏移量
//  @tcp: 根据工具坐标系待改变的路点
//  @value: 偏移值
//  @direction: 指定轴
//  @is_degree_value: 指明@value是否为角度值
std::vector<double> calculate_offset(Eigen::Vector3d tcp, double value,
                                     const int direction, const bool is_degree_value);

#endif // MATH_FUNC_H