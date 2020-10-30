#include "math_func.h"

using namespace Eigen;
using namespace std;

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

// 计算指定轴的偏移量
//  @tcp: 根据工具坐标系待改变的路点
//  @value: 偏移值
//  @direction: 指定轴
//  @is_degree_value: 指明@value是否为角度值
std::vector<double> calculate_offset(Eigen::Vector3d tcp, double value,
                                     const int direction, const bool is_degree_value)
{
    const double x = tcp[0], y = tcp[1], z = tcp[2];
    double psai = tcp[3], theta = tcp[4], phi = tcp[5];

    // 转成欧拉角
    psai = psai * 180 / M_PI;
    theta = theta * 180 / M_PI;
    phi = phi * 180 / M_PI;

    switch (direction)
    {
    case OFFSET_DIRECTION_X:
        tcp = Vector3d(value, 0, 0);
        break;
    
    case OFFSET_DIRECTION_Y:
        tcp = Vector3d(0, value, 0);
        break;

    case OFFSET_DIRECTION_Z:
        tcp = Vector3d(0, 0, value);
        break;

    case OFFSET_DIRECTION_RX:
        if (is_degree_value)
        {
            value = value * M_PI / 180;
        }
        tcp = Vector3d(value, 0, 0);
        break;

    case OFFSET_DIRECTION_RY:
        if (is_degree_value)
        {
            value = value * M_PI / 180;
        }
        tcp = Vector3d(0, value, 0);
        break;

    case OFFSET_DIRECTION_RZ:
        if (is_degree_value)
        {
            value = value * M_PI / 180;
        }
        tcp = Vector3d(0, 0, value);
        break;

    default:
        break;
    }

    const Matrix3d R = 
        Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix() * 
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() * 
        Eigen::AngleAxisd(psai, Eigen::Vector3d::UnitX()).matrix();

    const Vector3d d = R * tcp;

    if (direction == OFFSET_DIRECTION_X || direction == OFFSET_DIRECTION_Y
        || direction == OFFSET_DIRECTION_Z)
    {
        return vector<double>{d[0], d[1], d[2], psai, theta, phi};
    }
    else
    {
        return vector<double>{x, y, z, d[0], d[1], d[2]};
    }
}

// 旋转指定的轴向量
//  @dist: 轴向量距离原点的距离
//  @axis: 坐标轴
//  @rpy: 以欧拉角表示的旋转变换
Eigen::Vector3d tcp_move(const int dist, const std::string& axis, const Eigen::Vector3d& rpy)
{
    const Matrix3d R = 
        Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()).matrix() * 
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()).matrix() * 
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()).matrix();

    Vector3d tcp;
    if (axis == "-z")
    {
        tcp = Vector3d(0, 0, -dist);
    }
    else if (axis == "-y")
    {
        tcp = Vector3d(0, -dist, 0);
    }
    else if (axis == "-x")
    {
        tcp = Vector3d(-dist, 0, 0);
    }
    return R * tcp;
}
