#include "math_func.h"
#include "config.h"

#include <iostream>
#include <exception>

using namespace std;
using namespace Eigen;

// 计算指定轴的偏移量
//  @tcp: 根据工具坐标系待改变的路点
//  @value: 偏移值
//  @direction: 指定轴
//  @is_degree_value: 指明@value是否为角度值
//  参数值非法时，抛出std::invalid_argument异常
std::vector<double> calculate_offset_tool_coor(const std::vector<double>& tcp, double value,
                                               const int direction, const bool is_degree_value)
{
    if (tcp.size() < 6) throw std::invalid_argument("number of argument should be 6");

    const double x = tcp[0], y = tcp[1], z = tcp[2];
    double Rx = tcp[3], Ry = tcp[4], Rz = tcp[5];
    double Pcos = cos(value*M_PI/180);
    double Psin = sin(value*M_PI/180), Nsin = -1*Psin;
    
    Eigen::Matrix4d base4d_matrix ;

    cout << "Pcos:\n" << Pcos << endl;
    cout << "Psin:\n" << Psin << endl;


    // 转成欧拉角
    // psai = psai * 180 / M_PI;
    // theta = theta * 180 / M_PI;
    // phi = phi * 180 / M_PI;

    Vector3d t;

    switch (direction)
    {
        case OFFSET_DIRECTION_X:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,3) = value;

            break;
        
        case OFFSET_DIRECTION_Y:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(1,3) = value;
            break;

        case OFFSET_DIRECTION_Z:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(2,3) = value;
            break;

        case OFFSET_DIRECTION_RX:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(1,1) = Pcos;
            base4d_matrix(1,2) = Nsin;
            base4d_matrix(2,1) = Psin;
            base4d_matrix(2,2) = Pcos;
            break;

        case OFFSET_DIRECTION_RY:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,0) = Pcos;
            base4d_matrix(0,2) = Psin;
            base4d_matrix(2,0) = Nsin;
            base4d_matrix(2,2) = Pcos;
            break;

        case OFFSET_DIRECTION_RZ:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,0) = Pcos;
            base4d_matrix(0,1) = Nsin;
            base4d_matrix(1,0) = Psin;
            base4d_matrix(1,1) = Pcos;
            break;

        default:
            throw std::invalid_argument("invalid argument");
    }

     //求齐次变化矩阵
     double MO=sqrt(Rx*Rx+Ry*Ry+Rz*Rz);
     double newRx= Rx/MO;
     double newRy= Ry/MO;
     double newRz= Rz/MO;

     Eigen::AngleAxisd PP1(MO,Vector3d(newRx,newRy,newRz));
     
     Eigen::Matrix3d R3d_matrix = PP1.matrix();
     Eigen::Matrix4d R4d_matrix ;
     R4d_matrix.block<3,3>(0,0) = R3d_matrix.block<3,3>(0,0);

    R4d_matrix(3,0) = 0;
    R4d_matrix(3,1) = 0;
    R4d_matrix(3,2) = 0;
    R4d_matrix(3,3) = 1;
    R4d_matrix(0,3) = x;
    R4d_matrix(1,3) = y;
    R4d_matrix(2,3) = z;

    Eigen::Matrix4d last4d_matrix = R4d_matrix*base4d_matrix;
    Eigen::Matrix3d last3d_matrix = last4d_matrix.block<3,3>(0,0);
    
    cout << "R4d_matrix:\n" << R4d_matrix << endl;
    cout << "R4d_matrix:\n" << base4d_matrix << endl;    
    cout << "last4d_matrix:\n" << last4d_matrix << endl;
    cout << "last3d_matrix:\n" << last3d_matrix << endl;
    
    double angle1;
    Vector3d axis1;
    Vector3d rotvector;
    Eigen::AngleAxisd PP2=Eigen::AngleAxisd(last3d_matrix);    
   
    angle1=PP2.angle();
    axis1=PP2.axis().transpose();

    rotvector=angle1*axis1;

     


    // const Matrix3d R = 
    //     Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix() * 
    //     Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() * 
    //     Eigen::AngleAxisd(psai, Eigen::Vector3d::UnitX()).matrix();

    
    double d[3]={last4d_matrix(0,3),last4d_matrix(1,3),last4d_matrix(2,3)};


     if (direction == OFFSET_DIRECTION_X || direction == OFFSET_DIRECTION_Y
         || direction == OFFSET_DIRECTION_Z)
     {
         
         return vector<double>{d[0], d[1], d[2], Rx, Ry, Rz};
         
     }
     else
     {
         return vector<double>{x, y, z, rotvector(0), rotvector(1), rotvector(2)};
     }
}

// 计算指定轴的偏移量，失败返回false，成功返回true
//  @tcp: 根据工具坐标系待改变的路点
//  @value: 偏移值
//  @direction: 指定轴
std::vector<double> calculate_offset_base_coor(std::vector<double>& tcp, double value, const int direction)
{

    if (tcp.size() < 6) throw std::invalid_argument("number of argument should be 6");

    const double x = tcp[0], y = tcp[1], z = tcp[2];
    double Rx = tcp[3], Ry = tcp[4], Rz = tcp[5];
    double Pcos = cos(value*M_PI/180);
    double Psin = sin(value*M_PI/180), Nsin = -1*Psin;
    
    Eigen::Matrix4d base4d_matrix ;

    cout << "Pcos:\n" << Pcos << endl;
    cout << "Psin:\n" << Psin << endl;


    // 转成欧拉角
    // psai = psai * 180 / M_PI;
    // theta = theta * 180 / M_PI;
    // phi = phi * 180 / M_PI;

    Vector3d t;

    switch (direction)
    {
        case OFFSET_DIRECTION_X:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,3) = value;

            break;
        
        case OFFSET_DIRECTION_Y:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(1,3) = value;
            break;

        case OFFSET_DIRECTION_Z:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(2,3) = value;
            break;

        case OFFSET_DIRECTION_RX:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(1,1) = Pcos;
            base4d_matrix(1,2) = Nsin;
            base4d_matrix(2,1) = Psin;
            base4d_matrix(2,2) = Pcos;
            break;

        case OFFSET_DIRECTION_RY:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,0) = Pcos;
            base4d_matrix(0,2) = Psin;
            base4d_matrix(2,0) = Nsin;
            base4d_matrix(2,2) = Pcos;
            break;

        case OFFSET_DIRECTION_RZ:
            base4d_matrix << 1,0,0,0,
                             0,1,0,0,
                             0,0,1,0,
                             0,0,0,1;
            base4d_matrix(0,0) = Pcos;
            base4d_matrix(0,1) = Nsin;
            base4d_matrix(1,0) = Psin;
            base4d_matrix(1,1) = Pcos;
            break;

        default:
            throw std::invalid_argument("invalid argument");
    }

     //求齐次变化矩阵
     double MO=sqrt(Rx*Rx+Ry*Ry+Rz*Rz);
     double newRx= Rx/MO;
     double newRy= Ry/MO;
     double newRz= Rz/MO;

     Eigen::AngleAxisd PP1(MO,Vector3d(newRx,newRy,newRz));
     
     Eigen::Matrix3d R3d_matrix = PP1.matrix();
     Eigen::Matrix4d R4d_matrix ;
     R4d_matrix.block<3,3>(0,0) = R3d_matrix.block<3,3>(0,0);

    R4d_matrix(3,0) = 0;
    R4d_matrix(3,1) = 0;
    R4d_matrix(3,2) = 0;
    R4d_matrix(3,3) = 1;
    R4d_matrix(0,3) = x;
    R4d_matrix(1,3) = y;
    R4d_matrix(2,3) = z;

    Eigen::Matrix4d last4d_matrix = base4d_matrix*R4d_matrix;
    Eigen::Matrix3d last3d_matrix = last4d_matrix.block<3,3>(0,0);
    
    cout << "R4d_matrix:\n" << R4d_matrix << endl;
    cout << "R4d_matrix:\n" << base4d_matrix << endl;    
    cout << "last4d_matrix:\n" << last4d_matrix << endl;
    cout << "last3d_matrix:\n" << last3d_matrix << endl;
    
    double angle1;
    Vector3d axis1;
    Vector3d rotvector;
    Eigen::AngleAxisd PP2=Eigen::AngleAxisd(last3d_matrix);    
   
    angle1=PP2.angle();
    axis1=PP2.axis().transpose();

    rotvector=angle1*axis1;

     


    // const Matrix3d R = 
    //     Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix() * 
    //     Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() * 
    //     Eigen::AngleAxisd(psai, Eigen::Vector3d::UnitX()).matrix();

    
    double d[3]={last4d_matrix(0,3),last4d_matrix(1,3),last4d_matrix(2,3)};


     if (direction == OFFSET_DIRECTION_X || direction == OFFSET_DIRECTION_Y
         || direction == OFFSET_DIRECTION_Z)
     {
         
         return vector<double>{d[0], d[1], d[2], Rx, Ry, Rz};
         
     }
     else
     {
         return vector<double>{x, y, z, rotvector(0), rotvector(1), rotvector(2)};
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