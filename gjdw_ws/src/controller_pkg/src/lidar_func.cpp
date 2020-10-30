#include "lidar_func.h"
#include "ros/console.h"
#include "config.h"

#include <iostream>
#include <cmath>

using namespace std;
using namespace Eigen;

//Eigen::Matrix3d R;
//Vector3d x, y, z;

double mx0,mx1,mx2;
double my0,my1,my2;
double mz0,mz1,mz2;
extern int ConnectType;


std::vector<double>  calculate_offset_tool_coorNew(const std::vector<double>& tcp,double value,const int direction)
{

    //Eigen::AngleAxisd rotation_vector(alpha,Vector3d(x,y,z));
    //Eigen::Matrix3d rotation_matrix;

    //cout<<"tcp[0]="<<tcp[0]<<"\n";
    //cout<<"tcp[1]="<<tcp[1]<<"\n";
    // cout<<"tcp[2]="<<tcp[2]<<"\n";
    
    //rotation_vector(RX,RY,RZ);
    //rotation_matrix=rotation_vector.matrix();
    //return x;
    double xx1,yy1,zz1;

    if(direction==1)//x
    {
       // cout<<"direction=1,,,\n";
        xx1=tcp[0]+value*mx0;
        yy1=tcp[1]+value*mx1;
        zz1=tcp[2]+value*mx2;   
    }

    if(direction==2)//y
    {
       // cout<<"direction=2,,,\n";
        xx1=tcp[0]+value*my0;
        yy1=tcp[1]+value*my1;
        zz1=tcp[2]+value*my2;           
    }    

    if(direction==3)//z
    {
       // cout<<"direction=3,,,\n";
        xx1=tcp[0]+value*mz0;
        yy1=tcp[1]+value*mz1;
        zz1=tcp[2]+value*mz2;        
    }

  

    return vector<double>{xx1, yy1, zz1, 1, 2, 3};
}

// 将三维点从激光坐标系变换到机械臂坐标系
//  @point: 三维点
//  @dist: 滑台移动距离
//  @lidar_id: 激光雷达类型ID
//  @arm_id: 机械臂类型ID
Eigen::Vector3d lidar_to_arm_xyz(const Eigen::Vector3d& point, const double& dist, 
                                const int& lidar_id, const int& arm_id)
{
    Vector3d t(0, 0, 0);
    //水平雷达的零点
    if (LIDAR_TYPE_HORIZONTAL_LIDAR == lidar_id)
    {
        //相对于左臂
        if (ROBOT_ARM_LEFT_ARM == arm_id)
        {
             //t = Vector3d(404.93, 207.77, -68.7);
             //水平雷达相对于左臂的零点
            //t = Vector3d(304.93/1000, -242.23/1000, -68.7/1000);
            t = Vector3d(Lidar2ToMa1X,Lidar2ToMa1Y,Lidar2ToMa1Z);

            ROS_INFO_STREAM("Horizontal,master arm a0+p0,a1+mdist,a2+p1");
            ROS_INFO_STREAM("aPos: " << t.transpose());
        }
        //相对于右臂
        else if (ROBOT_ARM_RIGHT_ARM == arm_id)
        {
             //t = Vector3d(304.93, -242.23, -68.7);
             //水平雷达相对于右臂的零点
            //t = Vector3d(404.93/1000, 207.77/1000, -68.7/1000);
            t = Vector3d(Lidar2ToMa2X,Lidar2ToMa2Y,Lidar2ToMa2Z);

            ROS_INFO_STREAM("Horizontal,slave arm b0+p0,b1+mdist,b2+p1");
            ROS_INFO_STREAM("bPos: " << t.transpose());
        }
        //t += Vector3d(0, dist, 0);        
        t[0]=t[0]+point[0];
        t[1]=t[1]+dist;
        t[2]=t[2]+point[1];
    }
    //垂直雷达的零点
    else if (LIDAR_TYPE_VERTICAL_LIDAR == lidar_id)
    {
        //相对于左臂
        if (ROBOT_ARM_LEFT_ARM == arm_id)
        {
            
            //垂直雷达相对于左臂的零点
            //t = Vector3d(307.5/1000, -239.96/1000, -499.63/1000); 
            t = Vector3d(Lidar1ToMa1X,Lidar1ToMa1Y,Lidar1ToMa1Z);

            ROS_INFO_STREAM("Vertical,master arm c0+p1,c1-p0,c2+mdist");
            ROS_INFO_STREAM("cPos: " << t.transpose());
        }
        //相对于右臂
        else if (ROBOT_ARM_RIGHT_ARM == arm_id)
        {
            //t = Vector3d(307.5, -239.96, -499.63);
            //垂直雷达相对于右臂的零点
             
            //t = Vector3d(407.5/1000, 210.04/1000, -499.63/1000);
            t = Vector3d(Lidar1ToMa2X,Lidar1ToMa2Y,Lidar1ToMa2Z);

            ROS_INFO_STREAM("Vertical,slave arm d0+p1,d1-p0,d2+mdist");
            ROS_INFO_STREAM("dPos: " << t.transpose());             
        }
        //t += Vector3d(0, 0, dist);
        t[0]=t[0]+point[1];
        t[1]=t[1]-point[0];
        t[2]=t[2]+dist;
    }


    return t;
}



Eigen::Vector3d lidar_to_arm_rpyNew(const Eigen::Vector3d& toolsX,const int& lidartoma_id,const int& posetype)
{
    const Vector3d negative_x(-1, 0, 0),negative_y(0, -1, 0),negative_z(0, 0, -1);
    const Vector3d positive_x(1, 0, 0),positive_y(0, 1, 0), positive_z(0, 0, 1);
    Vector3d X=toolsX;
    ROS_INFO_STREAM("toolsX: " << toolsX.transpose()); 
    // 求坐标轴
    Vector3d Y, Z;
    if(lidartoma_id == LIDARTOMA1_ID)
    {
        if(posetype == 1)
        {
            ROS_INFO("lidar_to_arm_rpyNew,LIDARTOMA1======"); 
            Z = X.cross(negative_y);        
            Y = Z.cross(X);
        }
        else if(posetype == 2)
        {
            if(ConnectType == 1)
            {
                ROS_INFO("lidar_to_arm_rpyNew,LIDARTOMA1,left connect======"); 
                Z = X.cross(negative_x);        
                Y = Z.cross(X);
            }
            else if(ConnectType == 2)
            {
                ROS_INFO("lidar_to_arm_rpyNew,LIDARTOMA1,right connect======"); 
                Z = X.cross(positive_x);        
                Y = Z.cross(X);
            }
        }
        
    }      
    else if(lidartoma_id == LIDARTOMA2_ID)
    {
        if(posetype == 1)
        {
            ROS_INFO("lidar_to_arm_rpyNew,LIDARTOMA2======");
            Y = X.cross(negative_z);
            Z = X.cross(Y);
        }
    }
    else if(lidartoma_id == LIDARTOMA3_ID)
    {
        if(posetype == 1)
        {
            ROS_INFO("lidar_to_arm_rpyNew,LIDARTOMA3======");
            Y = X.cross(negative_z);
            Z = X.cross(Y);
        }
    }
  
    
    // 归一
    const Vector3d x = X.normalized(), y = Y.normalized(), z = Z.normalized();

    Eigen::Matrix3d R;
    R.block<3, 1>(0, 0) = x;
    R.block<3, 1>(0, 1) = y;
    R.block<3, 1>(0, 2) = z;

    cout << "R:\n" << R << endl;

    mx0=x[0];
    mx1=x[1];
    mx2=x[2];

    my0=y[0];
    my1=y[1];
    my2=y[2];

    mz0=z[0];
    mz1=z[1];
    mz2=z[2];

    // const Vector3d euler_angles=R.eulerAngles(2,1,0);

    // cout<<"euler_angles [0]="<<euler_angles[0]<<"\n";
    // cout<<"euler_angles [1]="<<euler_angles[1]<<"\n";
    // cout<<"euler_angles [2]="<<euler_angles[2]<<"\n";

    double angle1;
    Vector3d axis1;
    Vector3d rotvector;
    Eigen::AngleAxisd PP1=Eigen::AngleAxisd(R);    
   
    angle1=PP1.angle();
    axis1=PP1.axis().transpose();

    rotvector=angle1*axis1;

    // cout<<"rotvector0="<< axis1[0]<<"\n";
    // cout<<"rotvector1="<< axis1[1]<<"\n";
    // cout<<"rotvector2="<< axis1[2]<<"\n";
    // cout<<"rotvector2="<< angle1<<"\n";
    return rotvector;
}














// 计算两点表示的向量的旋转姿态，以RPY表示
//  @point1: 三维点
//  @point2: 三维点
//  @lidartoma_id: 坐标系类型
Eigen::Vector3d lidar_to_arm_rpy(const Eigen::Vector3d& toolsX,const int lidartoma_id)
{
    const Vector3d negative_y(0, -1, 0), negative_z(0, 0, -1);
    const Vector3d positive_y(0, 1, 0), positive__z(0, 0, 1);
    Vector3d X=toolsX;
    ROS_INFO_STREAM("toolsX: " << toolsX.transpose()); 

    // 求坐标轴
    Vector3d Y, Z;
    switch (lidartoma_id)
    {
    case LIDARTOMA1_ID:
        ROS_INFO("lidar_to_arm_rpy,LIDARTOMA1======"); 
        Z = X.cross(negative_y);
        Y = Z.cross(X);
        break;        
    case LIDARTOMA2_ID:
        ROS_INFO("lidar_to_arm_rpy,LIDARTOMA2======");
        Y = X.cross(negative_z);
        Z = X.cross(Y);
    case LIDARTOMA3_ID:
        ROS_INFO("lidar_to_arm_rpy,LIDARTOMA3======");
        Y = X.cross(negative_z);
        Z = X.cross(Y);
        break;        
    default:
        break;
    }
    
    // 归一
    const Vector3d x = X.normalized(), y = Y.normalized(), z = Z.normalized();

    // 生成旋转矩阵
    Eigen::Matrix3d R;
    R.block<3, 1>(0, 0) = x;
    R.block<3, 1>(0, 1) = y;
    R.block<3, 1>(0, 2) = z;

    cout << "R:\n" << R << endl;

    // 转换为欧拉角
    // (0, 1, 2)代表(X, Y, Z)
    // const Vector3d euler_angles = R.eulerAngles(0, 1, 2);
    // Eigen的转欧拉角接口不符合手算结果，手动实现
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; 

    

    float psai, theta, phi;

    //if (R(2, 0) != 1 && R(2, 0) != -1)
    if (!singular)
    {
        theta =atan2(R(2,1) , R(2,2));// -asin(R(2, 0));
        psai = atan2(-R(2,0), sy);//atan2(R(2, 1) / cos(theta), R(2, 2) / cos(theta));
        phi = atan2(R(1,0), R(0,0));//atan2(R(1, 0) / cos(theta), R(0, 0) / cos(theta));
    }
    else
    {
        theta=atan2(-R(1,2), R(1,1));
        psai=atan2(-R(2,0), sy);
        phi = 0;
        /*if (R(2, 0) == -1)
        {
            theta = M_PI / 2;
            psai = phi + atan2(R(0, 1), R(0, 2));
        }
        else
        {
            theta = -M_PI / 2;
            psai = -phi + atan2(-R(0, 1), -R(0, 2));
        }*/
    }
    return Vector3d(psai, theta, phi);
}














// int main()
// {
//     const Vector3d point(0, 1, 0);
//     const double dist = -25;
//     const Vector3d new_point = 
//         lidar_to_arm_xyz(point, dist, LIDAR_TYPE_HORIZONTAL_LIDAR, ROBOT_ARM_RIGHT_ARM);
//     cout << "lidar_to_arm_xyz: " << new_point.transpose() << endl;

//     const Vector3d point1(0, 0, 0), point2(0, 0, 1);
//     const Vector3d rpy = lidar_to_arm_rpy(point1, point2, LIDARTOMA3_ID);
//     cout << "lidar_to_arm_rpy: " << rpy.transpose() << endl;

//     cout << "tcp_move: " << tcp_move(10, "-z", Vector3d(1, 1, 1)).transpose() << endl;
//     cout << "tcp_move: " << tcp_move(10, "-y", Vector3d(1, 1, 1)).transpose() << endl;
//     cout << "tcp_move: " << tcp_move(10, "-x", Vector3d(1, 1, 1)).transpose() << endl;
//     return 0;
// }