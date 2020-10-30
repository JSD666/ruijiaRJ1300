#include "call_back.h"
#include "SegmentScheduler.h"
#include "config.h"
#include "utils.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h"

#include "Eigen/Eigen"
#include "lidar_func.h"

#include "ros/console.h"

#include <memory>

using namespace std;

// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;

// ------------
// Comments added by guanheng huang
// 激光雷达回调函数会设置这些变量的值，然后激光雷达的功能函数会读取它们
// TODO: 将数据成员委托给SegmentScheduler，见SegmentScheduler::setHorizontalSliderPos()
int mPointNum;
double mPointX1;
double mPointY1;
double mDist1;

double mPointX2;
double mPointY2;
double mDist2;

int WhichLidar;

//int pointNum1_flag=0;
//int pointNum2_flag=0;
int ConnectType=0;
// ------------
// ------------

//1为竖直

void lidar_call_back_common_func(const fusion::server_position_msg& msg)
{
    

    
}

//引流线A
void status_lidar1_call_back(const fusion::server_position_msg& msg)
{
    static int pointNum1_flag = 0;
    static int pointNum2_flag = 0;

    auto& event = scheduler->lidar_point_reach_flag_;
    auto& event1 = scheduler->lidar_first_point_reach_flag_;
    auto& event1check = scheduler->lidar_first_point_check_event;
    auto& event2check = scheduler->lidar_second_point_check_event;
    if (!event.isWaiting())
    {
        pointNum1_flag = 0;
        pointNum2_flag = 0;
    }

    if(msg.pointNum==1.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event1check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点数据确认结果完成");
            event1check.notify(true);
        }

    }

    if(msg.pointNum==2.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event2check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第二个选点数据确认结果完成");
            event2check.notify(true);
        }

    }
    
    
    
    if(msg.pointNum==1.0 && msg.flag==2.0)// && pointNum1_flag==0)
    {  
        pointNum1_flag=1;
        mPointX1=msg.position[0];
        mPointY1=msg.position[1];
        mDist1=msg.position[2]; 
             
    }
    else if(msg.pointNum==2 && msg.flag==2)
    {  
        mPointNum=msg.pointNum;
        mPointX2=msg.position[0];
        mPointY2=msg.position[1];
        mDist2=msg.position[2]; 
        pointNum2_flag=1; 
    }

    if (pointNum1_flag==1)
    {
        if(event1.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点完成");
            event1.notify(true);
        }
    }
    
    if (pointNum2_flag==1)
    {
        if (event.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达选点全部完成");
            event.notify(true);
            pointNum1_flag = 0;
            pointNum2_flag = 0;
        }
    }
    // ROS_INFO("status_lidar1_call_back1");
    //lidar_call_back_common_func(msg);
}
//行线A
void status_lidar2_call_back(const fusion::server_position_msg& msg)
{
    static int pointNum1_flag = 0;
    static int pointNum2_flag = 0;

    auto& event = scheduler->lidar_point_reach_flag_;
    auto& event1 = scheduler->lidar_first_point_reach_flag_;
    auto& event1check = scheduler->lidar_first_point_check_event;
    auto& event2check = scheduler->lidar_second_point_check_event;
    if (!event.isWaiting())
    {
        pointNum1_flag = 0;
        pointNum2_flag = 0;
    }

    if(msg.pointNum==1.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event1check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点数据确认结果完成");
            event1check.notify(true);
        }

    }

    if(msg.pointNum==2.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event2check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第二个选点数据确认结果完成");
            event2check.notify(true);
        }

    }
    
    
    
    if(msg.pointNum==1.0 && msg.flag==2.0)// && pointNum1_flag==0)
    {  
        pointNum1_flag=1;
        mPointX1=msg.position[0];
        mPointY1=msg.position[1];
        mDist1=msg.position[2]; 
             
    }
    else if(msg.pointNum==2 && msg.flag==2)
    {  
        mPointNum=msg.pointNum;
        mPointX2=msg.position[0];
        mPointY2=msg.position[1];
        mDist2=msg.position[2]; 
        pointNum2_flag=1; 
    }

    if (pointNum1_flag==1)
    {
        if(event1.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点完成");
            event1.notify(true);
        }
    }
    
    if (pointNum2_flag==1)
    {
        if (event.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达选点全部完成");
            event.notify(true);
            pointNum1_flag = 0;
            pointNum2_flag = 0;
        }
    }
    // ROS_INFO("status_lidar1_call_back2");
    //lidar_call_back_common_func(msg);
}
//引流线B
void status_lidar1b_call_back(const fusion::server_position_msg& msg)
{
    static int pointNum1_flag = 0;
    static int pointNum2_flag = 0;

    auto& event = scheduler->lidar_point_reach_flag_;
    auto& event1 = scheduler->lidar_first_point_reach_flag_;
    auto& event1check = scheduler->lidar_first_point_check_event;
    auto& event2check = scheduler->lidar_second_point_check_event;
    if (!event.isWaiting())
    {
        pointNum1_flag = 0;
        pointNum2_flag = 0;
    }

    if(msg.pointNum==1.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event1check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点数据确认结果完成");
            event1check.notify(true);
        }

    }

    if(msg.pointNum==2.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event2check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第二个选点数据确认结果完成");
            event2check.notify(true);
        }

    }
    
    
    
    if(msg.pointNum==1.0 && msg.flag==2.0)// && pointNum1_flag==0)
    {  
        pointNum1_flag=1;
        mPointX1=msg.position[0];
        mPointY1=msg.position[1];
        mDist1=msg.position[2]; 
             
    }
    else if(msg.pointNum==2 && msg.flag==2)
    {  
        mPointNum=msg.pointNum;
        mPointX2=msg.position[0];
        mPointY2=msg.position[1];
        mDist2=msg.position[2]; 
        pointNum2_flag=1; 
    }

    if (pointNum1_flag==1)
    {
        if(event1.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点完成");
            event1.notify(true);
        }
    }
    
    if (pointNum2_flag==1)
    {
        if (event.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达选点全部完成");
            event.notify(true);
            pointNum1_flag = 0;
            pointNum2_flag = 0;
        }
    }
    //lidar_call_back_common_func(msg);
}
//行线B
void status_lidar2b_call_back(const fusion::server_position_msg& msg)
{
    static int pointNum1_flag = 0;
    static int pointNum2_flag = 0;

    auto& event = scheduler->lidar_point_reach_flag_;
    auto& event1 = scheduler->lidar_first_point_reach_flag_;
    auto& event1check = scheduler->lidar_first_point_check_event;
    auto& event2check = scheduler->lidar_second_point_check_event;
    if (!event.isWaiting())
    {
        pointNum1_flag = 0;
        pointNum2_flag = 0;
    }

    if(msg.pointNum==1.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event1check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点数据确认结果完成");
            event1check.notify(true);
        }

    }

    if(msg.pointNum==2.0 && msg.flag==1.0)// && pointNum1_flag==0)
    {
        if(event2check.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第二个选点数据确认结果完成");
            event2check.notify(true);
        }

    }
    
    
    
    if(msg.pointNum==1.0 && msg.flag==2.0)// && pointNum1_flag==0)
    {  
        pointNum1_flag=1;
        mPointX1=msg.position[0];
        mPointY1=msg.position[1];
        mDist1=msg.position[2]; 
             
    }
    else if(msg.pointNum==2 && msg.flag==2)
    {  
        mPointNum=msg.pointNum;
        mPointX2=msg.position[0];
        mPointY2=msg.position[1];
        mDist2=msg.position[2]; 
        pointNum2_flag=1; 
    }

    if (pointNum1_flag==1)
    {
        if(event1.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达第一个选点完成");
            event1.notify(true);
        }
    }
    
    if (pointNum2_flag==1)
    {
        if (event.isWaiting())
        {
            send_info_to_ground_station(scheduler->getStatusDigitalMsgPub(),13,"雷达选点全部完成");
            event.notify(true);
            pointNum1_flag = 0;
            pointNum2_flag = 0;
        }
    }
    //lidar_call_back_common_func(msg);
}




