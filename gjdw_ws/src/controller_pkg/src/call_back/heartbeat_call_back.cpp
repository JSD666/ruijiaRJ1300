#include "call_back.h"
#include "SegmentScheduler.h"
#include "config.h"
#include "utils.h"
#include "string"

#include "rapidjson/reader.h"
#include "rapidjson/document.h"
#include "ros/console.h"


// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;

int now = current_time(TM_SEC),last_time = current_time(TM_SEC),diff_time = 0;

// 监听地面站心跳信息
void remotecontrol_heartbeat_call_back(const public_pkg::heart_beat_msg& msg)
{
    // now = current_time(TM_SEC);
    // diff_time = now -last_time;
    // if (diff_time > 6)
    // {
    //     ROS_INFO_STREAM("now - last_time: " << now << "-" << last_time << " = " << diff_time);
    // }
    // last_time = now;

    auto& event = scheduler->remote_hearbeat_flag_;
    event.notify(true);
}


int current_time(int ti)
{
    struct tm *local;
    time_t t; 
    t=time(NULL);
    local=localtime(&t);
    switch (ti)
    {
    case TM_YEAR:
        return local->tm_year + 1900; 
        break;
    
    case TM_MON:
        return local->tm_mon + 1; 
        break;

    case TM_MDAY:
        return local->tm_mday; 
        break;

    case TM_HOUR:
        return local->tm_hour; 
        break;
    
    case TM_MIN:
        return local->tm_min; 
        break;

    case TM_SEC:
        return local->tm_sec; 
        break;

    default:
        break;
    }
    
}