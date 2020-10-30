#include "automatic_point_selection.h"
#include "SegmentScheduler.h"




using namespace std;

extern std::shared_ptr<SegmentScheduler> scheduler;

void send_command_to_rangeparam(ros::Publisher& pub, const int pointnum,
                                    const int display , const int camera, 
                                    double zLaserPos,vector<double> range)
{
    public_pkg::range_msg msg;
    msg.pointNum = pointnum;
    msg.display = display;
    msg.camera = camera;
    msg.zLazerPos = zLaserPos;
    msg.ranges = range;

    pub.publish(msg);
}





void send_command_to_ground_check(ros::Publisher& pub, string value)
{
    std_msgs::String msg;
    msg.data = value;
    pub.publish(msg);
}

void send_command_to_all_ground_check(SegmentScheduler &scheduler, std::string value)
{
    
    
    std_msgs::String msg;
    msg.data = value;
    scheduler.getGround_Check_Pub1().publish(msg);
    scheduler.getGround_Check_Pub2().publish(msg);
    scheduler.getGround_Check_Pub3().publish(msg);
    scheduler.getGround_Check_Pub4().publish(msg);
}

