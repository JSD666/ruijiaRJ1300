#ifndef AUTOMATIC_POINT_SELECTION_H
#define AUTOMATIC_POINT_SELECTION_H

#include <string>
#include <vector>
#include <algorithm>
#include <codecvt>
#include <cstdlib>
#include <memory>

#include "ros/ros.h"
#include <time.h>
#include "utils.h"
#include "config.h"

#include "std_msgs/String.h"
#include "public_pkg/cmd_msg.h"
#include "public_pkg/cmd_rsp_msg.h"
#include "public_pkg/status_digital_msg.h"
#include "public_pkg/heart_beat_msg.h"
#include "public_pkg/arms_power_srv.h"
#include "public_pkg/ground_interact_msg.h"
#include "public_pkg/range_msg.h"
#include "std_msgs/String.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h" 
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <iostream>
#include <fstream>
#include <cstdarg>
#include <vector>

class SegmentScheduler;

void send_command_to_rangeparam(ros::Publisher& pub, const int pointnum,
                                    const int display , const int camera, 
                                    double zLaserPos,std::vector<double> range);





void send_command_to_ground_check(ros::Publisher& pub, std::string value);

void send_command_to_all_ground_check(SegmentScheduler &scheduler, std::string value);




#endif // AUTO_POINT_SEL_H
