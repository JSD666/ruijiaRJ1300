#include "call_back.h"
#include "SegmentScheduler.h"
#include "config.h"
#include "utils.h"

#include "rapidjson/reader.h"
#include "rapidjson/document.h"

#include "ros/console.h"

#include <memory>

using namespace std;

// 在controller.cpp中定义
extern std::shared_ptr<SegmentScheduler> scheduler;

// 监听通用模拟量信息
void status_analog_call_back(const public_pkg::status_analog_msg& msg)
{
    // 过滤不匹配类型的消息
    if (msg.header.msgType != MSG_TYPE_STATUS_ANALOG_MSG)
    {
        ROS_ERROR_STREAM("status_analog_call_back: drop one message because type mismatch. "
                         "msgType should be " << MSG_TYPE_STATUS_ANALOG_MSG << ", instead of " << msg.header.msgType);
        return;
    }

    // 消息频率高，因此使用DEBUG接口

    //ROS_INFO_("status_analog_call_back: received content: \"%s\"", msg.content.c_str());
    
    //ROS_DEBUG("status_analog_call_back: received content: \"%s\"", msg.content.c_str());
    // 过滤自己发的包
    if (msg.header.srcNodeName == NODE_NAME_CONTROLLER)
    {
        return;
    }

    //ROS_INFO_STREAM("status_analog_call_back: received content" << msg.content.c_str());

    rapidjson::Document d;
    d.Parse(msg.content.c_str());
    // if (!d.IsObject())
    // {
    //     ROS_ERROR("status_analog_call_back: received content invalid");
    //     return;
    // }

    // 来自滑台结点的消息
    if (msg.header.srcNodeName == NODE_NAME_SLIDER && msg.header.srcNodeType == NODE_TYPE_SLIDER)
    {
        const rapidjson::Value& analogArray = d["analogArray"];
        if (!analogArray.IsArray())
        {
            ROS_ERROR("status_analog_call_back: received content invalid, cannot parse analogArray");
            return;
        }

        for (size_t i = 0; i < analogArray.Size(); ++i)
        {
            const rapidjson::Value& seq = analogArray[i]["seq"];
            if (!seq.IsNumber())
            {
                ROS_ERROR("status_analog_call_back: received content invalid, cannot parse seq of analogArray");
                return;
            }

            switch (seq.GetUint())
            {
            case GENERAL_ANALOG_MSG_HORIZONTAL_SLIDER_POS_SEQ:
                {
                    const rapidjson::Value& value = analogArray[i]["value"];
                    if (!value.IsNumber())
                    {
                        ROS_ERROR("status_analog_call_back: received content invalid, cannot parse value of analogArray");
                        return;
                    }
                    scheduler->setHorizontalSliderPos(value.GetDouble());
                    ROS_DEBUG_STREAM("Horizontal slider position: " << scheduler->getHorizontalSliderPos());
                }
                break;

            case GENERAL_ANALOG_MSG_VERTICAL_SLIDER_POS_SEQ:
                {
                    const rapidjson::Value& value = analogArray[i]["value"];
                    if (!value.IsNumber())
                    {
                        ROS_ERROR("status_analog_call_back: received content invalid, cannot parse value of analogArray");
                        return;
                    }
                    scheduler->setVerticalSliderPos(value.GetDouble());
                    ROS_DEBUG_STREAM("Vertical slider position: " << scheduler->getVerticalSliderPos());
                }
                break;

            default:
                break;
            }
        }
    }

	// 来自机械臂的遥测消息
    if (msg.header.srcNodeName == NODE_NAME_ARM_INFO_ANALOG && msg.header.srcNodeType == NODE_TYPE_ARM)
    {
        const rapidjson::Value& analogArray = d["analogArray"];
        if (!analogArray.IsArray())
        {
            ROS_ERROR("status_analog_call_back: received content invalid, cannot parse analogArray");
            return;
        }

        auto& event = scheduler->arm_analog_info_received_flag_;
        if (event.isWaiting())
        {
        	try
        	{
        		DoubleValueArray point1 = {0, 0, 0, 0, 0, 0};
				DoubleValueArray point2 = {0, 0, 0, 0, 0, 0};
                DoubleValueArray point0 = {0, 0, 0, 0, 0, 0};
                for (size_t i = 0; i < analogArray.Size(); ++i)
                {
                    const rapidjson::Value& seq = analogArray[i]["seq"];
                    if (1 <= seq.GetInt() && seq.GetInt() <= 6) // 主臂tcp //(1 <= seq.GetInt() <= 6)这样写判断无效
                    {                
                        point1[i] = analogArray[i]["value"].GetDouble();
                        //point1[5] = analogArray[GENERAL_ANALOG_MSG_l_wrist_3_joint - 1]["value"].GetDouble();
                    }
                    if (101 <= seq.GetInt() && seq.GetInt() <= 106) // 从臂tcp
                    {
                        point2[i] = analogArray[i]["value"].GetDouble();
                    }
                }
                
                if (point1 != point0)
                {
                    ROS_INFO_STREAM("get analog point1 value: " << point1 );
                    scheduler->setArm1Joint(point1);
                }
                if (point2 != point0)
                {
                    ROS_INFO_STREAM("get analog point2 value: " << point2 );
                    scheduler->setArm2Joint(point2);
                }
				//ROS_INFO("Notifying arm analog value received.");
				event.notify(true);
        	}
			catch (const std::exception& e) 
			{
		        ROS_ERROR_STREAM("ERROR: failed to get analog value: " << e.what());
		    }
        }
    }
}
