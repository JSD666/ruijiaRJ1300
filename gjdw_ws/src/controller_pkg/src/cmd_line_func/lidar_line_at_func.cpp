#include "cmd_line_func.h"
#include "config.h"
#include "JobParser.h"
#include "utils.h"
#include "SegmentScheduler.h"
#include "math_func.h"
#include "lidar_func.h"
#include "Variable.h"


#include "ros/console.h"
#include "public_pkg/cmd_msg.h"
#include "public_pkg/status_digital_msg.h"
#include "automatic_point_selection.h"

#include "Eigen/Eigen"

#include <vector>
#include <chrono>
#include <map>
#include <regex>

using namespace std;
using namespace Eigen;
extern int mPointNum;

extern double mPointX1;
extern double mPointY1;
extern double mDist1;

extern double mPointX2;
extern double mPointY2;
extern double mPointY2;
extern double mDist2;

extern int WhichLidar;
extern int ConnectType;




CmdLineFuncRsp lidar_AT_func(SegmentScheduler& scheduler, const std::string& line,
    const vector<string>& arg_names)
{    
    // 解析命令字段内容
    //选择主从臂要准确，不能可能产生意外的结果
    ROS_INFO_STREAM("lidar_AT_func is running : " << line);
    
    vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != arg_names.size())
    {
        ROS_ERROR_STREAM("expected " << arg_names.size() << " arguments, but got " << fields.size());
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    // 设定字段名称到字段值的映射
    map<string, string> argname_to_field_value;
    
    for (size_t i = 0; i < arg_names.size(); ++i)
    {
        argname_to_field_value[arg_names[i]] = fields[i];
    }

    //ROS_INFO_STREAM("size: " << arg_names.size() << " string; " << argname_to_field_value["illustrate"]); 
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,argname_to_field_value["illustrate"],"info");
        
    // 提取命令字段ID
    //LidarID=1是竖直激光雷达，LidarID=2是水平激光雷达 
    CmdLineFuncRsp cmdrspmsg; 
    string var_define;
    string var2_define;
    //提前点偏移值
    double translate_value=0.0;
    double sliderpos_value=0.0;
    int LineType=0;
    int posetype = 0;
    unsigned int LidarID;
    unsigned int MaID;
    double sliderStartPos;
    double sliderEndPos;
    int camera;
    DoubleValueArray point1range(4);
    DoubleValueArray point2range(4);
    try 
    {
        var_define = argname_to_field_value["varname1_pp"];
        var2_define = argname_to_field_value["varname2_pp"];
        translate_value = stod(argname_to_field_value["translate_int"]);
        LidarID = stoi(argname_to_field_value["ID"]);
        MaID = stoi(argname_to_field_value["MaID"]);
        sliderStartPos = stod(argname_to_field_value["sliderPos1"])/1000.0;
        sliderEndPos = stod(argname_to_field_value["sliderPos2"])/1000.0;
        point1range[0] = stod(argname_to_field_value["ALowerLimit1"])*M_PI/180.0;
        point1range[1] = stod(argname_to_field_value["AUpperLimit1"])*M_PI/180.0;
        point1range[2] = stod(argname_to_field_value["DLowerLimit1"])/1000.0;
        point1range[3] = stod(argname_to_field_value["DUpperLimit1"])/1000.0;
        point2range[0] = stod(argname_to_field_value["ALowerLimit2"])*M_PI/180;
        point2range[1] = stod(argname_to_field_value["AUpperLimit2"])*M_PI/180;
        point2range[2] = stod(argname_to_field_value["DLowerLimit2"])/1000.0;
        point2range[3] = stod(argname_to_field_value["DUpperLimit2"])/1000.0;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("#LIDARTOMA* ID or MaID should be a int" );
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
    }

    WhichLidar = LidarID;

    if (LidarID != CMD_FUNC_ARM_ID_LEFT_ARM && LidarID != CMD_FUNC_ARM_ID_RIGHT_ARM)
    {
        ROS_ERROR_STREAM("unrecognized LidarID: " << LidarID);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }    
    
    if (MaID != CMD_FUNC_ARM_ID_LEFT_ARM && MaID != CMD_FUNC_ARM_ID_RIGHT_ARM)
    {
        ROS_ERROR_STREAM("unrecognized MaID: " << MaID);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    ROS_INFO_STREAM("#LIDARTOMA1_AT* send_command_to_ground_check" );
    //切换到自动选点模式
    
    send_command_to_all_ground_check(scheduler,"clear");
    std::this_thread::sleep_for(chrono::milliseconds(100));//500ms

    //滑台移动到第一点
    cmdrspmsg = Slider_moveabs_func(scheduler,LidarID,sliderStartPos);
    if (cmdrspmsg!= CMD_LINE_FUNC_RSP_SUCCESS)
    return cmdrspmsg;

   
    double tempPos;

    if (LidarID  == 1)//竖直
    {
        //-135 to 0,camera0;0 to 135,camera3. -45~45,camera1; -90～-45,camera4
        if ((point1range[0] >= -135*M_PI/180) && (point1range[1] <= 0))
        {
            camera = 0;
        }
        else
        {
            camera = 3;
        }

        tempPos = scheduler.getVerticalSliderPos();
        //ROS_ERROR_STREAM("Horizontal slider position: " << scheduler.getHorizontalSliderPos()); 

        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),9,"主控向地面站发送消息，指示在竖直网络摄像机画面内，点选像素坐标","info");
        
    }
    else if (LidarID  == 2)//水平
    {
        if ((point1range[0] >= -45*M_PI/180) && (point1range[1] <= 45*M_PI/180))
        {
            camera = 1;
        }
        else
        {
            camera = 4;
        }

        
        tempPos = scheduler.getHorizontalSliderPos();
        //ROS_ERROR_STREAM("Vertical slider position: " << scheduler.getVerticalSliderPos());
    
        
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),8,"主控向地面站发送消息，指示在水平网络摄像机画面内，点选像素坐标","info");

    } 
    
    

    send_command_to_rangeparam(scheduler.getRangeParam_Pub(),1,1,camera,sliderStartPos,point1range);
    std::this_thread::sleep_for(chrono::milliseconds(10));//500ms
    send_command_to_all_ground_check(scheduler,"3d_pos");
    

    ROS_INFO_STREAM("[IS WAITING FOR LIDAR FIRST POINT REACH]");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待雷达第一个选点","info");
    /////////////////////////////////////////////////////////////////////////////////////////////
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ////////////////////////////////////////////////////////////////////////////////////////////
    {
    auto& event = scheduler.lidar_first_point_check_event;
    if (!event.wait_for(chrono::seconds(15)))
    {
        ROS_INFO("waitLidarPointReach timeout");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURETOMANUAL,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        goto ManualLidarChoose;
        //return CMD_LINE_FUNC_RSP_TIMEOUT;
    }       
    else if (event.get() == false)
    {
        ROS_INFO("waitLidarPointReach failed");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    }

    //send first point check
    std::this_thread::sleep_for(chrono::milliseconds(2000));//500ms
    send_command_to_all_ground_check(scheduler,"check");
    




    {
    auto& event = scheduler.lidar_first_point_reach_flag_;
    if (!event.wait_for(chrono::seconds(20)))
    {
        ROS_INFO("waitLidarPointReach timeout");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURETOMANUAL,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        goto ManualLidarChoose;
        
        //return CMD_LINE_FUNC_RSP_TIMEOUT;
    }       
    else if (event.get() == false)
    {
        ROS_INFO("waitLidarPointReach failed");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    }

    std::this_thread::sleep_for(chrono::milliseconds(100));//500ms
        



    if (LidarID  == 1)//竖直
        tempPos = scheduler.getVerticalSliderPos();
        
    else
        tempPos = scheduler.getHorizontalSliderPos();

    ROS_INFO_STREAM("CurretPos2: " << tempPos);
    tempPos = sliderEndPos;// - tempPos; 
    //ROS_INFO_STREAM("send tempPos2: " << tempPos);


    //滑台移动到第二点
    cmdrspmsg = Slider_moveabs_func(scheduler,LidarID,sliderEndPos);

    if (cmdrspmsg!= CMD_LINE_FUNC_RSP_SUCCESS)
     return cmdrspmsg;


    send_command_to_rangeparam(scheduler.getRangeParam_Pub(),2,1,camera,sliderEndPos,point2range);
    std::this_thread::sleep_for(chrono::milliseconds(10));//500ms
    send_command_to_all_ground_check(scheduler,"3d_pos");
    
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待雷达第二个选点","info");

    {
    auto& event = scheduler.lidar_second_point_check_event;
    if (!event.wait_for(chrono::seconds(15)))
    {
        ROS_INFO("waitLidarPointReach timeout");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURETOMANUAL,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        goto ManualLidarChoose;
        
        //return CMD_LINE_FUNC_RSP_TIMEOUT;
    }       
    else if (event.get() == false)
    {
        ROS_INFO("waitLidarPointReach failed");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    }

    

    std::this_thread::sleep_for(chrono::milliseconds(2000));//500ms
    send_command_to_all_ground_check(scheduler,"check");
    

    
    {
    auto& event = scheduler.lidar_point_reach_flag_;
    if (!event.wait_for(chrono::seconds(DEFAULT_LIDAR_RESPONSE_WAIT_TIME_SECONDS)))
    {
        ROS_INFO("waitLidarPointReach timeout");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURETOMANUAL,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        goto ManualLidarChoose;
        
        //return CMD_LINE_FUNC_RSP_TIMEOUT;
    }       
    else if (event.get() == false)
    {
        ROS_INFO("waitLidarPointReach failed");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        send_command_to_all_ground_check(scheduler,"clear");
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    }
    
    //自动失败跳到人工选点
    ManualLidarChoose:

    {// 雷达选点，确认选点完成信号
    auto& event = scheduler.lidar_point_confirm_flag_;
    if (!event.wait_for(chrono::seconds(900)))// 点击确认选点完成按钮等待时间为一分钟，超时则重新开始；
    {
        ROS_INFO("waitLidarPointConfirm timeout");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        return CMD_LINE_FUNC_RSP_TIMEOUT;
    }
    else if (event.get() == false)
    {
        ROS_INFO("waitLidarPointConfirm failed");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    }

     


    ROS_INFO_STREAM("[IS WAITING FOR LIDAR POINT REACH] OK+++++++++++++++++++++++++++++++++++++:");
    ROS_INFO_STREAM("mPointX1: " << mPointX1 << ", mPointY1: " << mPointY1);
    ROS_INFO_STREAM("mPointX2: " << mPointX2 << ", mPointY2: " << mPointY2);
    ROS_INFO_STREAM("mDist1: " << mDist1 << ", mDist2: " << mDist2);   

    

    send_command_to_all_ground_check(scheduler,"clear");
        

    ROS_INFO_STREAM("lidar point confirm reached, arm begin moveing");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"确认选点完成，开始计算路径","info");
   

    Vector3d tddd1(1,2,3);
    Vector3d tddd2(1,2,3);
   
    tddd1[0]=mPointX1;
    tddd1[1]=mPointY1;
    tddd1[2]=0;
    
    tddd2[0]=mPointX2;
    tddd2[1]=mPointY2;
    tddd2[2]=0; 

    const string& var_cmdname = fields[0];
   
    //如果是命令LIDARTOMA1
    if(var_cmdname == "#LIDARTOMA1_AT") 
    {
        ROS_INFO("#LIDARTOMA1_AT");
        try
        {
            posetype = std::stoi(argname_to_field_value["posetype"]);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("#LIDARTOMA1 parameters wrong" );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
        }
        if (posetype != 1 && posetype != 2)
        {
            ROS_ERROR_STREAM("unrecognized posetype: " << posetype);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }

        ROS_INFO("LIDARTOMA1--------");
        cmdrspmsg = lidartoma1_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,MaID,posetype);
    
    }
    //如果是命令LIDARTOMA2
    else if(var_cmdname == "#LIDARTOMA2_AT") 
    {
        ROS_INFO("#LIDARTOMA2_AT");
        try
        {
            LineType=std::stoi(argname_to_field_value["LineType"]);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("#LIDARTOMA2_AT parameters wrong" );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
        }

        ROS_INFO("LIDARTOMA2--------");
        cmdrspmsg = lidartoma2_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,MaID,LineType);
    
        
    }
    //如果是命令LIDARTOMA3
    else if(var_cmdname == "#LIDARTOMA3_AT") 
    {
        ROS_INFO("#LIDARTOMA3_AT");
        try
        {
            LineType=std::stoi(argname_to_field_value["LineType"]);
            {
                const string tempvarName = argname_to_field_value["RecordSliderpos"];
                string var_name, var_typename;
                if (!JobParser::ParseVariableDefine(tempvarName, var_name, var_typename))
                {
                    ROS_ERROR_STREAM("4th field of #LIDARTOMA3_AT should be variable definition, instead of " << var_define);
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
                VariablePointer vp = scheduler.getVariableTable().at(var_name);
                switch (vp->getType())
                {
                case Variable::VARIABLE_TYPE_INT:
                    sliderpos_value = vp->getValue<IntValue>();
                    break;

                case Variable::VARIABLE_TYPE_DOUBLE:
                    sliderpos_value = vp->getValue<DoubleValue>();
                    break;

                default:
                    ROS_ERROR_STREAM("Variable type of 4th field of #LIDARTOMA3_AT should be a number, instead of " << var_typename);
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("#LIDARTOMA3 parameters wrong" );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
        }

        ROS_INFO("LIDARTOMA3--------");
        cmdrspmsg = lidartoma3_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,sliderpos_value,MaID,LineType);
    
    }
    else
    {
        ROS_INFO("#LIDAR2MA unknow");
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    if (cmdrspmsg == CMD_LINE_FUNC_RSP_FAILED)
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
    }
    return cmdrspmsg;
}


CmdLineFuncRsp makepoint_lidartoma1_AT_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA1_AT");
    const vector<string> arg_names{"CMD", "ID", "MaID", "posetype","varname1_pp", 
                                    "translate_int", "varname2_pp", "sliderPos1",
                                    "ALowerLimit1","AUpperLimit1","DLowerLimit1",
                                    "DUpperLimit1", "sliderPos2", "ALowerLimit2",
                                    "AUpperLimit2","DLowerLimit2","DUpperLimit2","illustrate"};

    return lidar_AT_func(scheduler, line, arg_names);
}

CmdLineFuncRsp makepoint_lidartoma2_AT_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA2_AT");
    const vector<string> arg_names{"CMD", "ID", "MaID", "LineType","varname1_pp", 
                                    "translate_int", "varname2_pp", "sliderPos1",
                                    "ALowerLimit1","AUpperLimit1","DLowerLimit1",
                                    "DUpperLimit1", "sliderPos2","ALowerLimit2",
                                    "AUpperLimit2","DLowerLimit2","DUpperLimit2","illustrate" };

    return lidar_AT_func(scheduler, line, arg_names);
}

CmdLineFuncRsp makepoint_lidartoma3_AT_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA3_AT");
    const vector<string> arg_names{"CMD", "ID", "MaID", "LineType","RecordSliderpos","varname1_pp", 
                                    "translate_int", "varname2_pp", "sliderPos1",
                                    "ALowerLimit1","AUpperLimit1","DLowerLimit1","DUpperLimit1", 
                                    "sliderPos2","ALowerLimit2","AUpperLimit2","DLowerLimit2",
                                    "DUpperLimit2","illustrate"};

    return lidar_AT_func(scheduler, line, arg_names);
}



