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

class CCmd_line_func
{
    public:
        int MaID_global;
        int Lidar_cmd_type;
    private:
       
};

CCmd_line_func mcmd_line_func;
int Lnew;

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


static CmdLineFuncRsp lidar_func(SegmentScheduler& scheduler, const std::string& line,
    const vector<string>& arg_names)
{   
    CmdLineFuncRsp cmdrspmsg; 
    string var_define;
    string var2_define;
    double translate_value=0.0;
    double sliderpos_value=0.0;
    int LineType=0;
    int posetype = 0;

    // 解析命令字段内容
    //选择主从臂要准确，不能可能产生意外的结果
    ROS_INFO("lidar_func is running");
    
    vector<string> fields = JobParser::GetCommandFields(line);
    if (fields.size() != arg_names.size())
    {
        ROS_ERROR_STREAM("expected " << arg_names.size() << " arguments, but got " << fields.size());
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    const string& var_cmdname = fields[0];
    //如果是命令LIDARTOMA
    if(var_cmdname == "#LIDARTOMA") 
    {
        ROS_INFO("#LIDARTOMA");
        Lnew=0;
    }
    //如果是命令LIDARTOMA1
    else if(var_cmdname == "#LIDARTOMA1") 
    {
        ROS_INFO("#LIDARTOMA1");
        try
        {
            Lnew=1;
            posetype = std::stoi(fields[3]);
            var_define = fields[4];
            translate_value = std::stod(fields[5]);
            var2_define = fields[6];
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
    }
    //如果是命令LIDARTOMA2
    else if(var_cmdname == "#LIDARTOMA2") 
    {
        ROS_INFO("#LIDARTOMA2");
        try
        {
            Lnew=2;
            LineType=std::stoi(fields[3]);
            var_define = fields[4];
            translate_value = std::stod(fields[5]);
            var2_define = fields[6];
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("#LIDARTOMA2 parameters wrong" );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
        }
        
    }
    //如果是命令LIDARTOMA3
    else if(var_cmdname == "#LIDARTOMA3") 
    {
        ROS_INFO("#LIDARTOMA3");
        try
        {
            Lnew=3;
            LineType=std::stoi(fields[3]);
            {
                const string var_define = fields[4];
                string var_name, var_typename;
                if (!JobParser::ParseVariableDefine(var_define, var_name, var_typename))
                {
                    ROS_ERROR_STREAM("4th field of #LIDARTOMA3 should be variable definition, instead of " << var_define);
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
                    ROS_ERROR_STREAM("Variable type of 4th field of #LIDARTOMA3 should be a number, instead of " << var_typename);
                    return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
                }
            }     
            var_define = fields[5];
            translate_value = std::stod(fields[6]);        
            var2_define = fields[7];
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("#LIDARTOMA3 parameters wrong" );
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;            
        }
    }
    else
    {
        ROS_INFO("#LIDAR2MA unknow");
        Lnew=-1;
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        //return -1;
    }

    // 设定字段名称到字段值的映射
    map<string, string> argname_to_field_value;
    
    for (size_t i = 0; i < arg_names.size(); ++i)
    {
        argname_to_field_value[arg_names[i]] = fields[i];
    }

    // 提取命令字段ID
    //LidarID=1是竖直激光雷达，LidarID=2是水平激光雷达 
    unsigned int LidarID;
    unsigned int MaID;
    try 
    {
         LidarID = stoi(argname_to_field_value["ID"]);
         MaID = stoi(argname_to_field_value["MaID"]);

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
    
    if (MaID != 1 && MaID != 2)
    {
        ROS_ERROR_STREAM("unrecognized MaID: " << MaID);
        return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }

    
    send_command_to_all_ground_check(scheduler,"clear");
    std::this_thread::sleep_for(chrono::milliseconds(100));//500ms
    send_command_to_all_ground_check(scheduler,"auto");

    if (LidarID  == 1)//竖直
    {
        ROS_INFO("LidarID  == 1,vertical lidar");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),9,"主控向地面站发送消息，指示在竖直网络摄像机画面内，点选像素坐标","info");
    }
    else if (LidarID  == 2)//水平
    {
        ROS_INFO("LidarID  == 2,horizontal lidar");
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),8,"主控向地面站发送消息，指示在水平网络摄像机画面内，点选像素坐标","info");

    }    
    ROS_INFO_STREAM("[IS WAITING FOR LIDAR POINT REACH]");
    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"等待雷达选点","info");
    /////////////////////////////////////////////////////////////////////////////////////////////
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ////////////////////////////////////////////////////////////////////////////////////////////

    // {
    // auto& event = scheduler.lidar_point_reach_flag_;
    // if (!event.wait_for(chrono::seconds(DEFAULT_LIDAR_RESPONSE_WAIT_TIME_SECONDS)))
    // {
    //     ROS_INFO("waitLidarPointReach timeout");
    //     send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
    //     send_command_to_all_ground_check(scheduler,"clear");
    //     return CMD_LINE_FUNC_RSP_TIMEOUT;
    // }       
    // else if (event.get() == false)
    // {
    //     ROS_INFO("waitLidarPointReach failed");
    //     send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
    //     send_command_to_all_ground_check(scheduler,"clear");
    //     return CMD_LINE_FUNC_RSP_FAILED;
    // }
    // }

    
    
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

    send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),13,"确认选点完成，开始计算数据","info");
    
    Vector3d tddd1(1,2,3);
    Vector3d tddd2(1,2,3);
   
    tddd1[0]=mPointX1;
    tddd1[1]=mPointY1;
    tddd1[2]=0;
    
    tddd2[0]=mPointX2;
    tddd2[1]=mPointY2;
    tddd2[2]=0;     
    if(Lnew==1)//LIDARTOMA1
    {
        ROS_INFO("LIDARTOMA1--------");
        cmdrspmsg = lidartoma1_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,MaID,posetype);
    }
    else if(Lnew==2)//LIDARTOMA2
    {
        ROS_INFO("LIDARTOMA2--------");
        cmdrspmsg = lidartoma2_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,MaID,LineType);
    }
    else if(Lnew==3)//LIDARTOMA3
    {
        ROS_INFO("LIDARTOMA3--------");
        cmdrspmsg = lidartoma3_variable_creat(scheduler,var_define,var2_define,tddd1,tddd2,translate_value,sliderpos_value,MaID,LineType);
    }
    else
    {
        ROS_ERROR("lidar_func: Invalid value for Lnew (should in [1, 3]), got %d", Lnew);
        cmdrspmsg = CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
    }
    if (cmdrspmsg == CMD_LINE_FUNC_RSP_FAILED)
    {
        send_info_to_ground_station(scheduler.getStatusDigitalMsgPub(),15,LIDARCHOOSEFAILURE,"WARNING");
    }
    return cmdrspmsg;
}


CmdLineFuncRsp lidartoma1_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         int MaID,int posetype)
{
    Vector3d new_point1;//点1转换出来的坐标
    Vector3d new_point2;//点2转换出来的坐标
    Vector3d mToolsX;
    DoubleValueArray pp;//var_PP_varname1
    DoubleValueArray var_pp;
    DoubleValueArray pp2;//var_PP_varname2
    int direction;
    double mtranslate=0; 
    ROS_INFO_STREAM("point1: " << point1.transpose() << ", mDist1: " << mDist1 << ", WhichLidar: " << WhichLidar << ", MaID: " << MaID);
    new_point1 = lidar_to_arm_xyz(point1, mDist1, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point1: " << new_point1.transpose());    
    new_point2 = lidar_to_arm_xyz(point2, mDist2, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point2: " << new_point2.transpose());
    ROS_INFO_STREAM("ConnectType: " << ConnectType); 

    if(mDist1 >= mDist2)
    {
        mToolsX =new_point1-new_point2;//ok
    }    
    else if(mDist1 < mDist2)
    {
        mToolsX =new_point2-new_point1;
    }    
    const Vector3d rpy1=lidar_to_arm_rpyNew(mToolsX,1,posetype);

    ROS_INFO_STREAM("lidar_to_arm_rpy: " << rpy1.transpose());  

    pp={new_point1[0],new_point1[1],new_point1[2],rpy1[0],rpy1[1],rpy1[2]};

    if(!DoubleValueArray_IsNumer(pp))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    liar_variable_output(scheduler,pp,var_define);
    
    ROS_INFO_STREAM("var_PP_varname1： " << pp);
    
    if(abs(offset_value)>1)
    {
        mtranslate=-offset_value/1000;
    }
    else
    {
        mtranslate=-offset_value;
    }    
    direction=3;//Z轴

    ROS_INFO_STREAM("WhichLidar: " << WhichLidar << ", offset_value: " << mtranslate << ", direction: " << direction );  
    
    var_pp=calculate_offset_tool_coorNew(pp,mtranslate,direction);

    pp2={var_pp[0],var_pp[1],var_pp[2],rpy1[0],rpy1[1],rpy1[2]};

    if(!DoubleValueArray_IsNumer(pp2))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    liar_variable_output(scheduler,pp2,var2_define);

    ROS_INFO_STREAM("var_PP_varname2： " << pp2);

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

CmdLineFuncRsp lidartoma2_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         int MaID,int LineType)
{
    Vector3d new_point1;//点1转换出来的坐标
    Vector3d new_point2;//点2转换出来的坐标
    Vector3d mToolsX;
    DoubleValueArray pp;//var_PP_varname1
    DoubleValueArray var_pp;
    DoubleValueArray pp2;//var_PP_varname2
    int direction;
    double mtranslate=0;  
      
    new_point1 = lidar_to_arm_xyz(point1, mDist1, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point1: " << new_point1.transpose());    
    new_point2 = lidar_to_arm_xyz(point2, mDist2, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point2: " << new_point2.transpose());
    ROS_INFO_STREAM("ConnectType: " << ConnectType);
          
    if(ConnectType==1)//左接,大的指向小的,小的减大的
    {
        //滑台位置较大指向滑台位置较小为工具X轴正方向，其实是负方向
        //点1大于点2，点2减点1
        if(mDist1>mDist2)
        {               
            mToolsX =new_point2-new_point1;//ok
        }        
        else if(mDist1<mDist2)
        {                
            mToolsX =new_point1-new_point2;
        }
    }
    else if(ConnectType==2)//右接，小的指向大的，就是大的减小的
    {
        //滑台位置较小指向滑台位置较大为工具X轴正方向，其实是正方向
        //点1大于点2，点1减点2，减后坐标方向为正
        if(mDist1>mDist2)
        {
            mToolsX =new_point1-new_point2;//ok
        }
        else if(mDist1<mDist2)
        {
            mToolsX =new_point2-new_point1;
        }
    } 
     
    const Vector3d rpy1=lidar_to_arm_rpyNew(mToolsX,2,1);
    ROS_INFO_STREAM("lidar_to_arm_rpy: " << rpy1.transpose()); 
    
    pp={new_point1[0],new_point1[1],new_point1[2],rpy1[0],rpy1[1],rpy1[2]};

    if(!DoubleValueArray_IsNumer(pp))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }
    liar_variable_output(scheduler,pp,var_define);
    ROS_INFO_STREAM("var_PP_varname1： " << pp);

    if(abs(offset_value)>1)
    {
        mtranslate=-offset_value/1000;
    }
    else
    {
        mtranslate=-offset_value;
    }                  
    if(LineType==1)//引流线。从lidar call back里继承的全局变量, 引流线
    {
        direction=1;//X轴
    }
    else if(LineType==2)//平行线
    {
        direction=3;//Z轴                      
    }  

    ROS_INFO_STREAM("WhichLidar: " << WhichLidar << ", offset_value: " << mtranslate << ", direction: " << direction );  
    var_pp=calculate_offset_tool_coorNew(pp,mtranslate,direction);
    pp2={var_pp[0],var_pp[1],var_pp[2],rpy1[0],rpy1[1],rpy1[2]};  

    if(!DoubleValueArray_IsNumer(pp2))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }      
    liar_variable_output(scheduler,pp2,var2_define);
    ROS_INFO_STREAM("var_PP_varname2： " << pp2);
    return CMD_LINE_FUNC_RSP_SUCCESS;
}


CmdLineFuncRsp lidartoma3_variable_creat(SegmentScheduler& scheduler,const std::string& var_define,
                                        const std::string& var2_define,const Eigen::Vector3d& point1,
                                        const Eigen::Vector3d& point2,  double offset_value, 
                                         double sliderpos_value,int MaID,int LineType)
{
    Vector3d new_point1;//点1转换出来的坐标
    Vector3d new_point2;//点2转换出来的坐标
    Vector3d mToolsX;
    DoubleValueArray pp;//var_PP_varname1
    DoubleValueArray var_pp;
    DoubleValueArray pp2;//var_PP_varname2
    int direction;
    double mtranslate=0; 
    double sliderpos_value_temp=0; 

    double cos_alpha=0;
    double sin_alpha=0;
    double tan_alpha=0;

    double sin_beta=0;
    double cos_beta=0;
    double tan_beta=0;
    
    double delta_z_p1p3;
    double delta_y_p1p3;
    double delta_xy_p1p3;
    double delta_x_p1p3;
    //double delta_y_p1p3;

    double x1,y1,z1;
    double x2,y2,z2;
    double x3,y3,z3;

    x3=0;
    y3=0;
    z3=0;  
    //double x1y1_x2y2_len;

    //如果滑台位置是mm，则转成m
    if(sliderpos_value>=1.5 )
    {
        sliderpos_value_temp=sliderpos_value/1000;
    }
    
    if(sliderpos_value<1.5 )
    {
        sliderpos_value_temp=sliderpos_value;
    }
    
    cout << "sliderpos_value=  "<<sliderpos_value<<"\n";
    //cout << "sliderpos_value_temp=  "<<sliderpos_value_temp<<"\n";

    if (ROBOT_ARM_LEFT_ARM == MaID)
    {
        //t = Vector3d(407.5, 210.04, -499.63);
        //垂直雷达相对于左臂的零点

        //cout<<"C====================================\n";
        y3= -242.23/1000+ sliderpos_value_temp;  
    }
    //相对于右臂
    else if (ROBOT_ARM_RIGHT_ARM == MaID)
    {
        //t = Vector3d(307.5, -239.96, -499.63);
        //垂直雷达相对于右臂的零点
        

        //t = Vector3d(1, -1, -1); 
        //cout<<"D====================================\n"; 

        y3= 207.77/1000+     sliderpos_value_temp;    
    }
      
    
    //XY坐标，滑台位置，雷达编号，机械臂 
    new_point1 = lidar_to_arm_xyz(point1, mDist1, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point1: " << new_point1.transpose());    
    new_point2 = lidar_to_arm_xyz(point2, mDist2, WhichLidar, MaID);
    ROS_INFO_STREAM("new_point2: " << new_point2.transpose());

    x1=new_point1[0];
    y1=new_point1[1];
    z1=new_point1[2];

    x2=new_point2[0];
    y2=new_point2[1];
    z2=new_point2[2];

       
  
    tan_beta=(z2-z1)/sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    cout << "tan_beta= "<<tan_beta<<"\n";
        
    cos_alpha=(y2-y1)/sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));
    cout << "cos_alpha="<<cos_alpha<<"\n"; 

    sin_alpha=sqrt(1-cos_alpha*cos_alpha);
    cout << "sin_alpha="<<sin_alpha<<"\n";
   
    tan_alpha=sin_alpha/cos_alpha;//=(x2-x1)/(y2-y1)=x3/y3

    x3=x1+(y3-y1)*tan_alpha;

    z3=z1+((y3-y1)/cos_alpha)*tan_beta;

     //作业方式，地面站触发作业的时候下发这个信息，在call barck里提取，并保存为全局变量
    ROS_INFO_STREAM("ConnectType: " << ConnectType);
          
    if(ConnectType==1)//左接,大的指向小的,小的减大的
    {
        //滑台位置较大指向滑台位置较小为工具X轴正方向，其实是负方向
        //点1大于点2，点2减点1
        if(mDist1>mDist2)
        {               
            mToolsX =new_point2-new_point1;//ok
        }        
        else if(mDist1<mDist2)
        {                
            mToolsX =new_point1-new_point2;
        }
    }
    else if(ConnectType==2)//右接，小的指向大的，就是大的减小的
    {
        //滑台位置较小指向滑台位置较大为工具X轴正方向，其实是正方向
        //点1大于点2，点1减点2，减后坐标方向为正
        if(mDist1>mDist2)
        {
            mToolsX = new_point1-new_point2;//ok
        }

        else if(mDist1<mDist2)
        {
            mToolsX = new_point2-new_point1;
        }
    }    

    const Vector3d rpy1=lidar_to_arm_rpyNew(mToolsX,3,1);

    ROS_INFO_STREAM("lidar_to_arm_rpy: " << rpy1.transpose());   

    pp={x3, y3, z3 ,rpy1[0],rpy1[1],rpy1[2]};

    if(!DoubleValueArray_IsNumer(pp))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    liar_variable_output(scheduler,pp,var_define);

    ROS_INFO_STREAM("var_PP_varname1： " << pp);

    if(abs(offset_value)>1)
    {
        mtranslate=-offset_value/1000;
    }
    else
    {
        mtranslate=-offset_value;
    }                 
    if(LineType==1)//引流线。从lidar call back里继承的全局变量, 引流线
    {
        direction=1;//X轴
    }
    else if(LineType==2)//平行线
    {
        direction=3;//Z轴                      
    }                    
    ROS_INFO_STREAM("WhichLidar: " << WhichLidar << ", offset_value: " << mtranslate << ", direction: " << direction );  
   
    var_pp=calculate_offset_tool_coorNew(pp,mtranslate,direction);

    pp2={var_pp[0], var_pp[1], var_pp[2] ,rpy1[0],rpy1[1],rpy1[2]};

    if(!DoubleValueArray_IsNumer(pp2))
    {
        return CMD_LINE_FUNC_RSP_FAILED;
    }

    liar_variable_output(scheduler,pp2,var2_define);

    ROS_INFO_STREAM("var_PP_varname2： " << pp2);    

    return CMD_LINE_FUNC_RSP_SUCCESS;
}

   
        
                 
 CmdLineFuncRsp liar_variable_output(SegmentScheduler& scheduler,const DoubleValueArray& pp,const std::string& var_define)
{        
        
    //写point1参数到数据表
    VariableTable& var_table = scheduler.getVariableTable();
    {            
        string var_name, var_typename;
        JobParser::ParseVariableDefine(var_define, var_name, var_typename);

        // 检查变量类型是否符合定义
        
        VariablePointer& vp = var_table.at(var_name);
        if (vp->getTypeName() != var_typename)
        {
            ROS_ERROR_STREAM("Variable definition conflicts with: " << *vp);
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        if (vp->getType() != Variable::VARIABLE_TYPE_DOUBLE_6)
        {
            ROS_ERROR_STREAM("Variable type should be INT intread of : " << vp->getTypeName());
            return CMD_LINE_FUNC_RSP_SYNTAX_ERROR;
        }
        
        
        vp->setValue<DoubleValueArray>(pp);

        //ROS_INFO_STREAM("Variable:" << var_define << ": " << *vp);
        //再读出
        //output_pp = vp->getValue<DoubleValueArray>();
    }

    return CMD_LINE_FUNC_RSP_SUCCESS;
}
       
    



CmdLineFuncRsp makepoint_lidar1_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDAR1");
    //目标定位（返回一个测量点）	
    //LIDAR1	ID	var_DOUBLE[2]_varname	var_INT_varname	
    const vector<string> arg_names{"CMD", "ID", "varname_double", "varname_int"};

    return lidar_func(scheduler, line, arg_names);
    //return 0;
}

CmdLineFuncRsp makepoint_lidar2_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDAR2");
    //目标定位（返回两个测量点）	
    //LIDAR2	ID	var_DOUBLE[2]_varname1	var_INT_varname1	var_DOUBLE[2]_varname2	var_INT_varname2
    const vector<string> arg_names{"CMD", "ID", 
                                    "varname1_double", "varname1_int", "varname2_double", "varname2_int"};

    return lidar_func(scheduler, line, arg_names);
    //return 0;
}

CmdLineFuncRsp makepoint_lidartoma_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA");
    //将雷达数据换算到机械臂坐标系下，姿态固定	
    //LIDARTOMA	LidarID	MaID	RX	RY	RZ	var_PP_varname
    const vector<string> arg_names{"CMD", "ID", "MaID",
                                    "RX", "RY", "RZ", 
                                    "varname_pp"};

    return lidar_func(scheduler, line, arg_names);
    //return 0;
}

CmdLineFuncRsp makepoint_lidartoma1_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA1");
    //将雷达数据换算到机械臂坐标系下，姿态不固定	
    //LIDARTOMA1	LidarID	MaID	var_PP_varname1	var_INT_translate	var_PP_varname2	
    const vector<string> arg_names{"CMD", "ID", "MaID",
                                    "posetype","varname1_pp", "translate_int", "varname2_pp"};

    return lidar_func(scheduler, line, arg_names);
    //return 0;
}

CmdLineFuncRsp makepoint_lidartoma2_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA2");
    //将雷达数据换算到机械臂坐标系下，姿态不固定	
    //LIDARTOMA2	LidarID	MaID	LineType	var_PP_varname1	var_INT_translate	var_PP_varname2	
    //#LIDARTOMA2,2,1,1,var_PP_YinLiuXian2,400,var_PP_preYinLiuXian2	
    const vector<string> arg_names{"CMD", "ID", "MaID", "LineType",
                                    "varname1_pp", "translate_int", "varname2_pp"};

    return lidar_func(scheduler, line, arg_names);
    //return 0;
}

CmdLineFuncRsp makepoint_lidartoma3_func(SegmentScheduler& scheduler, const std::string& line)
{
    ROS_INFO("Running command LIDARTOMA3");
    //将雷达数据换算到机械臂坐标系下，姿态不固定	
    //LIDARTOMA3	LidarID1	MaID2	LineType3	var_INT_sliderpos4	var_PP_varname15	
    //      var_INT_translate6	var_PP_varname27	
    const vector<string> arg_names{"CMD", "ID", "MaID","LineType",
                                    "sliderpos_int", 
                                    "varname1_pp", "translate_int", "varname2_pp", 
                                    };

    return lidar_func(scheduler, line, arg_names);
}

bool DoubleValueArray_IsNumer(const DoubleValueArray& pp)
{
    const regex num_pattern("[0-9]*.*[0-9]+");    // 数值匹配模式
    bool isNumber;
    for(int i = 0;i< pp.size();i++)
    {
        if (!regex_match(to_string(pp[i]), num_pattern))
        {            
            return false;
        }
    }
    return true;   
}