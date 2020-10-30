/*
 *  Created on: Jun 29, 2019
 *      Author: shanchao
 */

#ifndef ROBOTENUM_H
#define ROBOTENUM_H

namespace RobotEnum {

enum RobotNodeType
{
  E_REGION_WORKSTATION = 0,   //workstation,pc or pad
  E_NODE_JOB_MNG,             //robot job manager node
  E_NODE_NODES_MNG,           //robot nodes manager node
  E_NODE_IMG_PROC,            //robot image process node
  E_NODE_LASER_PROC,          //robot laser data process node
  E_NODE_ARM_CTL,             //robot arm control node
  E_NODE_ARM_SAMPLE,          //robot arm data sample node
  E_NODE_ARM_TOOL_STRIP_WIRE, //robot arm strip wire tool node
  E_NODE_ARM_TOOL_CLAMP_WIRE, //robot arm clamp wire tool node
  E_NODE_ARM_TOOL_CUT_WIRE,   //robot arm cut wire tool node
  E_NODE_ARM_TOOL_CLAW,       //robot arm claw tool node
  E_NODE_SLIDE,               //robot slide node
  E_NODE_GIMBAL,              //robot gimbal node
  E_NODE_CAMERA,              //robot camera node
  E_NODE_LASER,               //robot laser node
};

enum RobotCmdCtlType
{
   E_CTRL_TYPE_SINGLE = 0, //single point control
   E_CTRL_TYPE_MULTI,      //multi point control
   E_CTRL_TYPE_SET         //set value
};

enum RobotCmdCtlStat
{
   E_CTRL_STAT_IDLE = 0,   //control idle
   E_CTRL_STAT_ACT,        //control active
   E_CTRL_STAT_EXE,        //control execute
   E_CTRL_STAT_STOP        //control stop
};

enum RobotErrCode
{
    E_ERR_CODE_SUCCESS = 0,
    E_ERR_CODE_CTL_NOT_ACT = -1,
    E_ERR_CODE_CTL_WITHOUT_STAT = -2,
};

}
#endif // ROBOTENUM_H
