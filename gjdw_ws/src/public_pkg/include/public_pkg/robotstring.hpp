/*
 *  Created on: Jun 29, 2019
 *      Author: shanchao
 */

#ifndef ROBOT_STRING_HPP
#define ROBOT_STRING_HPP

#include "robotenum.h"

template<typename EnumType>
struct SEnumName
{
  static const char* List[];
};

const char* SEnumName<RobotNodeType>::List[] =
{
  "WorkStation",
  "JobMng",
  "NodesMng",
  "ImgProc",
  "LaserProc",
  "ArmCtl",
  "ArmSample",
  "StripWireTool",
  "ClampWireTool",
  "CutWireTool",
  "ClawTool",
  "Slide",
  "Gimbal",
  "Camera",
  "Laser"
};

template<typename EnumType>
EnumType ConvertStringToEnum(const char* pStr)
{
  EnumType fooEnum = static_cast<EnumType>(-1);
  int count = sizeof(SEnumName<EnumType>::List) /
    sizeof(SEnumName<EnumType>::List[0]);
  for (int i = 0; i < count; ++i)
  {
    if (!abs(strcmp(pStr, SEnumName<EnumType>::List[i])))
    {
      fooEnum = static_cast<EnumType>(i);
      break;
    }
  }
  return fooEnum;
}

template<typename EnumType>
const char* ConvertEnumToString(EnumType enumPara)
{
  return SEnumName<EnumType>::List[enumPara];
}

#endif
