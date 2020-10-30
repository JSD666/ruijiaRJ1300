/*
 *  Created on: Aug 23, 2019
 *      Author: shanchao
 */

#ifndef MSGWRAPPER_H
#define MSGWRAPPER_H

#include <stdint.h>
#include <string>
#include <time.h>
#include <ros/param.h>

#include "public_pkg/status_header_msg.h"
#include "public_pkg/cmd_header_msg.h"

#include "public_pkg/rapidjson/stringbuffer.h"
#include "public_pkg/rapidjson/writer.h"
#include "public_pkg/rapidjson/document.h"

namespace MsgWrapper
{

class CBaseContent
{
public:
  unsigned    m_seq;
  std::string m_type;
  std::string m_name;
  union
  {
    double m_dval;
    int m_nval;
  } m_value;

  CBaseContent(): m_seq(0), m_type(""), m_name("")
  {
      m_value.m_dval = 0;
      m_value.m_nval = 0;
  }
};

class CCmdContent: public CBaseContent
{
public:
  unsigned m_ctlID;
  unsigned m_ctlStat;

  CCmdContent(): m_ctlID(0), m_ctlStat(0){}
};

class CJsonMsgContent
{

public:

  static std::string oneDoubleAnalog(const CBaseContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("analogArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Double");
    writers.Key("value");
    writers.Double(content.m_value.m_dval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneIntAnalog(const CBaseContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("analogArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Int");
    writers.Key("value");
    writers.Int(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneDigital(const CBaseContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("digitalArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Uint");
    writers.Key("value");
    writers.Uint(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneDoubleCmd(const CBaseContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("cmdArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Double");
    writers.Key("value");
    writers.Uint(content.m_value.m_dval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneIntCmd(const CBaseContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("cmdArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Int");
    writers.Key("value");
    writers.Int(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneIntCmd(const CCmdContent &content)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("cmdArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Int");
    writers.Key("value");
    writers.Int(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.Key("ctlStat");
    writers.Uint(content.m_ctlStat);
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneIntCmdRsp(const CBaseContent &content, int errCode)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("cmdRspArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Int");
    writers.Key("value");
    writers.Int(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.Key("errCode");
    writers.Int(errCode);
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static std::string oneIntCmdRsp(const CCmdContent &content, int errCode)
  {
    rapidjson::StringBuffer ss;
    rapidjson::Writer<rapidjson::StringBuffer> writers(ss);

    writers.StartObject();

    writers.Key("cmdRspArray");

    writers.StartArray();

    writers.StartObject();
    writers.Key("seq");
    writers.Uint(content.m_seq);
    writers.Key("type");
    writers.String("Int");
    writers.Key("value");
    writers.Int(content.m_value.m_nval);
    writers.Key("name");
    writers.String(content.m_name.c_str());
    writers.Key("errCode");
    writers.Int(errCode);
    writers.Key("ctlStat");
    writers.Uint(content.m_ctlStat);
    writers.EndObject();

    writers.EndArray();

    writers.EndObject();

    return std::string(ss.GetString());
  }

  static bool parseAnalog(const std::string &content, std::vector<CBaseContent> &analogs)
  {
    analogs.clear();
    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError() || !document.HasMember("analogArray"))
    {
      return false;
    }

    const rapidjson::Value &jvObjArray = document["analogArray"];
    if(!jvObjArray.IsArray())
    {
      return false;
    }

    for (size_t i = 0; i < jvObjArray.Size(); ++i)
    {
      CBaseContent analog;
      const rapidjson::Value &jvObj = jvObjArray[i];
      if(!jvObj.IsObject())
      {
        return false;
      }

      if(jvObj.HasMember("seq") && jvObj["seq"].IsUint())
      {
        analog.m_seq = jvObj["seq"].GetUint();
      }
      else
      {
        continue;
      }

      if(jvObj.HasMember("type") && jvObj["type"].IsString())
      {
        analog.m_type = jvObj["type"].GetString();
      }
      else
      {
        continue;
      }

      if(analog.m_type.compare(std::string("Double")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsDouble())
        {
          analog.m_value.m_dval = jvObj["value"].GetDouble();
        }
        else
        {
          continue;
        }
      }
      else if(analog.m_type.compare(std::string("Int")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsInt())
        {
          analog.m_value.m_nval = jvObj["value"].GetInt();
        }
        else
        {
          continue;
        }
      }
      else
      {
        continue;
      }

      if(jvObj.HasMember("name") && jvObj["name"].IsString())
      {
        analog.m_name = jvObj["name"].GetString();
      }
      analogs.push_back(analog);
    }
    return (analogs.size() > 0) ? true : false;
  }

  static bool parseDigital(const std::string &content, std::vector<CBaseContent> &digitals)
  {
    digitals.clear();
    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError() || !document.HasMember("digitalArray"))
    {
      return false;
    }

    const rapidjson::Value &jvObjArray = document["digitalArray"];
    if(!jvObjArray.IsArray())
    {
      return false;
    }

    for (size_t i = 0; i < jvObjArray.Size(); ++i)
    {
      CBaseContent digital;
      const rapidjson::Value &jvObj = jvObjArray[i];
      if(!jvObj.IsObject())
      {
        return false;
      }

      if(jvObj.HasMember("seq") && jvObj["seq"].IsUint())
      {
        digital.m_seq = jvObj["seq"].GetUint();
      }
      else
      {
        continue;
      }

      if(jvObj.HasMember("type") && jvObj["type"].IsString())
      {
        digital.m_type = jvObj["type"].GetString();
      }
      else
      {
        continue;
      }

      if(digital.m_type.compare(std::string("Uint")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsUint())
        {
          digital.m_value.m_nval = jvObj["value"].GetUint();
        }
        else
        {
          continue;
        }
      }
      else
      {
        continue;
      }

      if(digital.m_value.m_nval > 1)
      {
          continue;
      }

      if(jvObj.HasMember("name") && jvObj["name"].IsString())
      {
        digital.m_name = jvObj["name"].GetString();
      }

      digitals.push_back(digital);
    }
    return (digitals.size() > 0) ? true : false;
  }

  static bool parseCmd(const std::string &content, std::vector<CBaseContent> &cmds)
  {
    cmds.clear();
    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError() || !document.HasMember("cmdArray"))
    {
      return false;
    }

    const rapidjson::Value &jvObjArray = document["cmdArray"];
    if(!jvObjArray.IsArray())
    {
      return false;
    }

    for (size_t i = 0; i < jvObjArray.Size(); ++i)
    {
      CBaseContent cmd;
      const rapidjson::Value &jvObj = jvObjArray[i];
      if(!jvObj.IsObject())
      {
        return false;
      }

      if(jvObj.HasMember("seq") && jvObj["seq"].IsUint())
      {
        cmd.m_seq = jvObj["seq"].GetUint();
      }
      else
      {
        continue;
      }

      if(jvObj.HasMember("type") && jvObj["type"].IsString())
      {
        cmd.m_type = jvObj["type"].GetString();
      }
      else
      {
        continue;
      }

      if(cmd.m_type.compare(std::string("Double")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsDouble())
        {
          cmd.m_value.m_dval = jvObj["value"].GetDouble();
        }
        else
        {
          continue;
        }
      }
      else if(cmd.m_type.compare(std::string("Int")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsInt())
        {
          cmd.m_value.m_nval = jvObj["value"].GetInt();
        }
        else
        {
          continue;
        }
      }
      else
      {
        continue;
      }

      if(jvObj.HasMember("name") && jvObj["name"].IsString())
      {
        cmd.m_name = jvObj["name"].GetString();
      }
      cmds.push_back(cmd);
    }
    return (cmds.size() > 0) ? true : false;
  }

  static bool parseCmd(const std::string &content, CCmdContent &cmd)
  {
    rapidjson::Document document;
    if (document.Parse(content.c_str()).HasParseError() || !document.HasMember("cmdArray"))
    {
      return false;
    }

    const rapidjson::Value &jvObjArray = document["cmdArray"];
    if(!jvObjArray.IsArray())
    {
      return false;
    }

//    for (size_t i = 0; i < jvObjArray.Size(); ++i)
//    {
//      CBaseContent cmd;
      const rapidjson::Value &jvObj = jvObjArray[0];
      if(!jvObj.IsObject())
      {
        return false;
      }

      if(jvObj.HasMember("seq") && jvObj["seq"].IsUint())
      {
        cmd.m_ctlID = cmd.m_seq = jvObj["seq"].GetUint();
      }
      else
      {
//        continue;
          return false;
      }

      if(jvObj.HasMember("type") && jvObj["type"].IsString())
      {
        cmd.m_type = jvObj["type"].GetString();
      }
      else
      {
//        continue;
          return false;
      }

      if(cmd.m_type.compare(std::string("Double")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsDouble())
        {
          cmd.m_value.m_dval = jvObj["value"].GetDouble();
        }
        else
        {
//          continue;
            return false;
        }
      }
      else if(cmd.m_type.compare(std::string("Int")) == 0)
      {
        if(jvObj.HasMember("value") && jvObj["value"].IsInt())
        {
          cmd.m_value.m_nval = jvObj["value"].GetInt();
        }
        else
        {
//          continue;
            return false;
        }
      }
      else
      {
//        continue;
          return false;
      }

      if(jvObj.HasMember("name") && jvObj["name"].IsString())
      {
        cmd.m_name = jvObj["name"].GetString();
      }

      if(jvObj.HasMember("ctlStat") && jvObj["ctlStat"].IsUint())
      {
        cmd.m_ctlStat = jvObj["ctlStat"].GetUint();
      }
      else
      {
          cmd.m_ctlStat = -1;
//          return false;
      }
//      cmds.push_back(cmd);
//    }
//    return (cmds.size() > 0) ? true : false;
      return true;
  }

};

class CMsgHeader
{
public:
  static public_pkg::status_header_msg getAnalogHeader(const std::string &nodeType,
                                                       const std::string &nodeName)
  {
    public_pkg::status_header_msg msg;
    msg.srcNodeType = nodeType;
    msg.srcNodeName = nodeName;
    msg.msgType = "public_pkg/status_analog_msg";
    msg.msgTime = ros::Time::now();
    return msg;
  }

  static public_pkg::status_header_msg getDigitalHeader(const std::string &nodeType,
                                                       const std::string &nodeName)
  {
    public_pkg::status_header_msg msg;
    msg.srcNodeType = nodeType;
    msg.srcNodeName = nodeName;
    msg.msgType = "public_pkg/status_digital_msg";
    msg.msgTime = ros::Time::now();
    return msg;
  }

  static public_pkg::cmd_header_msg getCmdHeader(uint64_t msgID,
                                                 const std::string &srcNodeType,
                                                 const std::string &srcNodeName,
                                                 const std::string &dstNodeType,
                                                 const std::string &dstNodeName)
  {
    public_pkg::cmd_header_msg msg;
    msg.msgID = msgID;
    msg.srcNodeType = srcNodeType;
    msg.srcNodeName = srcNodeName;
    msg.dstNodeType = dstNodeType;
    msg.dstNodeName = dstNodeName;
    msg.msgType = "public_pkg/cmd_msg";
    msg.msgTime = ros::Time::now();
    return msg;
  }

  static public_pkg::cmd_header_msg getCmdRspHeader(uint64_t msgID,
                                                 const std::string &srcNodeType,
                                                 const std::string &srcNodeName,
                                                 const std::string &dstNodeType,
                                                 const std::string &dstNodeName)
  {
    public_pkg::cmd_header_msg msg;
    msg.msgID = msgID;
    msg.srcNodeType = srcNodeType;
    msg.srcNodeName = srcNodeName;
    msg.dstNodeType = dstNodeType;
    msg.dstNodeName = dstNodeName;
    msg.msgType = "public_pkg/cmd_rsp_msg";
    msg.msgTime = ros::Time::now();
    return msg;
  }

  static bool getMsgValidTime(int &secs)
  {
    return ros::param::get("/nari/szrd/dnrobot/msg/valid_time_secs", secs);
  }

  static bool getNodeInitTime(int &secs)
  {
    return ros::param::get("/nari/szrd/dnrobot/node/init_time_secs", secs);
  }

  static bool getNodeHeartbeatTime(int &secs)
  {
    return ros::param::get("/nari/szrd/dnrobot/node/heart_beat_secs", secs);
  }

};

class CMsgMath
{
public:
  CMsgMath() {}

//  x = rand()%11; /*产生1~10之间的随机整数*/
//  y = rand()%51 - 25; /*产生-25 ~ 25之间的随机整数*/
//  z = ((double)rand()/RAND_MAX)*(b-a) + a;/*产生区间[a,b]上的随机数*/
  static double getRandom(int max)
  {
    srand((int)std::time(0));
    return (double)(std::rand()%max);
  }
};

class CNodeCheck
{
public:
  CNodeCheck() {}

  static bool isNodeReady(ros::Time startTime)
  {
    ros::Time currentTime = ros::Time::now();
    int initSecs;
    if(!CMsgHeader::getNodeInitTime(initSecs))
    {
      initSecs = 5;
    }

    return ((currentTime.toSec() - startTime.toSec()) < initSecs) ? false : true;
  }

  static bool isMsgValid(ros::Time msgTime)
  {
    ros::Time currentTime = ros::Time::now();
    int validSecs;
    if(!CMsgHeader::getMsgValidTime(validSecs))
    {
      validSecs = 5;
    }

    return ((currentTime.toSec() - msgTime.toSec()) > validSecs) ? false : true;
  }
};

}

#endif // MSGWRAPPER_H
