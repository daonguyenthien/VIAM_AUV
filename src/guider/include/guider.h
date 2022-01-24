#ifndef GUIDER_H
#define GUIDER_H

#include <ros/ros.h>

#include <utils/WaypointList.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/Error.h>

#include <utils/CommandLong.h>
#include <utils/ParamGet.h>
#include <utils/ParamSet.h>
#include <utils/SetMode.h>

#include <fstream>
#include <mutex>

#include "utils/geo.h"
#include "straight_los.h"

using namespace sensor_msgs;
using namespace std_msgs;
using namespace utils;

class Guider
{
public:
  Guider();
  ~Guider();

  double ned_lat;
  double ned_lon;
  double guiding_period;

  ros::Publisher pubSetpoint;
  ros::Publisher pubError;
  ros::Subscriber subOdom;
  ros::Subscriber subItemList;
  ros::Timer loopGuidance;

  ros::ServiceServer resStartMission;
  ros::ServiceServer resSetMode;
  ros::ServiceServer resSetSetpointParams;
  ros::ServiceServer resGetSetpointParams;
  ros::ServiceServer resSetLOSParams;
  ros::ServiceServer resGetLOSParams;

  StraightLOS straightLOSGuider;

  double desiredSpeed;
  double desiredHeading;
  double currX;
  double currY;
  double currHeading;

  bool isMissionStarted = false;
  std::string customMode;
  std::mutex mutex;

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onItemListCallBack(const WaypointList::ConstPtr& msg);
  void onGuidanceLoop(const ros::TimerEvent& event);

  bool onStartMissionCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onSetModeCallBack(SetModeRequest& req, SetModeResponse& res);
  bool onSetSetpointParamsCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetSetpointParamsCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool onSetLOSParamsCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetLOSParamsCallBack(ParamGetRequest& req, ParamGetResponse& res);

  void publishSetpoint(const double& speed, const double& heading);
  void publishError(const double& xe, const double& ye);
  inline bool compareString(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }
};

#endif // GUIDER_H
