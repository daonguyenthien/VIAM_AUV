#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>

#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/MotorsCommand.h>
#include <utils/Error.h>

#include <utils/ParamGet.h>
#include <utils/ParamSet.h>

#include <mutex>

#include "utils/math.h"
#include "pid.h"

using namespace utils;
using namespace std;

class Controller
{
public:
  Controller();
  ~Controller();

  double controlling_period;
  double avg_rpm;

  ros::Publisher pubMotorsCmd;
  ros::Subscriber subOdom;
  ros::Subscriber subSetpoint;
  ros::Timer loopControl;

  ros::ServiceServer resSetSpeedPID;
  ros::ServiceServer resGetSpeedPID;
  ros::ServiceServer resSetHeadingPID;
  ros::ServiceServer resGetHeadingPID;

  PID speedPID;
  PID headingPID;

  double currSpeed;
  double currHeading;
  double desiredSpeed;
  double desiredHeading;
  ros::Time lastSetpointTime;
  ros::Time lastControlUpdateTime;

  std::mutex mutex;

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onSetpointCallBack(const Setpoint::ConstPtr& msg);
  void onControlLoop(const ros::TimerEvent& event);

  bool onSetSpeedPIDCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetSpeedPIDCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res);
  bool onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res);

  inline bool compareString(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }
};

#endif // CONTROLLER_H
