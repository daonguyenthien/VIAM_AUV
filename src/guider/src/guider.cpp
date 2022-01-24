#include "guider.h"

Guider::Guider()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("guiding_period", guiding_period);
  private_nh.getParam("set_speed", desiredSpeed);
  private_nh.getParam("set_heading", desiredHeading);
  private_nh.getParam("LOS_radius", BaseLOS::radius);
  private_nh.getParam("LOS_min_delta", BaseLOS::minDelta);
  private_nh.getParam("LOS_max_delta", BaseLOS::maxDelta);
  private_nh.getParam("LOS_beta", BaseLOS::beta);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubSetpoint = nh.advertise<Setpoint>("setpoint", 1);
  pubError = nh.advertise<Error>("error", 1);
  subOdom = nh.subscribe("odom", 10, &Guider::onOdomCallBack, this);
  subItemList = nh.subscribe("mission/item_list", 1, &Guider::onItemListCallBack, this);
  loopGuidance = nh.createTimer(ros::Duration(guiding_period), &Guider::onGuidanceLoop, this);

  resStartMission = nh.advertiseService("command/start_mission", &Guider::onStartMissionCallBack, this);
  resSetMode = nh.advertiseService("command/set_mode", &Guider::onSetModeCallBack, this);
  resSetSetpointParams =
      nh.advertiseService("parameter/set_setpoint_params", &Guider::onSetSetpointParamsCallBack, this);
  resGetSetpointParams =
      nh.advertiseService("parameter/get_setpoint_params", &Guider::onGetSetpointParamsCallBack, this);
  resSetLOSParams = nh.advertiseService("parameter/set_LOS_params", &Guider::onSetLOSParamsCallBack, this);
  resGetLOSParams = nh.advertiseService("parameter/get_LOS_params", &Guider::onGetLOSParamsCallBack, this);
}

Guider::~Guider()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void Guider::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(mutex);

  currX = msg->position.x;
  currY = msg->position.y;
  currHeading = msg->orientation.z;
}

void Guider::onItemListCallBack(const WaypointList::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(mutex);

  straightLOSGuider.resetLOS();

  for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
  {
    switch (it->command)
    {
    case 16: // MAV_CMD_NAV_WAYPOINT
    {
      double x, y;
      convert_global_to_local_coords(it->x_lat * 1e-7, it->y_long * 1e-7, ned_lat, ned_lon, x, y);
      ROS_INFO_STREAM("Waypoint: x = " << x << ", y = " << y);
      BaseLOS::waypoints.push_back(BaseLOS::Point(x, y));

      break;
    }
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
    {
      BaseLOS::waypoints.push_back(BaseLOS::waypoints[0]);
      break;
    }
    }
  }

  straightLOSGuider.setupLOS();
}

void Guider::onGuidanceLoop(const ros::TimerEvent& /*event*/)
{
  std::unique_lock<std::mutex> lock(mutex);

  if (!isMissionStarted)
    return;

  if (customMode == "1") // AUTO_HEADING
    publishSetpoint(desiredSpeed, desiredHeading);

  if (customMode == "2") // LOS_STRAIGHT
  {
    if (straightLOSGuider.runLOS(currX, currY))
    {
      publishSetpoint(desiredSpeed, straightLOSGuider.desiredHeading);
      publishError(straightLOSGuider.alongTrackError, straightLOSGuider.crossTrackError);
    }
    else
      isMissionStarted = false;
  }
}

bool Guider::onStartMissionCallBack(CommandLongRequest& /*req*/, CommandLongResponse& res)
{
  isMissionStarted = true;
  res.result = 0; // MAV_RESULT_ACCEPTED
  res.success = true;
  ROS_INFO("Mission started");

  return res.success;
}

bool Guider::onSetModeCallBack(SetModeRequest& req, SetModeResponse& res)
{
  customMode = req.custom_mode;
  res.mode_sent = 0;
  res.success = true;

  return res.success;
}

bool Guider::onSetSetpointParamsCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "set_speed"))
  {
    desiredSpeed = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "set_heading"))
  {
    desiredHeading = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Guider::onGetSetpointParamsCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "set_speed"))
  {
    res.value.real = desiredSpeed;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "set_heading"))
  {
    res.value.real = desiredHeading;
    res.success = true;
  }

  return res.success;
}

bool Guider::onSetLOSParamsCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_radius"))
  {
    BaseLOS::radius = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_min_delta"))
  {
    BaseLOS::minDelta = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_max_delta"))
  {
    BaseLOS::maxDelta = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_beta"))
  {
    BaseLOS::beta = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Guider::onGetLOSParamsCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_radius"))
  {
    res.value.real = BaseLOS::radius;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_min_delta"))
  {
    res.value.real = BaseLOS::minDelta;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_max_delta"))
  {
    res.value.real = BaseLOS::maxDelta;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_beta"))
  {
    res.value.real = BaseLOS::beta;
    res.success = true;
  }

  return res.success;
}

void Guider::publishSetpoint(const double& speed, const double& heading)
{
  Setpoint setpointMsg;
  setpointMsg.header.stamp = ros::Time::now();
  setpointMsg.linear_velocity.x = speed;
  setpointMsg.orientation.z = heading;
  pubSetpoint.publish(setpointMsg);
}

void Guider::publishError(const double& xe, const double& ye)
{
  Error errorMsg;
  errorMsg.header.stamp = ros::Time::now();
  errorMsg.along_track = xe;
  errorMsg.cross_track = ye;
  pubError.publish(errorMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guider");
  Guider guide;
  ros::spin();
}
