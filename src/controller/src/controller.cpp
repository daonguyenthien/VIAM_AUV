#include "controller.h"

Controller::Controller()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("controlling_period", controlling_period);
  private_nh.getParam("avg_rpm", avg_rpm);
  private_nh.getParam("speed_Kp", speedPID.Kp);
  private_nh.getParam("speed_Ki", speedPID.Ki);
  private_nh.getParam("speed_Kd", speedPID.Kd);
  private_nh.getParam("heading_Kp", headingPID.Kp);
  private_nh.getParam("heading_Ki", headingPID.Ki);
  private_nh.getParam("heading_Kd", headingPID.Kd);

  ros::NodeHandle nh;
  pubMotorsCmd = nh.advertise<MotorsCommand>("motors/cmd", 1);
  subOdom = nh.subscribe("odom", 10, &Controller::onOdomCallBack, this);
  subSetpoint = nh.subscribe("setpoint", 10, &Controller::onSetpointCallBack, this);
  loopControl = nh.createTimer(ros::Duration(controlling_period), &Controller::onControlLoop, this);

  resSetSpeedPID = nh.advertiseService("parameter/set_speed_PID", &Controller::onSetSpeedPIDCallBack, this);
  resGetSpeedPID = nh.advertiseService("parameter/get_speed_PID", &Controller::onGetSpeedPIDCallBack, this);
  resSetHeadingPID = nh.advertiseService("parameter/set_heading_PID", &Controller::onSetHeadingPIDCallBack, this);
  resGetHeadingPID = nh.advertiseService("parameter/get_heading_PID", &Controller::onGetHeadingPIDCallBack, this);

  lastControlUpdateTime = lastSetpointTime = ros::Time::now();
  speedPID.threshold = avg_rpm;
  headingPID.threshold = 60.0;
}

Controller::~Controller()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void Controller::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(mutex);

  currSpeed = msg->linear_velocity.x;
  currHeading = msg->orientation.z;
}

void Controller::onSetpointCallBack(const Setpoint::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(mutex);

  lastSetpointTime = msg->header.stamp;
  desiredSpeed = msg->linear_velocity.x;
  desiredHeading = msg->orientation.z;
}

void Controller::onControlLoop(const ros::TimerEvent& event)
{
  std::unique_lock<std::mutex> lock(mutex);

  double dtc = (event.current_real - lastSetpointTime).toSec();
  if (dtc > 0.5)
    return;

  double dt = event.current_real.toSec() - lastControlUpdateTime.toSec();
  speedPID.error = desiredSpeed - currSpeed;
  speedPID.Ts = dt;
  speedPID.runPID();
  headingPID.error = atan2(sin(desiredHeading - currHeading), cos(desiredHeading - currHeading));
  headingPID.Ts = dt;
  headingPID.runPID();

  MotorsCommand motorsMsg;
  motorsMsg.header.stamp = event.current_real;
  motorsMsg.thruster_speed = avg_rpm + speedPID.output;
  motorsMsg.rudder_angle = headingPID.output;
  motorsMsg.mass_shifter_position = 0.0;
  pubMotorsCmd.publish(motorsMsg);

  lastControlUpdateTime = event.current_real;
}

bool Controller::onSetSpeedPIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "speed_Kp"))
  {
    speedPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "speed_Ki"))
  {
    speedPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "speed_Kd"))
  {
    speedPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Controller::onGetSpeedPIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "speed_Kp"))
  {
    res.value.real = speedPID.Kp;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "speed_Ki"))
  {
    res.value.real = speedPID.Ki;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "speed_Kd"))
  {
    res.value.real = speedPID.Kd;
    res.success = true;
  }

  return res.success;
}

bool Controller::onSetHeadingPIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "heading_Kp"))
  {
    headingPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Ki"))
  {
    headingPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Kd"))
  {
    headingPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool Controller::onGetHeadingPIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "heading_Kp"))
  {
    res.value.real = headingPID.Kp;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Ki"))
  {
    res.value.real = headingPID.Ki;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "heading_Kd"))
  {
    res.value.real = headingPID.Kd;
    res.success = true;
  }

  return res.success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  Controller control;
  ros::spin();
}
