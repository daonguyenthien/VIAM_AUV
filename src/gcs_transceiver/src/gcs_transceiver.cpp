#include "gcs_transceiver.h"

GCSTransceiver::GCSTransceiver()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("main_period", main_period);
  private_nh.getParam("main_url", main_url);

  int autopilot_sysid_, gcs_sysid_;
  private_nh.getParam("autopilot_sysid", autopilot_sysid_);
  private_nh.getParam("gcs_sysid", gcs_sysid_);
  autopilot_sysid = static_cast<uint8_t>(autopilot_sysid_);
  gcs_sysid = static_cast<uint8_t>(gcs_sysid_);

  autopilot_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_AUTOPILOT1);
  gcs_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_ALL);

  MainLink = MAVConnInterface::open_url(main_url);
  MainLink->set_system_id(autopilot_sysid);
  MainLink->set_component_id(autopilot_compid);
  MainLink->message_received_cb = boost::bind(&GCSTransceiver::onMainLinkCallBack, this, _1, _2);

  missionProtocol = std::make_shared<MissionProtocol>(this, MainLink);
  commandProtocol = std::make_shared<CommandProtocol>(this, MainLink);
  parameterProtocol = std::make_shared<ParameterProtocol>(this, MainLink);

  setupHeartbeat();

  ros::NodeHandle nh;
  loopMainLink = nh.createTimer(ros::Duration(main_period), &GCSTransceiver::onMainLinkLoop, this);
  loopGlobalPos = nh.createTimer(ros::Duration(0.1), &GCSTransceiver::onGlobalPosLoop, this);
  subOdom = nh.subscribe("odom", 10, &GCSTransceiver::onOdomCallBack, this);
  subSetpoint = nh.subscribe("setpoint", 10, &GCSTransceiver::onSetpointCallBack, this);
  subMotorsCmd = nh.subscribe("motors/cmd", 10, &GCSTransceiver::onMotorsCmdCallBack, this);
  subError = nh.subscribe("error", 10, &GCSTransceiver::onErrorCallBack, this);
  subThrusterStatus = nh.subscribe("status/thruster", 10, &GCSTransceiver::onThrusterStatusCallBack, this);
  subMassShifterStatus = nh.subscribe("status/mass_shifter", 10, &GCSTransceiver::onMassShifterStatusCallBack, this);
  subPistonStatus = nh.subscribe("status/piston", 10, &GCSTransceiver::onPistonStatusCallBack, this);
  subBoardARM1Status = nh.subscribe("status/board_arm1", 10, &GCSTransceiver::onBoardARM1StatusCallBack, this);
  subBoardARM2Status = nh.subscribe("status/board_arm2", 10, &GCSTransceiver::onBoardARM2StatusCallBack, this);

  initTime = ros::Time::now();
}

GCSTransceiver::~GCSTransceiver()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GCSTransceiver::setupHeartbeat()
{
  Heartbeat.type = static_cast<uint8_t>(MAV_TYPE::GROUND_ROVER);
  Heartbeat.autopilot = static_cast<uint8_t>(MAV_AUTOPILOT::GENERIC);
  Heartbeat.base_mode = static_cast<uint8_t>(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
}

void GCSTransceiver::onMainLinkLoop(const ros::TimerEvent& /*event*/)
{
  pack_and_send_mavlink_message_t(Heartbeat, MainLink);
}

void GCSTransceiver::onGlobalPosLoop(const ros::TimerEvent& event)
{
  std::unique_lock<std::mutex> lock(mutex);
  auto currTime = event.current_real - initTime;

  GLOBAL_POSITION_INT pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.lat = static_cast<int32_t>(lat * 1e7);
  pack.lon = static_cast<int32_t>(lon * 1e7);
  pack.alt = static_cast<int32_t>(alt * 1e3);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing)
{
  if (framing == mavconn::Framing::ok)
  {
    ROS_INFO_STREAM("msgid = " << int(msg->msgid) << ", sysid = " << int(msg->sysid)
                               << ", compid = " << int(msg->compid));

    if (msg->sysid == gcs_sysid && msg->compid == gcs_compid)
    {
      switch (msg->msgid)
      {
      case 44:
      case 73:
      case 43:
      case 51:
      case 41:
      case 45:
        missionProtocol->handleMission(msg);
        break;
      case 76:
      case 75:
      case 11:
        commandProtocol->handleCommand(msg);
        break;
      case 21:
      case 20:
      case 23:
        parameterProtocol->handleParameter(msg);
        break;
      default:
        break;
      }
    }
  }
}

void GCSTransceiver::onOdomCallBack(const Odometry::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_ODOMETRY pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.lat = static_cast<int32_t>(msg->latitude * 1e7);
  pack.lon = static_cast<int32_t>(msg->longitude * 1e7);
  pack.alt = static_cast<float>(msg->altitude);
  pack.x = static_cast<float>(msg->position.x);
  pack.y = static_cast<float>(msg->position.y);
  pack.z = static_cast<float>(msg->position.z);
  pack.vx = static_cast<float>(msg->linear_velocity.x);
  pack.vy = static_cast<float>(msg->linear_velocity.y);
  pack.vz = static_cast<float>(msg->linear_velocity.z);
  pack.roll = static_cast<float>(msg->orientation.x);
  pack.pitch = static_cast<float>(msg->orientation.y);
  pack.yaw = static_cast<float>(msg->orientation.z);
  pack.vroll = static_cast<float>(msg->angular_velocity.x);
  pack.vpitch = static_cast<float>(msg->angular_velocity.y);
  pack.vyaw = static_cast<float>(msg->angular_velocity.z);
  pack.ax = static_cast<float>(msg->linear_acceleration.x);
  pack.ay = static_cast<float>(msg->linear_acceleration.y);
  pack.az = static_cast<float>(msg->linear_acceleration.z);
  pack.aroll = static_cast<float>(msg->angular_acceleration.x);
  pack.apitch = static_cast<float>(msg->angular_acceleration.y);
  pack.ayaw = static_cast<float>(msg->angular_acceleration.z);
  pack_and_send_mavlink_message_t(pack, MainLink);

  {
    std::unique_lock<std::mutex> lock(mutex);
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
  }
}

void GCSTransceiver::onSetpointCallBack(const Setpoint::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_SETPOINT pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.x = static_cast<float>(msg->position.x);
  pack.y = static_cast<float>(msg->position.y);
  pack.z = static_cast<float>(msg->position.z);
  pack.vx = static_cast<float>(msg->linear_velocity.x);
  pack.vy = static_cast<float>(msg->linear_velocity.y);
  pack.vz = static_cast<float>(msg->linear_velocity.z);
  pack.roll = static_cast<float>(msg->orientation.x);
  pack.pitch = static_cast<float>(msg->orientation.y);
  pack.yaw = static_cast<float>(msg->orientation.z);
  pack.vroll = static_cast<float>(msg->angular_velocity.x);
  pack.vpitch = static_cast<float>(msg->angular_velocity.y);
  pack.vyaw = static_cast<float>(msg->angular_velocity.z);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onMotorsCmdCallBack(const MotorsCommand::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_MOTORS_COMMAND pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.thruster_speed = static_cast<float>(msg->thruster_speed);
  pack.rudder_angle = static_cast<float>(msg->rudder_angle);
  pack.mass_shifter_position = static_cast<float>(msg->mass_shifter_position);
  pack.piston_position = static_cast<float>(msg->piston_position);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onErrorCallBack(const Error::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_ERROR pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.along_track = static_cast<float>(msg->along_track);
  pack.cross_track = static_cast<float>(msg->cross_track);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onThrusterStatusCallBack(const ThrusterStatus::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_THRUSTER_STATUS pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.motor_duty = static_cast<float>(msg->motor_duty);
  pack.motor_temp_on_chip = static_cast<float>(msg->motor_temp_on_chip);
  pack.motor_temp_ambient = static_cast<float>(msg->motor_temp_ambient);
  pack.motor_current = static_cast<float>(msg->motor_current);
  pack.motor_rspeed = static_cast<float>(msg->motor_rspeed);
  pack.motor_dspeed = static_cast<float>(msg->motor_dspeed);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onMassShifterStatusCallBack(const MassShifterStatus::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_MASS_SHIFTER_STATUS pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.position = static_cast<float>(msg->position);
  pack.motor_duty = static_cast<float>(msg->motor_duty);
  pack.motor_temp_on_chip = static_cast<float>(msg->motor_temp_on_chip);
  pack.motor_temp_ambient = static_cast<float>(msg->motor_temp_ambient);
  pack.motor_current = static_cast<float>(msg->motor_current);
  pack.motor_rspeed = static_cast<float>(msg->motor_rspeed);
  pack.motor_dspeed = static_cast<float>(msg->motor_dspeed);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onPistonStatusCallBack(const PistonStatus::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_PISTON_STATUS pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.position = static_cast<float>(msg->position);
  pack.motor_duty = static_cast<float>(msg->motor_duty);
  pack.motor_temp_on_chip = static_cast<float>(msg->motor_temp_on_chip);
  pack.motor_temp_ambient = static_cast<float>(msg->motor_temp_ambient);
  pack.motor_current = static_cast<float>(msg->motor_current);
  pack.motor_rspeed = static_cast<float>(msg->motor_rspeed);
  pack.motor_dspeed = static_cast<float>(msg->motor_dspeed);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onBoardARM1StatusCallBack(const BoardARM1Status::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_BOARD_ARM1_STATUS pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.ls_piston_athead = static_cast<uint8_t>(msg->ls_piston_athead);
  pack.ls_piston_attail = static_cast<uint8_t>(msg->ls_piston_attail);
  pack.ls_mass_shifter_athead = static_cast<uint8_t>(msg->ls_mass_shifter_athead);
  pack.ls_mass_shifter_attail = static_cast<uint8_t>(msg->ls_mass_shifter_attail);
  pack.altimeter_in_metres = static_cast<float>(msg->altimeter_in_metres);
  pack.altimeter_in_feet = static_cast<float>(msg->altimeter_in_feet);
  pack.altimeter_in_fathoms = static_cast<float>(msg->altimeter_in_fathoms);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onBoardARM2StatusCallBack(const BoardARM2Status::ConstPtr& msg)
{
  auto currTime = ros::Time::now() - initTime;

  VIAM_BOARD_ARM2_STATUS pack;
  pack.time_boot_ms = static_cast<uint32_t>(currTime.toNSec() * 1e-6);
  pack.rudder_position = static_cast<float>(msg->rudder_position);
  pack.rudder_speed = static_cast<float>(msg->rudder_speed);
  pack.rudder_load = static_cast<float>(msg->rudder_load);
  pack.rudder_voltage = static_cast<float>(msg->rudder_voltage);
  pack.rudder_temperature = static_cast<float>(msg->rudder_temperature);
  pack.keller_pa3_pressure = static_cast<float>(msg->keller_pa3_pressure);
  pack.keller_pa3_temperature = static_cast<float>(msg->keller_pa3_temperature);
  pack_and_send_mavlink_message_t(pack, MainLink);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcs_transceiver");
  GCSTransceiver trans;
  ros::spin();
}
