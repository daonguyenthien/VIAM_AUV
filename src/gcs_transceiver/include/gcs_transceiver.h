#ifndef GCS_TRANSCEIVER_H
#define GCS_TRANSCEIVER_H

#include <ros/ros.h>

#include <utils/Error.h>
#include <utils/Odometry.h>
#include <utils/Setpoint.h>
#include <utils/MotorsCommand.h>
#include <utils/ThrusterStatus.h>
#include <utils/MassShifterStatus.h>
#include <utils/PistonStatus.h>
#include <utils/BoardARM1Status.h>
#include <utils/BoardARM2Status.h>

#include "mavconn/interface.h"
#include "mavconn/mavlink_dialect.h"
#include "utils/geo.h"

#include "command_protocol.h"
#include "mission_protocol.h"
#include "msg_handler.h"
#include "parameter_protocol.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavlink::viamlab_auv;
using namespace mavlink::viamlab_auv::msg;
using namespace utils;

class GCSTransceiver
{
public:
  GCSTransceiver();
  ~GCSTransceiver();

  MAVConnInterface::Ptr MainLink;

  HEARTBEAT Heartbeat;

  uint8_t autopilot_sysid;
  uint8_t autopilot_compid;
  uint8_t gcs_sysid;
  uint8_t gcs_compid;

  double main_period;
  std::string main_url;

  ros::Timer loopMainLink;
  ros::Timer loopGlobalPos;
  ros::Subscriber subOdom;
  ros::Subscriber subSetpoint;
  ros::Subscriber subMotorsCmd;
  ros::Subscriber subError;
  ros::Subscriber subThrusterStatus;
  ros::Subscriber subMassShifterStatus;
  ros::Subscriber subPistonStatus;
  ros::Subscriber subBoardARM1Status;
  ros::Subscriber subBoardARM2Status;

  MissionProtocol::Ptr missionProtocol;
  CommandProtocol::Ptr commandProtocol;
  ParameterProtocol::Ptr parameterProtocol;

  double lat, lon, alt;
  ros::Time initTime;
  std::mutex mutex;

  void setupHeartbeat();

  void onMainLinkLoop(const ros::TimerEvent& event);
  void onGlobalPosLoop(const ros::TimerEvent& event);
  void onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing);

  void onOdomCallBack(const Odometry::ConstPtr& msg);
  void onSetpointCallBack(const Setpoint::ConstPtr& msg);
  void onMotorsCmdCallBack(const MotorsCommand::ConstPtr& msg);
  void onErrorCallBack(const Error::ConstPtr& msg);
  void onThrusterStatusCallBack(const ThrusterStatus::ConstPtr& msg);
  void onMassShifterStatusCallBack(const MassShifterStatus::ConstPtr& msg);
  void onPistonStatusCallBack(const PistonStatus::ConstPtr& msg);
  void onBoardARM1StatusCallBack(const BoardARM1Status::ConstPtr& msg);
  void onBoardARM2StatusCallBack(const BoardARM2Status::ConstPtr& msg);
};

#endif // GCS_TRANSCEIVER_H
