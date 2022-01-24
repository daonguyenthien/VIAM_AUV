#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <ros/ros.h>

#include <utils/CommandInt.h>
#include <utils/CommandLong.h>
#include <utils/SetMode.h>

#include "mavconn/interface.h"
#include "msg_handler.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace utils;

class GCSTransceiver;

class CommandProtocol
{
public:
  using Ptr = std::shared_ptr<CommandProtocol>;

  CommandProtocol();
  CommandProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  void handleCommand(const mavlink_message_t* msg);

private:
  ros::ServiceClient reqSetArming;
  ros::ServiceClient reqStartMission;
  ros::ServiceClient reqSetMode;

  MAVConnInterface::Ptr link;
  GCSTransceiver* trans;

  void handleCommandLong(const mavlink_message_t* msg);
  void handleCommandInt(const mavlink_message_t* msg);
  void handleSetMode(const mavlink_message_t* msg);

  void commandRequestAutopilotCapabilities(const COMMAND_LONG& reqPack);
  void commandRequestProtocolVersion(const COMMAND_LONG& reqPack);
  void commandSetMessageInterval(const COMMAND_LONG& reqPack);
  void commandComponentArmDisarm(const COMMAND_LONG& reqPack);
  void commandMissionStart(const COMMAND_LONG& reqPack);

  void sendCommandACK(uint16_t command, uint8_t result);
  void serviceCommandLong(const COMMAND_LONG& reqPack, CommandLong& srv);
};

#endif // COMMAND_PROTOCOL_H
