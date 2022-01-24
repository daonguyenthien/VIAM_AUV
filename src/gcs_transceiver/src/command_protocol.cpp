#include "command_protocol.h"
#include "gcs_transceiver.h"

CommandProtocol::CommandProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link) : link(link), trans(trans)
{
  ros::NodeHandle nh;
  reqSetArming = nh.serviceClient<CommandLong>("command/set_arming");
  reqStartMission = nh.serviceClient<CommandLong>("command/start_mission");
  reqSetMode = nh.serviceClient<SetMode>("command/set_mode");
}

void CommandProtocol::handleCommand(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 76: // COMMAND_LONG
    handleCommandLong(msg);
    break;
  case 75: // COMMAND_INT
    handleCommandInt(msg);
    break;
  case 11: // SET_MODE
    handleSetMode(msg);
    break;
  }
}

void CommandProtocol::handleCommandLong(const mavlink_message_t* msg)
{
  COMMAND_LONG reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    switch (reqPack.command)
    {
    case static_cast<uint16_t>(MAV_CMD::REQUEST_AUTOPILOT_CAPABILITIES):
      commandRequestAutopilotCapabilities(reqPack);
      break;
    case static_cast<uint16_t>(MAV_CMD::REQUEST_PROTOCOL_VERSION):
      commandRequestProtocolVersion(reqPack);
      break;
    case static_cast<uint16_t>(MAV_CMD::SET_MESSAGE_INTERVAL):
      commandSetMessageInterval(reqPack);
      break;
    case static_cast<uint16_t>(MAV_CMD::COMPONENT_ARM_DISARM):
      commandComponentArmDisarm(reqPack);
      break;
    case static_cast<uint16_t>(MAV_CMD::MISSION_START):
      commandMissionStart(reqPack);
      break;
    }
  }
}

void CommandProtocol::handleCommandInt(const mavlink_message_t* /*msg*/) {}

void CommandProtocol::handleSetMode(const mavlink_message_t* msg)
{
  SET_MODE reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  if (reqPack.target_system == trans->autopilot_sysid)
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    SetMode srv;
    srv.request.base_mode = reqPack.base_mode;
    srv.request.custom_mode = std::to_string(reqPack.custom_mode);

    if (reqSetMode.call(srv))
    {
      sendCommandACK(srv.request.base_mode, srv.response.mode_sent);

      if (srv.response.mode_sent == static_cast<uint8_t>(MAV_RESULT::ACCEPTED))
      {
        trans->Heartbeat.base_mode = reqPack.base_mode;
        trans->Heartbeat.custom_mode = reqPack.custom_mode;
      }
    }
  }
}

void CommandProtocol::commandRequestAutopilotCapabilities(const COMMAND_LONG& reqPack)
{
  sendCommandACK(reqPack.command, static_cast<uint8_t>(MAV_RESULT::ACCEPTED));

  AUTOPILOT_VERSION resPack;
  resPack.capabilities = static_cast<uint64_t>(MAV_PROTOCOL_CAPABILITY::MISSION_INT) |
                          static_cast<uint64_t>(MAV_PROTOCOL_CAPABILITY::MAVLINK2);
  pack_and_send_mavlink_message_t(resPack, link);
}

void CommandProtocol::commandRequestProtocolVersion(const COMMAND_LONG& reqPack)
{
  sendCommandACK(reqPack.command, static_cast<uint8_t>(MAV_RESULT::ACCEPTED));

  PROTOCOL_VERSION resPack;
  resPack.version = 200;
  resPack.min_version = 200;
  resPack.max_version = 200;
  pack_and_send_mavlink_message_t(resPack, link);
}

void CommandProtocol::commandSetMessageInterval(const COMMAND_LONG& reqPack)
{
  // Trick to allow messages accepted in MAVSDK
  sendCommandACK(reqPack.command, static_cast<uint8_t>(MAV_RESULT::ACCEPTED));
}

void CommandProtocol::commandComponentArmDisarm(const COMMAND_LONG& reqPack)
{
  CommandLong srv;
  serviceCommandLong(reqPack, srv);

  if (reqSetArming.call(srv))
  {
    sendCommandACK(srv.request.command, srv.response.result);

    if (srv.response.result == static_cast<uint8_t>(MAV_RESULT::ACCEPTED))
    {
      const uint8_t arm_flag = static_cast<uint8_t>(MAV_MODE_FLAG::SAFETY_ARMED);
      if (srv.request.param1 == 1.0f)
        trans->Heartbeat.base_mode |= arm_flag;
      else if (srv.request.param1 == 0.0f)
        trans->Heartbeat.base_mode &= ~arm_flag;
    }
  }
}

void CommandProtocol::commandMissionStart(const COMMAND_LONG& reqPack)
{
  CommandLong srv;
  serviceCommandLong(reqPack, srv);

  if (reqStartMission.call(srv))
    sendCommandACK(srv.request.command, srv.response.result);
}

void CommandProtocol::sendCommandACK(uint16_t command, uint8_t result)
{
  COMMAND_ACK pack;
  pack.command = command;
  pack.result = result;
  pack_and_send_mavlink_message_t(pack, link);
}

void CommandProtocol::serviceCommandLong(const COMMAND_LONG& reqPack, CommandLong& srv)
{
  srv.request.command = reqPack.command;
  srv.request.confirmation = reqPack.confirmation;
  srv.request.param1 = reqPack.param1;
  srv.request.param2 = reqPack.param2;
  srv.request.param3 = reqPack.param3;
  srv.request.param4 = reqPack.param4;
  srv.request.param5 = reqPack.param5;
  srv.request.param6 = reqPack.param6;
  srv.request.param7 = reqPack.param7;
}
