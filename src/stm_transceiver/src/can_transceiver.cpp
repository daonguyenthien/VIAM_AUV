#include "can_transceiver.h"

#include <mutex>

static std::mutex mutexMonitoringData;

char generateChecksum(char* data, int start, int count)
{
  int value = 0;
  for (int i = start; i < (count + start); i++)
    value += data[i];
  value = ~value;
  value++;
  return char(value);
}

void convertFloatToBytes(float input, char* output)
{
  union {
    float value;
    char bytes[4];
  } temp;
  temp.value = input;
  output[0] = temp.bytes[3];
  output[1] = temp.bytes[2];
  output[2] = temp.bytes[1];
  output[3] = temp.bytes[0];
}

void convertBytesToFloat(char* input, float* output)
{
  union {
    float value;
    char bytes[4];
  } temp;
  temp.bytes[0] = input[0];
  temp.bytes[1] = input[1];
  temp.bytes[2] = input[2];
  temp.bytes[3] = input[3];
  *output = temp.value;
}

template <typename T> void parseMotorStatus(char* data, T msg)
{
  std::string identifier = {data[0], data[1], data[2]};
  float value;
  convertBytesToFloat(&data[3], &value);
  if (identifier == "RPV")
    msg.motor_rspeed = double(value);
  else if (identifier == "AOC")
    msg.motor_temp_on_chip = double(value);
  else if (identifier == "ATK")
    msg.motor_temp_ambient = double(value);
  else if (identifier == "AIM")
    msg.motor_current = double(value);
  else if (identifier == "TDC")
    msg.motor_duty = double(value);
  else if (identifier == "RSP")
    msg.motor_dspeed = double(value);
}

CANTransceiverNode::CANTransceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("enabled", enabled);
  private_nh.getParam("monitoring_period", monitoring_period);
  private_nh.getParam("port", port);

  ros::NodeHandle nh;
  pubThrusterStatus = nh.advertise<ThrusterStatus>("status/thruster", 1);
  pubMassShifterStatus = nh.advertise<MassShifterStatus>("status/mass_shifter", 1);
  pubPistonStatus = nh.advertise<PistonStatus>("status/piston", 1);
  pubBoardARM1Status = nh.advertise<BoardARM1Status>("status/board_arm1", 1);
  pubBoardARM2Status = nh.advertise<BoardARM2Status>("status/board_arm2", 1);
  subMotorsCmd = nh.subscribe("motors/cmd", 10, &CANTransceiverNode::onMotorsCmdCallBack, this);
  resSetArming = nh.advertiseService("command/set_arming", &CANTransceiverNode::onSetArmingCallBack, this);
  loopMonitoringData =
      nh.createTimer(ros::Duration(monitoring_period), &CANTransceiverNode::onMonitoringDataLoopCallBack, this);
}

CANTransceiverNode::~CANTransceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void CANTransceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void CANTransceiverNode::onMotorsCmdCallBack(const MotorsCommand::ConstPtr& msg)
{
  auto currThrusterSpeed = msg->thruster_speed;
  auto currRudderAngle = msg->rudder_angle;
  if (motorsLocked)
    currThrusterSpeed = 0;

  if (currThrusterSpeed != preThrusterSpeed)
  {
    sendThrusterSpeed(M_CCW, currThrusterSpeed);
    preThrusterSpeed = currThrusterSpeed;
  }
  if (currRudderAngle != preRudderAngle)
  {
    sendRudderAngle(currRudderAngle);
    preRudderAngle = currRudderAngle;
  }
}

bool CANTransceiverNode::onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res)
{
  if (req.param1 == 1.0f)
  {
    motorsLocked = false;
    ROS_INFO("Motors unlocked.");
  }
  else if (req.param1 == 0.0f)
  {
    motorsLocked = true;
    ROS_INFO("Motors locked.");
  }
  res.result = 0; // MAV_RESULT_ACCEPTED
  res.success = true;

  return res.success;
}

void CANTransceiverNode::onMonitoringDataLoopCallBack(const ros::TimerEvent& /*event*/)
{
  std::unique_lock<std::mutex> lock(mutexMonitoringData);

  requestMonitoringData(AUV_ARMBOARD_1);
  requestMonitoringData(AUV_ARMBOARD_2);
  requestMonitoringData(AUV_THRUSTER);
  requestMonitoringData(AUV_MASS_SHIFTER);
  requestMonitoringData(AUV_PISTON);

  pubThrusterStatus.publish(thrusterStatusMsg);
  pubMassShifterStatus.publish(massShifterStatusMsg);
  pubPistonStatus.publish(pistonStatusMsg);
  pubBoardARM1Status.publish(boardARM1StatusMsg);
  pubBoardARM2Status.publish(boardARM2StatusMsg);
}

void CANTransceiverNode::sendMotorState(AUV_BOARD_TypeDef id, Function_Cmd_TypeDef state)
{
  QByteArray payload;
  payload.resize(8);
  payload[0] = 'C';
  payload[1] = 'A';
  payload[2] = 'N';
  payload[3] = (state == ENABLE) ? 'O' : 'L';
  payload[4] = payload[5] = payload[6] = 0x00;
  payload[7] = generateChecksum(payload.data(), 0, 7);

  QCanBusFrame frame(id, payload);
  emit needToTransmitFrame(frame);
}

void CANTransceiverNode::sendThrusterSpeed(Motor_Direction_TypeDef direction, double speed)
{
  QByteArray payload;
  payload.resize(8);
  payload[0] = 'C';
  payload[1] = 'P';
  payload[2] = direction;
  convertFloatToBytes(float(speed), &payload.data()[3]);
  payload[7] = generateChecksum(payload.data(), 0, 7);

  QCanBusFrame frame(AUV_THRUSTER, payload);
  emit needToTransmitFrame(frame);
}

void CANTransceiverNode::sendRudderAngle(double angle)
{
  QByteArray payload;
  payload.resize(8);
  payload[0] = ARM2_RUDDER;
  payload[1] = WRITE_DATA;
  payload[2] = ARM2_MX28_GOAL_POSITION;
  auto pulses = uint16_t((float(angle) + 180.0f + 45.0f) * 4095.0f / 360.0f);
  payload[3] = char(pulses & 0x00FF);
  payload[4] = char((pulses & 0xFF00) >> 8);
  payload[5] = 0x00;
  payload[6] = 0x00;
  payload[7] = generateChecksum(payload.data(), 0, 7);

  QCanBusFrame frame(AUV_ARMBOARD_2, payload);
  emit needToTransmitFrame(frame);
}

void CANTransceiverNode::requestMonitoringData(AUV_BOARD_TypeDef id)
{
  QByteArray payload;
  payload.resize(8);
  switch (id)
  {
  case AUV_ARMBOARD_1:
  {
    payload[0] = char(ARM1_ALL_DATA);
    payload[1] = READ_DATA;
    payload[2] = payload[3] = payload[4] = payload[5] = 0x00;
    payload[6] = 0x0A;
    break;
  }
  case AUV_ARMBOARD_2:
  {
    payload[0] = char(ARM2_ALL_DATA);
    payload[1] = READ_DATA;
    payload[2] = payload[3] = payload[4] = payload[5] = 0x00;
    payload[6] = 0x0A;
    break;
  }
  case AUV_THRUSTER:
  case AUV_MASS_SHIFTER:
  case AUV_PISTON:
  {
    payload[0] = 'R';
    payload[1] = 'E';
    payload[2] = 'Q';
    payload[3] = 'A';
    payload[4] = 'L';
    payload[5] = 'L';
    payload[6] = 0x0A;
    break;
  }
  default:
    return;
  }

  QCanBusFrame frame(id, payload);
  emit needToTransmitFrame(frame);
}

CANTransceiver::CANTransceiver()
{
  if (node.enabled)
  {
    device = std::shared_ptr<QCanBusDevice>(QCanBus::instance()->createDevice("socketcan", node.port.data()));
    device->connectDevice();
    connect(&node, &CANTransceiverNode::needToTransmitFrame, this, &CANTransceiver::transmitFrame);
    connect(device.get(), &QCanBusDevice::framesReceived, this, &CANTransceiver::processFrame);
  }

  node.start();
  if (node.isRunning())
  {
    node.sendMotorState(AUV_THRUSTER, ENABLE);
    node.sendMotorState(AUV_MASS_SHIFTER, ENABLE);
    node.sendMotorState(AUV_PISTON, ENABLE);
  }
}

CANTransceiver::~CANTransceiver() { device->disconnectDevice(); }

void CANTransceiver::transmitFrame(const QCanBusFrame& frame) { device->writeFrame(frame); }

void CANTransceiver::processFrame()
{
  std::unique_lock<std::mutex> lock(mutexMonitoringData);

  auto frame = device->readFrame();
  auto id = frame.frameId();
  auto payload = frame.payload();

  if (payload[7] != generateChecksum(payload.data(), 0, 7))
    return;

  if (id == AUV_THRUSTER)
    parseMotorStatus<ThrusterStatus>(payload.data(), node.thrusterStatusMsg);
  else if (id == AUV_MASS_SHIFTER)
    parseMotorStatus<MassShifterStatus>(payload.data(), node.massShifterStatusMsg);
  else if (id == AUV_PISTON)
    parseMotorStatus<PistonStatus>(payload.data(), node.pistonStatusMsg);

  if (id == AUV_ARMBOARD_1)
  {
    auto data = payload.data();
    if (data[1] == STATUS_DATA)
    {
      switch (payload.data()[0])
      {
      case ARM1_PISTON:
      {
        if (data[2] == ARM1_REQ_ENCODER)
        {
          float position;
          convertBytesToFloat(&data[3], &position);
          node.pistonStatusMsg.position = double(position);
        }
        else if (payload.data()[2] == ARM1_REQ_PISTON_LIMIT_SWITCH)
        {
          node.boardARM1StatusMsg.ls_piston_athead = uint8_t(data[3]);
          node.boardARM1StatusMsg.ls_piston_attail = uint8_t(data[4]);
        }
        break;
      }
      case ARM1_MASS_SHIFTER:
      {
        if (data[2] == ARM1_REQ_ENCODER)
        {
          float position;
          convertBytesToFloat(&data[3], &position);
          node.massShifterStatusMsg.position = double(position);
        }
        else if (payload.data()[2] == ARM1_REQ_MASS_LIMIT_SWITCH)
        {
          node.boardARM1StatusMsg.ls_mass_shifter_athead = uint8_t(data[3]);
          node.boardARM1StatusMsg.ls_mass_shifter_attail = uint8_t(data[4]);
        }
        break;
      }
      case ARM1_BMS24V40AH:
      {
        break;
      }
      case ARM1_ALTIMETER:
      {
        float alt;
        convertBytesToFloat(&data[3], &alt);
        if (data[2] == ARM1_ALTIMETER_IN_FEET)
          node.boardARM1StatusMsg.altimeter_in_feet = double(alt);
        else if (data[2] == ARM1_ALTIMETER_IN_METRES)
          node.boardARM1StatusMsg.altimeter_in_metres = double(alt);
        else if (data[2] == ARM1_ALTIMETER_IN_FATHOMS)
          node.boardARM1StatusMsg.altimeter_in_fathoms = double(alt);
        break;
      }
      }
    }
  }

  if (id == AUV_ARMBOARD_2)
  {
    auto data = payload.data();
    if (data[1] == STATUS_DATA)
    {
      switch (payload.data()[0])
      {
      case ARM2_RUDDER:
      {
        float value;
        convertBytesToFloat(&data[3], &value);
        if (data[2] == ARM2_MX28_PRESENT_POSITION)
          node.boardARM2StatusMsg.rudder_position = double(value);
        else if (data[2] == ARM2_MX28_PRESENT_SPEED)
          node.boardARM2StatusMsg.rudder_speed = double(value);
        else if (data[2] == ARM2_MX28_PRESENT_LOAD)
          node.boardARM2StatusMsg.rudder_load = double(value);
        else if (data[2] == ARM2_MX28_PRESENT_VOL)
          node.boardARM2StatusMsg.rudder_voltage = double(value);
        else if (data[2] == ARM2_MX28_PRESENT_TEMP)
          node.boardARM2StatusMsg.rudder_temperature = double(value);
        break;
      }
      case ARM2_BMS24V40AH:
      {
        break;
      }
      case ARM2_PRESSURE:
      {
        float press;
        convertBytesToFloat(&data[3], &press);
        if (data[2] == char(ARM2_DEPTH_DATA))
          node.boardARM2StatusMsg.keller_pa3_pressure = double(press);
        else if (data[2] == char(ARM2_TEMP_DATA))
          node.boardARM2StatusMsg.keller_pa3_temperature = double(press);
        break;
      }
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_transceiver");
  QCoreApplication a(argc, argv);
  CANTransceiver transceiver;
  return a.exec();
}
