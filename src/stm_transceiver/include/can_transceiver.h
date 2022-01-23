#ifndef CAN_TRANSCEIVER_H
#define CAN_TRANSCEIVER_H

#include <ros/ros.h>

#include <utils/BoardARM1Status.h>
#include <utils/BoardARM2Status.h>
#include <utils/CommandLong.h>
#include <utils/MassShifterStatus.h>
#include <utils/MotorsCommand.h>
#include <utils/PistonStatus.h>
#include <utils/ThrusterStatus.h>

#include <QCanBus>
#include <QCoreApplication>
#include <QObject>
#include <QThread>

using namespace utils;

typedef enum {
  M_CW = 'R',
  M_CCW = 'L',
  M_NONE = 0x00,
} Motor_Direction_TypeDef;

typedef enum {
  WRITE_DATA = 0x00,
  READ_DATA = 0x01,
  STATUS_DATA = 0x02,
} Function_CANBUS_TypeDef;

typedef enum {
  DISABLE = 0x00,
  ENABLE = 0x01,
} Function_Cmd_TypeDef;

typedef enum {
  AUV_ARMBOARD_1 = 0x121,
  AUV_ARMBOARD_2 = 0x122,
  AUV_MASS_SHIFTER = 0x123,
  AUV_PISTON = 0x124,
  AUV_THRUSTER = 0x125,
  AUV_EPC = 0x126,
  AUV_BLACKBOX = 0x127,
} AUV_BOARD_TypeDef;

typedef enum {
  ARM1_MASS_SHIFTER = 0x00,
  ARM1_PISTON = 0x01,
  ARM1_LEAK_SENSOR = 0x02,
  ARM1_STRAIN_GAUGE = 0x03,
  ARM1_BMS24V10AH = 0x04,
  ARM1_BMS24V40AH = 0x05,
  ARM1_ALTIMETER = 0x06,
  ARM1_POWER_INT = 0x07,
  ARM1_LIGHT = 0x08,
  ARM1_ALL_DATA = 0xFF,
} ARM1_ID_CANBUS_TypeDef;

typedef enum {
  ARM1_LIGHT_ENABLE = 0xFF,
  ARM1_REQ_ENCODER = 0x10,
  ARM1_REQ_PISTON_LIMIT_SWITCH = 0x11,
  ARM1_REQ_MASS_LIMIT_SWITCH = 0x12,
  ARM1_LEAK_POSITION = 0xFF,
  ARM1_STATUS_HULL = 0xFF,
  ARM1_HOURS = 0x01,
  ARM1_MINUTES = 0x02,
  ARM1_SECONDS = 0x03,
  ARM1_BATTERY_TOTAL = 0x04,
  ARM1_BATTERY_CAPACITY = 0x05,
  ARM1_BATTERY_USED = 0x06,
  ARM1_BATTERY_PERCENTAGE = 0x07,
  ARM1_BATTERY_CURRENT = 0x08,
  ARM1_BATTERY_VOLTAGE = 0x09,
  ARM1_ALTIMETER_IN_FEET = 0x01,
  ARM1_ALTIMETER_IN_METRES = 0x02,
  ARM1_ALTIMETER_IN_FATHOMS = 0x03,
  ARM1_INT_24V40AH = 0xFE,
  ARM1_INT_24V10AH = 0xFF,
} ARM1_REGISTER_CANBUS_TypeDef;

typedef enum {
  ARM2_LEAK_SENSOR = 0x01,
  ARM2_STRAIN_GAUGE = 0x02,
  ARM2_BMS24V40AH = 0x03,
  ARM2_RUDDER = 0x04,
  ARM2_POWER_INT = 0x05,
  ARM2_PRESSURE = 0x06,
  ARM2_ALL_DATA = 0xFF,
} ARM2_CANBUS_TypeDef;

typedef enum {
  ARM2_LEAK_POSITION = 0xFF,
  ARM2_STATUS_HULL = 0xFF,
  ARM2_HOURS = 0x01,
  ARM2_MINUTES = 0x02,
  ARM2_SECONDS = 0x03,
  ARM2_BATTERY_TOTAL = 0x04,
  ARM2_BATTERY_CAPACITY = 0x05,
  ARM2_BATTERY_USED = 0x06,
  ARM2_BATTERY_PERCENTAGE = 0x07,
  ARM2_BATTERY_CURRENT = 0x08,
  ARM2_BATTERY_VOLTAGE = 0x09,
  ARM2_MX28_GOAL_POSITION = 0x00,
  ARM2_MX28_MOVING_SPEED = 0x01,
  ARM2_MX28_KP = 0x02,
  ARM2_MX28_KI = 0x03,
  ARM2_MX28_KD = 0x04,
  ARM2_MX28_BAUDRATE = 0x05,
  ARM2_MX28_PRESENT_POSITION = 0x06,
  ARM2_MX28_PRESENT_SPEED = 0x07,
  ARM2_MX28_PRESENT_LOAD = 0x08,
  ARM2_MX28_PRESENT_VOL = 0x09,
  ARM2_MX28_PRESENT_TEMP = 0x0A,
  ARM2_DEPTH_DATA = 0xFE,
  ARM2_TEMP_DATA = 0xFF,
  ARM2_INT_24V40AH = 0xFE,
  ARM2_INT_24V10AH = 0xFF,
} ARM2_REGISTER_CANBUS_TypeDef;

class CANTransceiverNode : public QThread
{
  Q_OBJECT

public:
  CANTransceiverNode();
  ~CANTransceiverNode();

  bool enabled;
  double monitoring_period;
  std::string port;

  ros::Publisher pubThrusterStatus;
  ros::Publisher pubMassShifterStatus;
  ros::Publisher pubPistonStatus;
  ros::Publisher pubBoardARM1Status;
  ros::Publisher pubBoardARM2Status;
  ros::Subscriber subMotorsCmd;
  ros::ServiceServer resSetArming;
  ros::Timer loopMonitoringData;

  bool motorsLocked = true;
  double preThrusterSpeed = 0;
  double preRudderAngle = 0;
  ThrusterStatus thrusterStatusMsg;
  MassShifterStatus massShifterStatusMsg;
  PistonStatus pistonStatusMsg;
  BoardARM1Status boardARM1StatusMsg;
  BoardARM2Status boardARM2StatusMsg;

  void run();

  void onMotorsCmdCallBack(const MotorsCommand::ConstPtr& msg);
  bool onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res);
  void onMonitoringDataLoopCallBack(const ros::TimerEvent& event);

  void sendMotorState(AUV_BOARD_TypeDef id, Function_Cmd_TypeDef state);
  void sendThrusterSpeed(Motor_Direction_TypeDef direction, double speed);
  void sendRudderAngle(double angle);
  void requestMonitoringData(AUV_BOARD_TypeDef id);

signals:
  void needToTransmitFrame(QCanBusFrame frame);
};

class CANTransceiver : public QObject
{
  Q_OBJECT

public:
  CANTransceiver();
  ~CANTransceiver();

  CANTransceiverNode node;
  std::shared_ptr<QCanBusDevice> device;

public slots:
  void transmitFrame(const QCanBusFrame& frame);
  void processFrame();
};

#endif // CAN_TRANSCEIVER_H
