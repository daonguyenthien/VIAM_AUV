#ifndef GPSAHRS_RECEIVER_H
#define GPSAHRS_RECEIVER_H

#include <ros/ros.h>
#include <utils/Odometry.h>

#include <QCoreApplication>
#include <QObject>
#include <QSerialPort>
#include <QThread>

#include <boost/algorithm/string.hpp>

#include "utils/geo.h"

#define BUFFER_SIZE 43

using namespace utils;
using namespace std;

class GPSAHRSReceiverNode : public QThread
{
  Q_OBJECT

public:
  GPSAHRSReceiverNode();
  ~GPSAHRSReceiverNode();

  double ned_lat;
  double ned_lon;
  bool gps_enabled;
  bool ahrs_enabled;
  string gps_port;
  string ahrs_port;
  int gps_baudrate;
  int ahrs_baudrate;

  ros::Publisher pubOdom;

  Odometry odomMsg;

  void run();
};

class GPSAHRSReceiver : public QObject
{
  Q_OBJECT

public:
  GPSAHRSReceiver();
  ~GPSAHRSReceiver();

  GPSAHRSReceiverNode node;
  QSerialPort gpsDevice;
  QSerialPort ahrsDevice;

  char ahrsBuffer[BUFFER_SIZE];
  int ahrsBytesReceived = 0;
  bool isAhrsFirst = true;

  inline double CalLat2Deg(double Lat)
  {
    static uint8_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Lat / 100;
    Min = Lat - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }
  inline double CalLong2Deg(double Long)
  {
    static uint16_t Deg = 0;
    static double Min = 0, Result = 0;
    Deg = Long / 100;
    Min = Long - Deg * 100;
    Result = Deg + Min / 60;
    return Result;
  }

public slots:
  void processGPSFrame();
  void processAHRSFrame();
};

#endif // GPSAHRS_RECEIVER_H
