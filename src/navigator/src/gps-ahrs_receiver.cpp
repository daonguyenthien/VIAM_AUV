#include "gps-ahrs_receiver.h"
#include <QDebug>

GPSAHRSReceiverNode::GPSAHRSReceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("gps_enabled", gps_enabled);
  private_nh.getParam("ahrs_enabled", ahrs_enabled);
  private_nh.getParam("gps_port", gps_port);
  private_nh.getParam("ahrs_port", ahrs_port);
  private_nh.getParam("gps_baudrate", gps_baudrate);
  private_nh.getParam("ahrs_baudrate", ahrs_baudrate);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubOdom = nh.advertise<Odometry>("gps", 10);
}

GPSAHRSReceiverNode::~GPSAHRSReceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GPSAHRSReceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

GPSAHRSReceiver::GPSAHRSReceiver()
{
  if (node.gps_enabled)
  {
    gpsDevice.setPortName(QString::fromStdString(node.gps_port));
    gpsDevice.setBaudRate(node.gps_baudrate);
    gpsDevice.setParity(QSerialPort::NoParity);
    gpsDevice.open(QIODevice::ReadOnly);
    connect(&gpsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processGPSFrame);
  }
  if (node.ahrs_enabled)
  {
    ahrsDevice.setPortName(QString::fromStdString(node.ahrs_port));
    ahrsDevice.setBaudRate(node.ahrs_baudrate);
    ahrsDevice.setParity(QSerialPort::NoParity);
    ahrsDevice.open(QIODevice::ReadOnly);
    connect(&ahrsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processAHRSFrame);
  }

  node.start();
}

GPSAHRSReceiver::~GPSAHRSReceiver()
{
  if (gpsDevice.isOpen())
    gpsDevice.close();
  if (ahrsDevice.isOpen())
    ahrsDevice.close();
}

void GPSAHRSReceiver::processGPSFrame()
{
  auto tmpBuffer = gpsDevice.readAll();
  vector<string> data_lines;
  boost::split(data_lines, tmpBuffer, boost::is_any_of("\r\n"));
  for (auto it : data_lines)
  {
    vector<string> data;
    boost::split(data, it, boost::is_any_of(","));
    if (data[0] == "$GNGGA")
    {
      node.odomMsg.latitude = CalLat2Deg(QString::fromStdString(data[2]).toDouble());
      node.odomMsg.longitude = CalLong2Deg(QString::fromStdString(data[4]).toDouble());
      node.odomMsg.altitude = QString::fromStdString(data[9]).toDouble();
      convert_global_to_local_coords(node.odomMsg.latitude, node.odomMsg.longitude, node.ned_lat, node.ned_lon,
                                     node.odomMsg.position.x, node.odomMsg.position.y);
      node.odomMsg.position.z = -node.odomMsg.altitude;
    }
  }

  if (!ahrsDevice.isOpen())
  {
    node.odomMsg.header.stamp = ros::Time::now();
    node.pubOdom.publish(node.odomMsg);
  }
}

void GPSAHRSReceiver::processAHRSFrame()
{
  auto tmpBuffer = ahrsDevice.readAll();

  if (isAhrsFirst)
  {
    ROS_INFO("Waiting for correct header.");
    if (tmpBuffer[0] == '\xFA')
      isAhrsFirst = false;
    else
      return;
  }

  auto tmpBufferSize = tmpBuffer.length();
  for (int i = ahrsBytesReceived; i < ahrsBytesReceived + tmpBufferSize; i++)
    ahrsBuffer[i] = tmpBuffer.data()[i - ahrsBytesReceived];
  ahrsBytesReceived += tmpBufferSize;
  if (ahrsBytesReceived < BUFFER_SIZE)
    return;
  ahrsBytesReceived = 0;

//  auto get_angle = [](int i, char* buffer) {
//    char raw[8] = {buffer[7 + 6 * i],
//                   buffer[6 + 6 * i],
//                   buffer[5 + 6 * i],
//                   buffer[4 + 6 * i],
//                   buffer[9 + 6 * i],
//                   buffer[8 + 6 * i],
//                   0,
//                   0};
//    uint64_t magnitude = 0;
//    memcpy(&magnitude, raw, sizeof(raw));
//    int sign = buffer[8 + 6 * i];
//    if (sign == -1)
//      magnitude = 281474976710655ULL - magnitude;
//    double number = sign * magnitude / 4294967296.0;
//    return number;
//  };

//  node.odomMsg.header.stamp = ros::Time::now();
//  node.odomMsg.orientation.x = get_angle(0, ahrsBuffer) / 180 * M_PI;
//  node.odomMsg.orientation.y = get_angle(1, ahrsBuffer) / 180 * M_PI;
//  node.odomMsg.orientation.z = get_angle(2, ahrsBuffer) / 180 * M_PI;
//  node.pubOdom.publish(node.odomMsg);

  qDebug() << QByteArray(ahrsBuffer).toHex();

  uint64_t num = 0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  int am = 0;
  for (int i = 0; i < 3; i++)
  {
    char temp[8] = {ahrsBuffer[7 + 6 * i],
                    ahrsBuffer[6 + 6 * i],
                    ahrsBuffer[5 + 6 * i],
                    ahrsBuffer[4 + 6 * i],
                    ahrsBuffer[9 + 6 * i],
                    ahrsBuffer[8 + 6 * i],
                    0,
                    0};
    memcpy(&num, temp, sizeof(temp));

    if ((int)ahrsBuffer[8 + 6 * i] == -1)
    {
      am = 1;
    }
    else
    {
      am = -1;
    }

    if (int(ahrsBuffer[8 + 6 * i]) == -1)
      num = 281474976710655ULL - num;

    double num_ = num / 4294967296.0;

    switch (i)
    {
    case 0:
      roll = num_ * am / 180 * M_PI;
      break;
    case 1:
      pitch = num_ * am / 180 * M_PI;
      break;
    case 2:
      yaw = num_ * am / 180 * M_PI;
      break;
    }
  }

  node.odomMsg.header.stamp = ros::Time::now();
  node.odomMsg.orientation.x = roll;
  node.odomMsg.orientation.y = pitch;
  node.odomMsg.orientation.z = yaw;
  node.pubOdom.publish(node.odomMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ahrs_receiver");
  QCoreApplication a(argc, argv);
  GPSAHRSReceiver receiver;
  return a.exec();
}
