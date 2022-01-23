#include "gps-ahrs_receiver.h"
#include <QDebug>

GPSAHRSReceiverNode::GPSAHRSReceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("gps_enabled", gps_enabled);
  private_nh.getParam("gps_port", gps_port);
  private_nh.getParam("gps_baudrate", gps_baudrate);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubOdom = nh.advertise<Odometry>("odom", 10);
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

  node.start();
}

GPSAHRSReceiver::~GPSAHRSReceiver()
{
  if (gpsDevice.isOpen())
    gpsDevice.close();
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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ahrs_receiver");
  QCoreApplication a(argc, argv);
  GPSAHRSReceiver receiver;
  return a.exec();
}
