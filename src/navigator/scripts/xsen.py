#!/usr/bin/env python3
import serial
import time
import rospy
from utils.msg import Odometry
def convert(string):
    string = string[8:12] + string[0:8]
    an_integer = int(string, 16)
    
    if string[0:2] == "ff":
        an_integer = 281474976710655 - an_integer
        an_integer = -an_integer/4294967296
        
        return an_integer
    an_integer = an_integer/4294967296
    return an_integer

serialPort = serial.Serial(
    port="/dev/ttyTHS1", baudrate=38400, bytesize=8, timeout=None,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE
)
serialString = ""  # Used to hold data coming over UART
time.sleep(1)
roll = 0.0
pitch = 0.0 
yaw = 0.0
sum_ = []
data_send = Odometry()

def callback(data):
    data_send.latitude = data.latitude
    data_send.longitude = data.longitude
    data_send.altitude = data.altitude

def Readframe():
    global roll , pitch , yaw, sum_
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rospy.Subscriber("gps", Odometry, callback)
    rospy.init_node('imu_transmit', anonymous=True)
    rate = rospy.Rate(10) 
    #print(serialPort.read(42).hex())
    #Wait until there is data waiting in the serial buffer
    if True:        
        if serialPort.read(1).hex() == "fa":
            #print(serialPort.read(42).hex())
            serialString = serialPort.read(42).hex()
            data_send.header.stamp = rospy.Time.now()
            data_send.orientation.x = ( convert(serialString[6:18])/180*3.14)
            data_send.orientation.y = ( convert(serialString[18:30])/180*3.14)
            data_send.orientation.z = ( convert(serialString[30:42])/180*3.14)
            #data_send.data = sum_
            #print("roll: %s | pitch: %s | yaw: %s",convert(serialString[6:18])*180/3.14,pitch,yaw)
        pub.publish(data_send)
                    
         

if __name__=="__main__":
    
    while 1:
        Readframe()
        
        
        
        
        
             
