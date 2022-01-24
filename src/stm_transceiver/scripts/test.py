#/usr/bin/env python3

import rospy
from utils.msg import MotorsCommand,BoardARM2Status,BoardARM1Status,ThrusterStatus,MassShifterStatus,PistonStatus
from utils.srv import CommandLong, CommandLongResponse

import struct, time, can
import sys, termios
from can.interfaces.robotell import robotellBus

#enabled = True
#port = '/dev/ttyUSB0'
#audrate = 115200

motorsLocked = True
thrusterSpeed = 0.0
rudderAngle = 0.0
massShifterPosition = 0.0
pistonPosition = 0.0

def check_sum(data):
    cs = 0
    for i in range(len(data)):
        cs = cs + data[i]
    cs = ~cs
    cs = (cs + 1) & 0xff
    return cs
    
def get_int(ba):
    return int.from_bytes(ba, 'big')
    
def send_one(bus):
    
    
    for i in range(12):
        if (i < 5):
            pass

        if True:
            frame,_ = bus._recv_internal(0)
            print(frame)
            if frame is not None:
                data = struct.unpack('>f', frame.data[3:7])
                print(frame)
                
                
          
                    
                    
                
                
                if frame.arbitration_id == 289:
                    if frame.data[1] == 77:
                        #MassShifterStatus.position = data
                        #pub_mass.publish(MassShifterStatus)
                        print("MassShifterStatus " + str(data))
                     
                    
                
                
    
def onMotorsCmdCallBack(msg):
    global bus, thrusterSpeed, rudderAngle, massShifterPosition, pistonPosition
    thrusterSpeed = msg.thruster_speed
    rudderAngle = msg.rudder_angle
    massShifterPosition = msg.mass_shifter_position
    pistonPosition = msg.piston_position
    send_one(bus)
    
def onSetArmingCallBack(req):
    global motorsLocked, thrusterSpeed
    if req.param1 == 1.0:
        motorsLocked = False
        send_one(bus)
        rospy.loginfo('Motors unlocked.')
    elif req.param1 == 0.0:
        motorsLocked = True
        thrusterSpeed = 0.0
        send_one(bus)
        rospy.loginfo('Motors locked.')
    res = CommandLongResponse(True, 0)
    return res

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    
    enabled = True
    port = '/dev/ttyUSB0'
    baudrate = 115200
    #pub_thruster = rospy.Publisher('thurster', ThrusterStatus, queue_size=10)
    #pub_rudder = rospy.Publisher('chatter', BoardARM2Status, queue_size=10)
    #pub_mass = rospy.Publisher('mass shifter', BoardARM1Status, queue_size=10)
    #pub_piston = rospy.Publisher('piston', PistonStatus, queue_size=10)
    #subMotorsCmd = rospy.Subscriber('motors/cmd', MotorsCommand, onMotorsCmdCallBack)
    #resSetArming = rospy.Service('command/set_arming', CommandLong, onSetArmingCallBack)
    bus = robotellBus(channel=port, ttyBaudrate=baudrate)
    while True:
        send_one(bus)
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
