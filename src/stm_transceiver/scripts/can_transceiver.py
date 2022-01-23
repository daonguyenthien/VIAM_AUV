#!/usr/bin/env python3

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
    
def send_one(bus,controll_onoff):
    global thrusterSpeed, rudderAngle, massShifterPosition, pistonPosition
    thruster_pwm = thrusterSpeed / 5
    rudder_pwm = (2600 - 1000) / 180 * (rudderAngle + 90) + 1000
    mass_shifter_pwm = 58 / 400 * (massShifterPosition + 200)
    piston_position_pwm = 0.0

    global motorsLocked

        
   
    ids = [0x123, 0x122, 0x121, 0x121, 0x122, 0x122, 0x122]
    headers = [[79, 76, 82 if (thruster_pwm < 0) else 76], [82, 85, 68], [79, 76, 77], [79, 76, 80], [67,79,78 if motorsLocked else 70 ], [69, 74, 83]]
    values = [abs(thruster_pwm), abs(rudder_pwm), abs(mass_shifter_pwm), abs(piston_position_pwm),0, 0]
    
    for i in range(7):
        if i (i <= 6):
            header = headers[i]
            ba = bytearray(struct.pack('>f', values[i]))
            payload = [header[0], header[1], header[2], get_int(ba[0:1]), get_int(ba[1:2]), get_int(ba[2:3]), get_int(ba[3:4])]
            cs = check_sum(payload)
            payload.append(cs)
            frame = can.Message(arbitration_id=ids[i], data=payload, is_extended_id=False)

            try:
                bus.send(frame)
                print(frame)
            except can.CanError:
                print("Message not sent.")

        if (i >= 4):
            frame = bus._recv_internal(0)
            if frame is not None:
                data = bytearray(struct.pack('>f', frame.data[3:7]))
                if frame.id == "0x123":
                    if frame.data[1] == 84:
                        #ThrusterStatus.motor_duty = data
                        #pub_thruster.publish(ThrusterStatus)
                        print("ThrusterStatus " + str(data))
                if frame.id == "0x122":
                    #ThrusterStatus.motor_duty = data
                    print("RudderStatus " + str(data))
                if frame.id == "0x121":
                    if frame.data[1] == 77:
                        #MassShifterStatus.position = data
                        #pub_mass.publish(MassShifterStatus)
                        print("MassShifterStatus " + str(data))
                    if frame.data[1] == 80:
                        PistonStatus.position = data
                        pub_piston.publish(PistonStatus)
                        print("PistonStatus " + str(data))
                    if frame.data[1] == 65:
                        #MassShifterStatus.position = data
                        print("AltimeterStatus " + str(data))       
                    
                
                print(frame)
    
def onMotorsCmdCallBack(msg):
    global bus, thrusterSpeed, rudderAngle, massShifterPosition, pistonPosition
    thrusterSpeed = msg.thruster_speed
    rudderAngle = msg.rudder_angle
    massShifterPosition = msg.mass_shifter_position
    pistonPosition = msg.piston_position
    send_one(bus, 1.0)
    
def onSetArmingCallBack(req):
    global motorsLocked, thrusterSpeed
    if req.param1 == 1.0:
        motorsLocked = False
        send_one(bus, 1.0)
        rospy.loginfo('Motors unlocked.')
    elif req.param1 == 0.0:
        motorsLocked = True
        thrusterSpeed = 0.0
        send_one(bus, 0.0)
        rospy.loginfo('Motors locked.')
    res = CommandLongResponse(True, 0)
    return res

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('can_transceiver')
    enabled = rospy.get_param('~enabled')
    port = rospy.get_param('~port')
    baudrate = rospy.get_param('~baudrate')
    #pub_thruster = rospy.Publisher('thurster', ThrusterStatus, queue_size=10)
    #pub_rudder = rospy.Publisher('chatter', BoardARM2Status, queue_size=10)
    #pub_mass = rospy.Publisher('mass shifter', BoardARM1Status, queue_size=10)
    pub_piston = rospy.Publisher('piston', PistonStatus, queue_size=10)
    subMotorsCmd = rospy.Subscriber('motors/cmd', MotorsCommand, onMotorsCmdCallBack)
    resSetArming = rospy.Service('command/set_arming', CommandLong, onSetArmingCallBack)
    bus = robotellBus(channel=port, ttyBaudrate=baudrate)
    
    rospy.spin()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
