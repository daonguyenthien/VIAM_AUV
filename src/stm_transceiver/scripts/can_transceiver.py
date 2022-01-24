#!/usr/bin/env python3

import rospy
from utils.msg import MotorsCommand, BoardARM2Status, BoardARM1Status, ThrusterStatus, MassShifterStatus, PistonStatus
from utils.srv import CommandLong, CommandLongResponse

import struct
import time
import can
import sys
import termios
from can.interfaces.robotell import robotellBus

motorsLocked = True
thrusterSpeed = 0.0
rudderAngle = 0.0
massShifterPosition = 0.0
pistonPosition = 0.0
count = 0
_ThrusterStatus = ThrusterStatus()
_BoardARM2Status = BoardARM2Status()
_MassShifterStatus = MassShifterStatus()
_PistonStatus = PistonStatus()
_BoardARM1Status = BoardARM1Status()


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
    global thrusterSpeed, rudderAngle, massShifterPosition, pistonPosition, count
    thruster_pwm = thrusterSpeed / 5
    rudder_pwm = (2600 - 1000) / 180 * (rudderAngle + 90) + 1000
    mass_shifter_pwm = 58 / 400 * (massShifterPosition + 200)
    piston_position_pwm = pistonPosition

    global motorsLocked

    ids = [0x123, 0x122, 0x121, 0x121, 0x122, 0x122, 0x121]
    headers = [[79, 76, 82 if (thruster_pwm < 0) else 76], [82, 85, 68], [79, 76, 77], [
        79, 76, 80], [67, 79, 78 if motorsLocked else 70], [69, 74, 83], [69, 74, 83]]
    values = [abs(thruster_pwm), abs(rudder_pwm), abs(
        mass_shifter_pwm), piston_position_pwm, 0, 0, 0]
    for i in range(13):
        if (i < 7):
            header = headers[i]
            ba = bytearray(struct.pack('>f', values[i]))
            payload = [header[0], header[1], header[2], get_int(
                ba[0:1]), get_int(ba[1:2]), get_int(ba[2:3]), get_int(ba[3:4])]
            cs = check_sum(payload)
            payload.append(cs)
            frame = can.Message(
                arbitration_id=ids[i], data=payload, is_extended_id=False)

            try:
                bus.send(frame)
            except can.CanError:
                print("Message not sent.")

        if i >= 6:
            frame, _ = bus._recv_internal(0)
            print(frame)
            if frame is not None:

                data = float(struct.unpack('>f', frame.data[3:7])[0])

                if frame.arbitration_id == 291:
                    if frame.data[1] == 84:
                        _ThrusterStatus.motor_duty = data
                        _ThrusterStatus.header.stamp = rospy.Time.now()
                        pub_thruster.publish(_ThrusterStatus)

                if frame.arbitration_id == 290:

                    _BoardARM2Status.rudder_position = data
                    print("rudder: "+str(data))

                    _BoardARM2Status.header.stamp = rospy.Time.now()
                    pub_rudder.publish(_BoardARM2Status)

                if frame.arbitration_id == 289:
                    if frame.data[1] == 77:
                        _MassShifterStatus.position = data
                        _MassShifterStatus.header.stamp = rospy.Time.now()
                        pub_mass.publish(_MassShifterStatus)

                    if frame.data[1] == 80 and frame.data[0] == 80:
                        _PistonStatus.position = data
                        _PistonStatus.header.stamp = rospy.Time.now()
                        pub_piston.publish(_PistonStatus)
                    if frame.data[1] == 65:
                        _BoardARM1Status.altimeter_in_metres = data
                        _BoardARM1Status.header.stamp = rospy.Time.now()
                        pub_altimeter.publish(_BoardARM1Status)


def onMotorsCmdCallBack(msg):
    global bus, thrusterSpeed, rudderAngle, massShifterPosition, pistonPosition
    thrusterSpeed = msg.thruster_speed
    rudderAngle = msg.rudder_angle
    massShifterPosition = msg.mass_shifter_position
    pistonPosition = msg.piston_position
    send_one(bus)


def onSetArmingCallBack(req):
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


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('can_transceiver')
    enabled = rospy.get_param('~enabled')
    port = rospy.get_param('~port')
    baudrate = rospy.get_param('~baudrate')
    bus = robotellBus(channel=port, ttyBaudrate=115200)
    pub_thruster = rospy.Publisher(
        'status/thruster', ThrusterStatus, queue_size=10)
    pub_rudder = rospy.Publisher(
        'status/board_arm2', BoardARM2Status, queue_size=10)
    pub_mass = rospy.Publisher(
        'status/mass_shifter', MassShifterStatus, queue_size=10)
    pub_piston = rospy.Publisher('status/piston', PistonStatus, queue_size=10)
    pub_altimeter = rospy.Publisher(
        'status/board_arm1', BoardARM1Status, queue_size=10)
    subMotorsCmd = rospy.Subscriber(
        'motors/cmd', MotorsCommand, onMotorsCmdCallBack)
    resSetArming = rospy.Service(
        'command/set_arming', CommandLong, onSetArmingCallBack)
    rospy.spin()
