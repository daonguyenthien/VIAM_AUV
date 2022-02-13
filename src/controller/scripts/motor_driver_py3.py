#!/usr/bin/env python3

import rospy
from utils.msg import MotorsCommand
from utils.srv import CommandLong

import math
import numpy
import sys
import select
import termios
import tty

cmdBindings = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (1, 0),
    'l': (-1, 0),
    '[': (1, 0),
    ']': (-1, 0),
}

thruster_speed = 0.0
rudder_angle = 0.0
mass_shifter_position = 0.0
stopped = True
piston = 0.0


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('motors_driver')
    pubMotorsCmd = rospy.Publisher('motors/cmd', MotorsCommand, queue_size=5)
    reqSetArming = rospy.ServiceProxy('command/set_arming', CommandLong)

    print("currently:\tspeed = %s rpm \tturn = %s deg \tdive = %s mm" %
          (thruster_speed, rudder_angle, mass_shifter_position))
    reqSetArming(False, 400, 0, stopped, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'n':
            piston = piston + 10
            if piston >= 0:
                piston = 0
            print(("piston = " + str(piston)))
        if key == 'm':
            piston = piston - 10
            if piston <= -50:
                piston = -50
            print(('piston = '+str(piston)))
        if key == 'i' or key == ',':
            thruster_speed = thruster_speed + 20 * cmdBindings[key][0]
            thruster_speed = min(thruster_speed, 500.0)
            thruster_speed = max(thruster_speed, -500.0)
            print("update:\tspeed = %s rpm \tturn = %s deg \tdive = %s mm" %
                  (thruster_speed, rudder_angle, mass_shifter_position))

        elif key == 'j' or key == 'l':
            rudder_angle = rudder_angle + 5.0 * cmdBindings[key][0]
            rudder_angle = min(rudder_angle, 60.0)
            rudder_angle = max(rudder_angle, -60.0)
            print("update:\tspeed = %s rpm \tturn = %s deg \tdive = %s mm" %
                  (thruster_speed, rudder_angle, mass_shifter_position))

        elif key == '[' or key == ']':
            mass_shifter_position = mass_shifter_position + \
                10 * cmdBindings[key][0]
            mass_shifter_position = min(mass_shifter_position, 200.0)
            mass_shifter_position = max(mass_shifter_position, -200.0)
            print("update:\tspeed = %s rpm \tturn = %s deg \tdive = %s mm" %
                  (thruster_speed, rudder_angle, mass_shifter_position))

        elif key == 'k':
            stopped = not stopped
            reqSetArming(False, 400, 0, stopped, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        elif True:
            msg = MotorsCommand()
            msg.header.stamp = rospy.Time.now()
            msg.thruster_speed = thruster_speed
            msg.rudder_angle = rudder_angle
            msg.mass_shifter_position = mass_shifter_position
            msg.piston_position = piston
            pubMotorsCmd.publish(msg)
            print("send:\tspeed = %s rpm \tturn = %s deg \tdive = %s mm" %
                  (thruster_speed, rudder_angle, mass_shifter_position))

        elif key == 'q':
            break
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
