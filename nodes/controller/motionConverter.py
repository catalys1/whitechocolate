#!/usr/bin/env python
import rospy
import struct
import time
import serial
import signal

# ser = serial.Serial('COM11', 115200, timeout=None) #windows
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)
# ser = serial.Serial('/dev/stdout', 115200, timeout=None) #TEST
from geometry_msgs.msg import Twist, Pose2D, Vector3
# from std_srvs.srv import Trigger, TriggerResponse

import numpy as np
from math import cos, sin, pi

# distances in meters
a = 0.087988
b = 0.043994
c = 0.0762
# radius of the wheel in meters (1 inch)
RHO = 0.0254
# wheel direction unit vectors
x = 0.5
y = 0.866
SX1 = -x
SY1 =  y
SX2 = -x
SY2 = -y
SX3 =  1
SY3 =  0
# wheel position vectors
RX1 =  c
RY1 =  b
RX2 = -c
RY2 =  b
RX3 =  0
RY3 = -a


theta_ = 0
vel_cmd_ = Vector3()
wheel_speeds_ = np.zeros(shape=(3))

M_ = (1/RHO)*np.array([[SX1, SY1, (SY1*RX1 - SX1*RY1)],
                       [SX2, SY2, (SY2*RX2 - SX2*RY2)],
                       [SX3, SY3, (SY3*RX3 - SX3*RY3)]])

# ============== MOTOR FUNCTIONS ====================
def writeFloat(f):
    ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
    return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
    ser.write('p')
    writeFloat(p1)
    writeFloat(p2)
    writeFloat(p3)
def setSpeed(s1, s2, s3):
    ser.write('s')
    writeFloat(s1)
    writeFloat(s2)
    writeFloat(s3)
def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
    ser.write('k')
    ser.write(str(motor))
    writeFloat(p)
    writeFloat(i)
    writeFloat(qpps)
def setT(period_ms, tau_ms):
    ser.write('t')
    writeFloat(period_ms)
    writeFloat(tau_ms)
def getSpeed():
    ser.write('v')
    return readFloat(), readFloat(), readFloat()
def getEncoderCount():
    ser.write('e')
    return readFloat(), readFloat(), readFloat()
def disengage():
    ser.write('d')


# ============== ROS NODE FUNCTIONS ====================


def _pose2array(msg):
    return np.array([msg.linear.x, msg.linear.y, msg.angular.z])

def _handle_velocity_command(msg):
    global vel_cmd_
    vel_cmd_.x = msg.linear.x
    vel_cmd_.y = msg.linear.y
    vel_cmd_.z = msg.angular.z
    vel_vec = _pose2array(msg)
    # print(msg)
    computeMotorSpeeds(vel_vec)

def computeMotorSpeeds(vel_vec):
    global wheel_speeds_
    theta = vel_vec[2]
    ct = cos(theta)
    st = sin(theta)

    R = np.array([[ ct, st, 0],
                  [-st, ct, 0],
                  [  0,  0, 1]])

    # print(velocities)
    wheel_speeds_ = np.dot(M_, R).dot(vel_vec)

    sendVelocityCommands()

def sendVelocityCommands():
    speedM1 = wheel_speeds_[0] # rot/s
    speedM2 = wheel_speeds_[1] # rot/s
    speedM3 = wheel_speeds_[2] # rot/s

    # print('Wheel speeds: {}',format(wheel_speeds_))

    pulsePerRotation = 4955 #Old motors

    setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)


def stopMotors(signal,frame):
    '''Tell the motors to shut down
    '''
    setSpeed(0,0,0)
    disengage()
    exit(0)


def setup():
    # Set tick period (triggers PID control) and velocity filter corner frequency
    setT(20, 50)
    # Set the PIDQ values for all motors
    setPID(1, 1.6, 0.4, 63000)
    setPID(2, 1.75, 0.5, 50000)
    setPID(3, 1.75, 0.5, 50000)


motor_speed_pub_ = None

def main():
    signal.signal(signal.SIGINT, stopMotors)
    rospy.init_node('MotionControl', anonymous=False)
    setup()

    # Subscribe to velocity commands
    rospy.Subscriber('/whitechocolate/vel_cmds', Twist, _handle_velocity_command)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()