#!/usr/bin/env python
import rospy
import struct
import time
import serial

# ser = serial.Serial('COM11', 115200, timeout=None) #windows
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)
# ser = serial.Serial('/dev/stdout', 115200, timeout=None) #TEST
from geometry_msgs.msg import Twist, Pose2D, Vector3
# from std_srvs.srv import Trigger, TriggerResponse

from slash_dash_bang_hash.msg import State, MotorSpeeds

import numpy as np
from math import cos, sin, pi

# Kinematic constants
RHO = 0.02875 # Wheel radius [m]
SX1 = -0.866 # Wheel spin vectors - body frame (unit vectors)
SY1 = 0.5
SX2 = -0.866
SY2 = -0.5
SX3 = 1.0
SY3 = 0
RY1 = 0.03889 # Wheel position vectors - body frame (in meters)
RX1 = 0.0777875
RY2 = 0.03889
RX2 = -0.0777875
RX3 = 0.0
RY3 = -0.03889

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

def _handle_velocity_command(msg):
    global vel_cmd_
    vel_cmd_.x = msg.linear.x
    vel_cmd_.y = msg.linear.y
    vel_cmd_.z = msg.angular.z
    print(msg)
    computeMotorSpeeds()

def computeMotorSpeeds():
    global wheel_speeds_
    ct = cos(theta_)
    st = sin(theta_)

    R = np.array([[ ct, st, 0],
                  [-st, ct, 0],
                  [  0,  0, 1]])

    velocities = np.array([vel_cmd_.x, vel_cmd_.y, vel_cmd_.z])
    print(velocities)
    wheel_speeds_ = np.dot(M_, R).dot(velocities)

    sendVelocityCommands()

def sendVelocityCommands():
    speedM1 = wheel_speeds_[0] # rot/s
    speedM2 = wheel_speeds_[1] # rot/s
    speedM3 = wheel_speeds_[2] # rot/s

    print(wheel_speeds_)

    # totalTime = 3   #seconds
    # sampleRate = 50 #samples per second
    pulsePerRotation = 4955 #Old motors
    # pulsePerRotation = 116.2 #New motors

    # Set the PIDQ values for all motors
    setPID(0, 1, 1, 40000)

    # Set tick period (triggers PID control) and velocity filter corner frequency
    setT(20, 50)

    setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)




def main():
    rospy.init_node('MotionControl', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('wc_vel_cmds', Twist, _handle_velocity_command)
    motor_speed_pub_ = rospy.Publisher('wc_motor_speeds', MotorSpeeds, queue_size=10)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()