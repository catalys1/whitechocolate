#!/usr/bin/env python
import rospy
import struct
import time
import serial
import signal
import numpy as np

from geometry_msgs.msg import Twist, Pose2D, Vector3
from whitechocolate.msg import MotorCommand, WheelSpeeds

# ser = serial.Serial('COM11', 115200, timeout=None) #windows
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None) #linux
# ser = serial.Serial('/dev/stdout', 115200, timeout=None) #TEST
# from std_srvs.srv import Trigger, TriggerResponse
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

vel_cmd_ = Vector3()
motor_speed_pub_ = None
theta_ = 0
wheel_speeds_ = np.zeros(shape=(3))

M_ = []

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
	# create a column vector with x, y, omega
	return np.array([
		[msg.linear.x], 
		[msg.linear.y], 
		[msg.angular.z]
	])

def _handle_velocity_command(msg):
	global vel_vec_, theta_
	vel_vec_ = _pose2array(msg.command)
	theta_ = msg.theta
	computeMotorSpeeds()

def computeMotorSpeeds():
	global wheel_speeds_
	ct = np.cos(theta_)
	st = np.sin(theta_)

	R = np.matrix([
		[ ct, st, 0],
		[-st, ct, 0],
		[  0,  0, 1]
	])

	# print(velocities)
	wheel_speeds_ = M_ * R * vel_vec

	sendVelocityCommands()

def sendVelocityCommands():
	global motor_speed_pub_
	pulsePerRotation = 4955 #Old motors

	speedM1 = wheel_speeds_.item(0) * pulsePerRotation
	speedM2 = wheel_speeds_.item(1) * pulsePerRotation
	speedM3 = wheel_speeds_.item(2) * pulsePerRotation

	# print('Wheel speeds: {}'.format(wheel_speeds_))
	setSpeed(speedM1, speedM2, speedM3)
	msg = WheelSpeeds()
	msg.wheel1 = speedM1
	msg.wheel2 = speedM2
	msg.wheel3 = speedM3
	motor_speed_pub_.publish(msg)


def stopMotors(signal,frame):
	'''Tell the motors to shut down
	'''
	setSpeed(0,0,0)
	disengage()
	exit(0)


def setup():
	global M_
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

	M_ = (1./RHO) * np.matrix([
		[SX1, SY1, (SY1*RX1 - SX1*RY1)],
		[SX2, SY2, (SY2*RX2 - SX2*RY2)],
		[SX3, SY3, (SY3*RX3 - SX3*RY3)]
	])

	# Set tick period (triggers PID control) and velocity filter corner frequency
	setT(20, 50)
	# Set the PIDQ values for all motors
	setPID(1, 1.60, 0.4, 63000)
	setPID(2, 1.75, 0.5, 50000)
	setPID(3, 1.75, 0.5, 50000)


def main():
	signal.signal(signal.SIGINT, stopMotors)
	rospy.init_node('MotionControl', anonymous=False)
	setup()

	global motor_speed_pub_
	motor_speed_pub_ = rospy.Publisher('/whitechocolate/wheels', WheelSpeeds, queue_size=10)

	# Subscribe to velocity commands
	rospy.Subscriber('/whitechocolate/vel_cmds', MotorCommand, _handle_velocity_command)


	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()