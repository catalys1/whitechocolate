#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

import Controller

#### Global vars ####

_ctrl_period = 1.0/100

# My current state
_xhat = 0
_yhat = 0
_thetahat = 0

# -------------------

def _handle_me(msg):
    global _xhat, _yhat, _thetahat
    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta


def _handle_desired_position(msg):
    Controller.set_commanded_position(msg.x, msg.y, msg.theta)


def main():
    rospy.init_node('controller', anonymous=False)

    # Subscribe to my current state (from the vision node)
    # and my desired state (from the ai node)
    rospy.Subscriber('me', Pose2D, _handle_me)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)

    # Publish velocity commands from PID controller
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)

    # initialize the controller
    Controller.init()

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

        # Publish Velocity Commands
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        pub.publish(msg)

        # Wait however long it takes to make this tick at proper control period
        rate.sleep()



if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()