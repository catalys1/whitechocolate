#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

import Controller

#### Global vars ####
pub = None

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
    (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

    # Publish Velocity Commands
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = w
    pub.publish(msg)



def main():
    rospy.init_node('controller', anonymous=False)
    global pub
    pub = rospy.Publisher('wc_vel_cmds', Twist, queue_size=10)
    Controller.init()

    # Subscribe to my current state (from the vision node)
    # and my desired state (from the ai node)
    rospy.Subscriber('wc_robot_position', Pose2D, _handle_me)
    rospy.Subscriber('wc_desired_position', Pose2D, _handle_desired_position)

    # Publish velocity commands from PID controller

    # initialize the controller

    # rate = rospy.Rate(int(1/_ctrl_period))
    # while not rospy.is_shutdown():

       
    #     # Wait however long it takes to make this tick at proper control period
    #     rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()