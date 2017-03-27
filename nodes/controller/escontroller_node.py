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

class ControlHandler(object):

    def __init__(self):
        self.ally1 = None
        self.ally2 = None
        self.opp1 = None
        self.opp2 = None
        self.ball = None

        self.desired = None


    def setPositions(self, msg):
        # We probably don't need all of these in this node, only what 
        # we are going to use to control the robot.
        self.ally1 = msg.ally1
        self.ally2 = msg.ally2
        self.opp1 = msg.opp1
        self.opp2 = msg.opp2
        self.ball = msg.ball

    def setDesired(self, msg):
        self.desired = msg
        Controller.set_commanded_position(msg.x, msg.y, msg.theta)


def main():
    rospy.init_node('controller', anonymous=False)
    pub = rospy.Publisher('wc_vel_cmds', Twist, queue_size=10)
    Controller.init()
    control = ControlHandler()

    # Subscribe to my current state (from the vision node)
    # and my desired state (from the ai node)
    rospy.Subscriber('/wc_estimation', VisionState, control.setPosition)
    rospy.Subscriber('/wc_des_estimation', Pose2D, control.setDesired)
    # rospy.Subscriber('/wc_desired_position', Pose2D, _handle_desired_position)

    # Publish velocity commands from PID controller

    # initialize the controller

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():
        (vx, vy, w) = Controller.update(
            _ctrl_period,
            ally1.x,
            ally1.y,
            ally1.theta)

        # Publish Velocity Commands
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        pub.publish(msg)
       
        # Wait however long it takes to make this tick at proper control period
        rate.sleep()
    # rospy.spin()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()