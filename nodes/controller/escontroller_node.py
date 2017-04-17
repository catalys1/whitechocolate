#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D
from whitechocolate.msg import VisionState
from whitechocolate.msg import MotorCommand
import numpy as np

import Controller

# #### Global vars ####
# pub = None

# _ctrl_period = 1.0/100

# # My current state
# _xhat = 0
# _yhat = 0
# _thetahat = 0

# # -------------------

class ControlHandler(object):

    def __init__(self):
        self.ally1 = Pose2D()
        self.ally2 = Pose2D()
        self.opp1 = Pose2D()
        self.opp2 = Pose2D()
        self.ball = Pose2D()

        self.desired = None

        self.ctrl_period = 1.0/100


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
    pub = rospy.Publisher('vel_cmds', MotorCommand, queue_size=10)
    Controller.init()
    control = ControlHandler()

    # Subscribe to my current state (from the vision node)
    # and my desired state (from the ai node)
    rospy.Subscriber('estimation', VisionState, control.setPositions)
    rospy.Subscriber('des_estimation', Pose2D, control.setDesired)
    # Add this in when the AI is actually publishing it
    # rospy.Subscriber('/desired_position', Pose2D, _handle_desired_position)

    # Publish velocity commands from PID controller

    # initialize the controller

    rate = rospy.Rate(int(1/control.ctrl_period))
    while not rospy.is_shutdown():
        (vx, vy, w) = Controller.update(
            control.ctrl_period,
            control.ally1.x,
            control.ally1.y,
            control.ally1.theta/57.3)

        # Publish Velocity Commands
        msg = MotorCommand()
        msg.command.linear.x = vx
        msg.command.linear.y = vy
        msg.command.angular.z = w
        msg.theta = control.ally1.theta/57.3
        pub.publish(msg)
       
        # Wait however long it takes to make this tick at proper control period
        rate.sleep()
    # rospy.spin()


if __name__ == '__main__':
    # If this file was run from the command line, then do the following:
    main()



# import rospy
# from geometry_msgs.msg import Twist, Pose2D
# from whitechocolate.msg import VisionState
# import numpy as np

# import Controller

# #### Global vars ####

# _ctrl_period = 1.0/100

# # My current state
# _xhat = 0
# _yhat = 0
# _thetahat = 0

# # -------------------

# def _handle_me(msg):
#     global _xhat, _yhat, _thetahat
#     # print msg
#     _xhat = msg.ally1.x
#     _yhat = msg.ally1.y
#     _thetahat = msg.ally1.theta


# def _handle_desired_position(msg):
#     Controller.set_commanded_position(msg.x, msg.y, msg.theta)


# def main():
#     rospy.init_node('controller', anonymous=False)

#     # Subscribe to my current state (from the vision node)
#     # and my desired state (from the ai node)
#     rospy.Subscriber('wc_vision_state', VisionState, _handle_me)
#     rospy.Subscriber('wc_click_desired_position', Pose2D, _handle_desired_position)

#     # Publish velocity commands from PID controller
#     pub = rospy.Publisher('wc_vel_cmds', Twist, queue_size=10)

#     # initialize the controller
#     Controller.init()

#     rate = rospy.Rate(int(1.0/_ctrl_period))
#     while not rospy.is_shutdown():

#         (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

#         # Publish Velocity Commands
#         msg = Twist()
#         msg.linear.x = vx
#         msg.linear.y = vy
#         msg.angular.z = w
#         pub.publish(msg)

#         # Wait however long it takes to make this tick at proper control period
#         rate.sleep()



# if __name__ == '__main__':
#     # If this file was run from the command line, then do the following:
#     main()