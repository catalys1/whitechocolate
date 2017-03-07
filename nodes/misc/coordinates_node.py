!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState
from whitechocolate.msg import RobotState


class VisionEstimator(object):

	def __init__(self):
		self.game_state = None
		self.state_publisher = None
		self.team_side = None


	def updateGameState(self, msg):
		self.game_state = msg


	def estimateVision(self, msg):
		'''Create a RobotState message from a position message given by the 
		vision node. The function will flip the coordinate system when
		necessary, in order to keep things consistent for the AI and controller.
		'''
		# Flip the coordinate system so that our goal is always in the
		# negative x direction. This means we don't have to worry about
		# it in our algorithms
		if (self.team_side != 'home') ^ bool(self.game_state.second_half):
			msg.x = -msg.x
			msg.y = -msg.y
			msg.theta = (msg.theta + 180) % 360

		# HERE WE WILL WANT TO DO ESTIMATION

		# Publish our position
		state_msg = RobotState()
	    state_msg.xhat = state_msg.vision_x = state_msg.xhat_future = msg.x
	    state_msg.yhat = state_msg.vision_y = state_msg.yhat_future = msg.y
	    state_msg.thetahat = state_msg.vision_theta = state_msg.thetahat_future = msg.theta
	    state_msg.correction = True
	    _state_pub.publish(state_msg)


def _handle_gamestate(gamestate_msg, vision):
	vision.updateGameState(gamestate_msg)


def _handle_vision(position_msg, vision):
	vision.estimateVision(position_msg)


def main():
	# vision is our VisionExtimator object
	rospy.init_node('robot_estimator')
	vision = VisionEstimator()

	param_name = rospy.search_param('team_side')
    vision.team_side = rospy.get_param(param_name, 'home')

    # We will be publishing the state of our robot. Remapping in the launch files will
    # create different topics for each of them
    vision.state_publisher = rospy.Publisher('robot_state', RobotState, queue_size=10)
    # We will be subscribing to vision and game state
    rospy.Subscriber('vision_position', Pose2d, _handle_vision, callback_args=vision)
    rospy.Subscriber('/game_state', GameState, _handle_gamestate, callback_args=vision)

    # Don't exit until the node is stopped externally
    rospy.spin()


if __name__ == '__main__':
	main()
