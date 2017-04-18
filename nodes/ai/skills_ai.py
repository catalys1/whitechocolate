#!/usr/bin/env python
import numpy as np
import rospy
import AI
from geometry_msgs.msg import Pose2D
from soccerref.msg import GameState
from whitechocolate.msg import VisionState


position = None
pub = None
pos_commanded = None
flag = True

class AIProcessor(object):

	def __init__(self,pub):
		self.ally1 = np.zeros(3, np.float64)
		self.ally2 = np.zeros(3, np.float64)
		self.pos1  = np.zeros(3, np.float64)
		self.pos2  = np.zeros(3, np.float64)
		self.ball  = np.zeros(3, np.float64)

		self.pub = pub
		self.AI = AI.AI('home',1)

		self.game_state = GameState()

		self.click_desired = np.zeros(3, np.float64)


	def play_soccer(self):
		'''Game loop
		'''
		command = np.array([0.0, 0.0, 0.0])
		# reset field has priority
		if self.game_state.reset_field:
			# either set up for a penelty of go to start position
			if self.game_state.home_penalty:
				pass
			elif self.game_state.away_penalty:
				pass
			else:
				command = self.AI.reset_offense()
		elif self.game_state.play:
			# command = self.click_desired
			command = self.AI.playOffense(self.ally1, self.ball)
		else:
			command = self.AI.stop(self.ally1)

		self.pub.publish(self.array2pose(command))


	def pose2array(self, pose_msg):
		return np.array([pose_msg.x,pose_msg.y,pose_msg.theta])


	def array2pose(self, arr):
		return Pose2D(*arr)


	def save_pos(self, msg):
		self.ally1[:] = self.pose2array(msg.ally1)
		self.ball[:] = self.pose2array(msg.ball)


	def update_game_state(self, msg):
		'''Save a snapshot of the game state. Expects a GameState message.
		'''
		self.game_state = msg


	def save_desired(self, msg):
		'''
		'''
		self.click_desired = self.pose2array(msg)





def main():
	rospy.init_node('whitechocolate_skills')
	pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)
	ai = AIProcessor(pub)

	# We will be subscribing to estimation and game state
	rospy.Subscriber('estimation', VisionState, ai.save_pos)
	rospy.Subscriber('game_state', GameState, ai.update_game_state)
	rospy.Subscriber('des_estimation', Pose2D, ai.save_desired)

	rate = rospy.Rate(100) # 100 Hz
	increment = 0
	testing = 1
	while not rospy.is_shutdown():
		ai.play_soccer()
		rate.sleep()


if __name__ == '__main__':
	main()