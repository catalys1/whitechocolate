#!/usr/bin/env python
import numpy as np
import rospy
import AI
from geometry_msgs.msg import Pose2D


position = None
pub = None
pos_commanded = None
flag = True

class AIProcessor(object):

	def __init__(self,pub):
		self.pos_r = [0,0,0]
		self.pos_b = [0,0]
		self.pub = pub
		self.AI = AI.AI()

	def save_pos(self, msg):
		self.pos_r = [
			msg.ally1.x,
			msg.ally1.y,
			msg.ally1.theta
		]
		self.pos_b = [
			msg.ball.x,
			msg.ball.y,
			msg.ball.theta
		]


	def strategize(self):


		self.AI.spin_360(self.pos_r[2])
		c_msg = Pose2D()
		c_msg.x, c_msg.y, c_msg.theta = [self.pos_r[0],self.pos_r[1],30]
		self.pub.publish(c_msg)

	# def spin():
	# 	global position
	# 	global pos_commanded
	# 	threshold = np.pi / 18.0
	# 	if abs(pos_commanded[2]-position[2]) < threshold:
	# 		pos_commanded[2] = (pos_commanded[2] + 2*threshold)
	# 	if pos_commanded[2] > 2*np.pi:
	# 		pos_commanded[2] -= 2*np.pi
	# 	return pos_commanded


def main():
	rospy.init_node('whitechocolate_skills')
	pub = rospy.Publisher('wc_desired_position', Pose2D, queue_size=10)
	ai = AIProcessor(pub)

	# We will be subscribing to vision and game state
	rospy.Subscriber('/wc_estimation', Pose2D, ai.save_pos)

	rate = rospy.Rate(100) # 100 Hz
	increment = 0
	testing = 1
	while not rospy.is_shutdown():
		if(increment == 100)
			self.ai.strategize()
			increment = 0
		if(testing = 1)
			self.ai.spin_90(self.ai.pos_r[2])
			testing = 0
		self.ai.update_sm(self.ai.pos_r[2])
		increment += 1
		rate.sleep()


if __name__ == '__main__':
	main()