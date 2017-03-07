#!/usr/bin/env python
import numpy as np
import rospy
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
	while not rospy.is_shutdown():
		ai.strategize()
		rate.sleep()


if __name__ == '__main__':
	main()