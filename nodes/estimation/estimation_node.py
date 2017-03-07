#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState


class Estimator(object):

	def __init__(self, publisher):
		self.publisher = publisher


	def estimate(self, vision_msg):
		self.publisher.publish(vision_msg)



def main():
	rospy.init_node('estimation_node')
	pub = rospy.Publisher('/wc_estimation', VisionState, queue_size=10)
	est = Estimator(pub)

	# We will be subscribing to vision and game state
	rospy.Subscriber('/wc_vision_state', VisionState, est.estimate)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()