#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState

import numpy as np


class EstimateManager(object):

	measure_period = 1.0/30	#camera frame rate
	ctrl_period = 1.0/100	#rate of sending out estimates
	alpha = 0.1
	lam  = 0.05
	beta = (2*lam-measure_period)/(2*lam+measure_period)

	def __init__(self, publisher):
		self.publisher = publisher
		self.des_pub = None
		self.inches_per_pix = 0.2
		self.convert = np.array([self.inches_per_pix, self.inches_per_pix, 1])

		self.ally1 = Estimator()
		self.ally2 = Estimator()
		self.opp1  = Estimator()
		self.opp2  = Estimator()
		self.ball  = Estimator()


	def estimate(self, vision_msg):

		# Our coordinate system is relative to the top left
		# corner of the field currently on the image produced by vision
		# Real world - this is the corner closest to our station/SE corner
		world_ally1 = self.convertToWorld(vision_msg.ally1)
		world_ally2 = self.convertToWorld(vision_msg.ally2)
		world_opp1 = self.convertToWorld(vision_msg.opp1)
		world_opp2 = self.convertToWorld(vision_msg.opp2)
		world_ball = self.convertToWorld(vision_msg.ball)
		# world_ally1 = Pose2D(
		# 	x=pix_ally1.x*inch_per_pix,
		# 	y=pix_ally1.y*inch_per_pix, 
		# 	theta=pix_ally1.theta)
		# world_ally2 = Pose2D(
		# 	x=pix_ally2.x*inch_per_pix, 
		# 	y=pix_ally2.y*inch_per_pix, 
		# 	theta=pix_ally2.theta)
		# world_opp1 = Pose2D(
		# 	x=pix_opp1.x*inch_per_pix, 
		# 	y=pix_opp1.y*inch_per_pix, 
		# 	theta=pix_opp1.theta)
		# world_opp2 = Pose2D(
		# 	x=pix_opp2.x*inch_per_pix, 
		# 	y=pix_opp2.y*inch_per_pix, 
		# 	theta=pix_opp2.theta)
		# world_ball = Pose2D(
		# 	x=pix_ball.x*inch_per_pix, 
		# 	y=pix_ball.y*inch_per_pix, 
		# 	theta=pix_ball.theta)
		est_msg = VisionState()
		est_msg.ally1 = world_ally1
		est_msg.ally2 = world_ally2
		est_msg.opp1 = world_opp1
		est_msg.opp2 = world_opp2
		est_msg.ball = world_ball
		self.publisher.publish(est_msg)


	def desired_estimate(self, desired_pos):
		world_pos = self.convertToWorld(desired_pos)
		self.des_pub.publish(world_pos)


	def pose2array(self, pose_msg):
		return np.array([pose_msg.x,pose_msg.y,pose_msg.theta])


	def array2pose(self, arr):
		return Pose2D(arr[0], arr[1], arr[2])


	def convertToWorld(self, pose_msg):
		arr = self.pose2array(pose_msg)
		world_msg = self.array2pose(arr * self.convert)
		return world_msg


class Estimator(object):

	
	def __init__(self):
		self.current_pos_estimate = None
		self.current_vel_estimate = None
		# What should these be initialized to?
		self.delayed_pos_estimate = np.array([0.0,0,0])
		self.delayed_vel_estimate = np.array([0.0,0,0])


	def updateState(self, measured):

		alpha = EstimateManager.alpha
		beta = EstimateManager.beta
		T = EstimateManager.measure_period
		
		# low pass filter for position estimation
		self.current_pos_estimate = (
			alpha*self.delayed_pos_estimate + (1-alpha)*measured)

		derivative = (
			(self.current_pos_estimate-self.delayed_pos_estimate)/T)

		self.current_vel_estimate = (
			beta*self.delayed_vel_estimate + (1-beta)*derivative)

		# This come after velocity estimation
		self.delayed_pos_estimate = self.current_pos_estimate
		self.delayed_vel_estimate = self.current_vel_estimate
	

	#TODO: Need to update these tustin derivative things for the Estimator
	def _tustin_derivative(self, x, Ts):
		"""Tustin Derivative
		Using the Tustin Approximation (Bilinear Transform), compute
		a discrete dirty-derivative.
		"""
		return (2*self.tau-Ts)/(2*self.tau+Ts)*self.xdot + 2/(2*self.tau+Ts)*(x-self.x_d1)

	def _tustin_integral(self, x, Ts):
		"""Tustin Integral
		Using the Tustin approximation (Bilinear Transform), compute
		a discrete integral.
		"""
		return self.integrator + (Ts/2)*(x+self.x_d1)

def main():
	rospy.init_node('estimation_node')
	pub = rospy.Publisher('/wc_estimation', VisionState, queue_size=10)
	des_pub = rospy.Publisher('/wc_des_estimation', Pose2D, queue_size=10)
	est = EstimateManager(pub)
	est.des_pub = des_pub


	# We will be subscribing to vision and game state
	rospy.Subscriber('/wc_vision_state', VisionState, est.estimate)
	rospy.Subscriber('/wc_click_desired_position',Pose2D,est.desired_estimate)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()