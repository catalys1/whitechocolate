#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState

import json
import numpy as np
import time
import filters


# TODO:
#
# - Better coordinate system
#    * The current coordinate system is measured from the
#	   southeast corner of the image in inches
#	 * We probably want to move the origin to the center of the
#      field and change the units to meters for consistency
#
# - Create a different estimation message system?
#    * different messages for each unit?
#    * message contains position and velocity?


class RobotEstimator(filters.LowPassFilter):
	'''A subclass of LowPassFilter used for estimating the position of the 
	Robot.
	'''

	def __init__(self, T_ctrl, alpha):

		# If any parameters need to be changed, do it here

		super(RobotEstimator,self).__init__(T_ctrl=T_ctrl, alpha=alpha)



class BallEstimator(filters.LowPassFilter):
	'''A subclass of LowPassFilter used for estimating the position of the
	ball.
	'''

	def __init__(self, T_ctrl, alpha):

		# If any parameters need to be changed, do it here

		super(BallEstimator,self).__init__(T_ctrl=T_ctrl, alpha=alpha)



class EstimateManager(object):

	measure_period = 1.0/30	 # camera frame rate
	ctrl_period = 1.0/100	 # rate of sending out estimates
	alpha = 0.1

	field_w = 126*0.0254     # field width in meters
	field_h = 88*0.0254      # field height in meters

	def __init__(self, publisher):

		f = '/home/robot/catkin_ws/src/whitechocolate/nodes/estimation/calibration/cal.json'
		self.params = json.load(open(f, 'r'))

		self._set_coordinate_system()

		self.publisher = publisher
		self.des_pub = None
		self.timestamp = time.time()

		self.ally1_est = RobotEstimator(self.ctrl_period, self.alpha)
		self.ally2_est = RobotEstimator(self.ctrl_period, self.alpha)
		self.opp1_est  = RobotEstimator(self.ctrl_period, self.alpha)
		self.opp2_est  = RobotEstimator(self.ctrl_period, self.alpha)
		self.ball_est  = BallEstimator(self.ctrl_period, self.alpha)

		self.world_ally1 = np.zeros(3)
		self.world_ally2 = np.zeros(3)
		self.world_opp1  = np.zeros(3)
		self.world_opp2  = np.zeros(3)
		self.world_ball  = np.zeros(3)


	def _set_coordinate_system(self):
		center_pix = self.params['center']
		self.pix_per_meter = int(
			(center_pix[0]/(self.field_w/2) + center_pix[1]/(self.field_h/2))/2
		)
		self.pix_offset = np.array(center_pix)
		self.conversion = np.array([1./self.pix_per_meter,1./self.pix_per_meter])


	def savePositions(self, vision_msg):
		'''Take the positions (in pixels) from the vision message and
		calculate and save the positions in world coordinates.
		'''
		# Our coordinate system is relative to the top left
		# corner of the field currently on the image produced by vision
		# Real world - this is the corner closest to our station/SE corner
		self.world_ally1 = self.pix_to_world(self.pose2array(vision_msg.ally1))
		self.world_ally2 = self.pix_to_world(self.pose2array(vision_msg.ally2))
		self.world_opp1  = self.pix_to_world(self.pose2array(vision_msg.opp1))
		self.world_opp2  = self.pix_to_world(self.pose2array(vision_msg.opp2))
		self.world_ball  = self.pix_to_world(self.pose2array(vision_msg.ball))
		# keep track of when the message was received
		self.timestamp = time.time()


	def estimate(self):
		'''Estimate positions and velocities and publish them.
		Not currently publishing velocities.
		'''
		# low pass filter
		Ts = time.time() - self.timestamp
		pos_ally1 = self.ally1_est.update(Ts, self.world_ally1)
		pos_ally2 = self.ally1_est.update(Ts, self.world_ally2)
		pos_opp1  = self.ally1_est.update(Ts, self.world_opp1)
		pos_opp2  = self.ally1_est.update(Ts, self.world_opp2)
		pos_ball  = self.ally1_est.update(Ts, self.world_ball)

		# Here we will want to grab the velocities and add them to the message

		est_msg = VisionState()
		est_msg.ally1 = self.array2pose(pos_ally1)
		est_msg.ally2 = self.array2pose(pos_ally2)
		est_msg.opp1  = self.array2pose(pos_opp1)
		est_msg.opp2  = self.array2pose(pos_opp2)
		est_msg.ball  = self.array2pose(pos_ball)
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
		world_msg = self.array2pose(self.pix_to_world(arr))
		return world_msg


	def pix_to_world(self, pix_coord):
		# set up the array, get theta in there
		world = pix_coord
		# convert the pixel x and y to world coordinates
		world[:-1] = (pix_coord[:-1]-self.pix_offset)*self.conversion
		return world

	

def main():
	rospy.init_node('estimation_node')
	pub = rospy.Publisher('/wc_estimation', VisionState, queue_size=10)
	des_pub = rospy.Publisher('/wc_des_estimation', Pose2D, queue_size=10)
	est = EstimateManager(pub)
	est.des_pub = des_pub

	# We will be subscribing to vision and game state
	# Need to add game state subscription
	rospy.Subscriber('/wc_vision_state', VisionState, est.savePositions)
	rospy.Subscriber('/wc_click_desired_position',Pose2D,est.desired_estimate)

	rate = rospy.Rate(est.ctrl_period)
	while not rospy.is_shutdown():
		# Do stuff
		est.estimate()

	# Don't exit until the node is stopped externally
	# get rid of this once the while loop is filled out
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()