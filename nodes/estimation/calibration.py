#!/usr/bin/python

import cv2 as cv
import cv_bridge as cvb
import numpy as np
import json
import rospy
from sensor_msgs.msg import Image as img_msg
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState



class Calibrator(object):
	
	def __init__(self):
		self.cv_bridge = cvb.CvBridge()
		self.show_val = False
		
		self.states = {
			'bounds': 0,
			'center': 1,
			'robot':  2,
			'ball':   3
		}
		self.state = self.states['bounds']

		self.lbdown = False
		
		self.cal_file = (
		'/home/robot/catkin_ws/src/whitechocolate/nodes/estimation/calibration/cal.json'
		)
		self.params = json.load(open(self.cal_file))

		self._reset()
		self._toggle = False

		self.mouse_pos = np.zeros(2)
		self.ball_thresh = np.zeros((2,3))

		# TESTING
		self.thresh = np.array([135,135,135])


	def _reset(self):
		self.box = np.zeros((2,2), np.uint16)
		self.point = np.zeros(2, np.uint16)
		self.center = None
		self.placed = False
		self.crop   = False


	def mouse_event(self,event,x,y,flags,params):
		'''Save the point that got clicked on in the image
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self.box[0,:] = [x,y]
			self.lbdown = True
			self.placed = True
		elif event == cv.EVENT_LBUTTONUP:
			self.lbdown = False
			self.box[1,:] = [x,y]
		elif event == cv.EVENT_MOUSEMOVE:
			if self.lbdown:
				self.box[1,:] = [x,y]
			elif not self.placed:
				self.point[...] = [x,y]


	def ball_mouse(self, event, x, y, flags, params):
		'''
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self.mouse_pos[:] = [x,y]
			p = params[y-2:y+2, x-2:x+2, :]
			m = [np.min(p[...,0]), np.min(p[...,1]), np.min(p[...,2])]
			M = [np.max(p[...,0]), np.max(p[...,1]), np.max(p[...,2])]
			self.ball_thresh[0,:] = m
			self.ball_thresh[1,:] = M
			self.params['ball'] = np.uint8(self.ball_thresh).tolist()
			print m, M


	def process(self, msg):

		image = self.cv_bridge.imgmsg_to_cv2(msg)[...,::-1]
		image = cv.cvtColor(image, cv.COLOR_RGB2HSV)
		if self.crop:
			image = image[self.box[0][1]:self.box[1][1]+1, self.box[0][0]:self.box[1][0]+1]
		if self._toggle and self.state == self.states['ball']:
			image = self._staticThresh(image, self.ball_thresh[0], self.ball_thresh[1])
		cv.imshow('Cal', image)

		if self.state == self.states['bounds']:
			cv.setMouseCallback('Cal', self.mouse_event)
			self._setBounds(image)
		elif self.state == self.states['ball']:
			self._ballVision(image)
			cv.setMouseCallback('Cal', self.ball_mouse, image)

		if self.center is not None:
			cv.circle(image, tuple(self.center), 10, (255,0,0), 1)
		# img = np.uint8(image[:,:]<self.thresh)*255
		k = cv.waitKey(1) & 0xFF
		self._keyPress(k)


	def _keyPress(self, k):

		if k < 255:
			print k
		# State change keys
		if k == 102:  # "F" key pressed
			self._switchState('bounds')
		elif k == 99:   # "C" key pressed
			self._switchState('center')
		elif k == 114:  # "R" key pressed
			self._switchState('robot')
		elif k == 98:   # "B" key pressed
			self._switchState('ball')
		elif k == 113:  # "Q" key pressed
			# start over
			self._switchState('bounds')
			self._reset()
		elif k == 115:  # "S" key pressed
			# Save
			print 'Saved'
			json.dump(self.params, open(self.cal_file,'w'))
		elif k == 116:  # "T" key pressed
			self._toggle = not self._toggle

		if self.state == self.states['bounds']:
			if k == 13:  # "Enter" key pressed	
				self.crop = True
				self.params['box'] = self.box.tolist()
				x = self.params['box'][1][0] - self.params['box'][0][0]
				y = self.params['box'][1][1] - self.params['box'][0][1]
				self.center = [x/2, y/2]
				self.params['center'] = self.center
				self._switchState('robot')
			elif k == 82:
				self.thresh += 10
				print self.thresh
			elif k == 84:
				self.thresh -= 5
				print self.thresh


	def _switchState(self, state_str):
		self.state = self.states[state_str]
		print 'State: {}'.format(state_str)


	def _setBounds(self, image):
		# print 'set bounds'

		if self.placed:
			cv.rectangle(image, tuple(self.box[0,:]), tuple(self.box[1,:]), (0,0,255), 2)
		else:
			# print 'not placed'
			p1 = [0, self.point[1]]
			p2 = [image.shape[1]-1, self.point[1]]
			cv.line(image, tuple(p1), tuple(p2), (0,0,255), 2)
			p1 = [self.point[0], 0]
			p2 = [self.point[0], image.shape[0]-1]
			cv.line(image, tuple(p1), tuple(p2), (0,0,255), 2)


	def _staticThresh(self, img, lowb, upb):
		thresh = cv.medianBlur(img, 5)
		thresh = cv.inRange(thresh, lowb, upb)
		self.thresh = thresh
		return thresh


	def _ballVision(self, image):
		pass




def main():
	cal = Calibrator()
	rospy.init_node('vision_node')

	# We will be subscribing to vision and game state
	rospy.Subscriber('/usb_cam_away/image_raw', img_msg, cal.process)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	main()