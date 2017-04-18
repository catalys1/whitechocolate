#!/usr/bin/env python

import cv2 as cv
import cv_bridge as cvb
import numpy as np
import rospy
import json
from sensor_msgs.msg import Image as img_msg
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState

import argparse


# TODO:
#
# - Create tracking for ally2
#
# - Create tracking for opponent(s)
#
# - Play and Calibration seperation
#
# - Our original plan was to have this in c++. Do we want to port it?


class VisionProcessor(object):
	
	def __init__(self, **kwargs):

		if 'thresh' in kwargs:
			v = np.array(kwargs['thresh'])
			self.lowb = v - 20
			self.upb = v + 20



class VisionProcessorRobot(VisionProcessor):

	def __init__(self, **kwargs):
		super(VisionProcessorRobot,self).__init__(**kwargs)
		self.position = [100,100,0]

		self.toggle_view = True

		if 'thresh' not in kwargs:
			self.lowb = np.array([85,85,220])
			self.upb  = np.array([106,125,250])

		self.thresh = None

		self.c0 = [0,0]
		self.c1 = [0,0]


	def processImage(self, img):
		x_r,y_r,theta_r = self.trackRobot(img.copy())
		pos = Pose2D(x=x_r, y=y_r, theta=theta_r)
		return pos


	def trackRobot(self, image):
		thresh = self.staticThresh(image, self.lowb, self.upb)
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		
		try:
			cs = []
			for c in cnts:
				M = cv.moments(c)
				cX = int(M['m10']/M['m00'])  # center x value
				cY = int(M['m01']/M['m00'])  # center y value
				area = int(M['m00'])         # area of the contour
				cs.append([cX,cY,area])
			
			# c0 is contour with greater area
			c0 = cs[0] if cs[0][2] > cs[1][2] else cs[1]
			c1 = cs[0] if cs[0][2] < cs[1][2] else cs[1]
			self.c0 = c0
			self.c1 = c1

			# vector from smaller to larger contour centers
			vx = c1[0] - c0[0]
			vy = c1[1] - c0[1]

			# get the angle of the vector
			dist = np.sqrt(vx**2+vy**2)+0.000000001
			theta = np.arccos(vx/dist)
			if vy < 0:
				theta = 2*np.pi - theta
			# theta = np.arctan(float(vy)/vx)
			# if vx < 0:
			# 	theta = (np.pi - theta) 
			# 	if theta < 0:
			# 		theta += 2*np.pi
			x = (c0[0] + c1[0]) / 2
			y = (c0[1] + c1[1]) / 2
			self.position = (x,y,np.rad2deg(theta)) 
		except:
			pass

		return self.position


	def staticThresh(self, img, lowb, upb):
		thresh = cv.inRange(img, lowb, upb)
		kernel = np.ones((5,5),np.uint8)
		thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		self.thresh = thresh
		return thresh


	def changeThresh(self, hsv, tolerence=20):
		self.lowb = np.maximum(0, hsv - tolerence)
		self.upb  = np.minimum(255, hsv + tolerence)


	# def adaptiveThresh(self, image):
	# 	H = image[...,0]
	# 	S = image[...,1]
	# 	V = image[...,2]

	# 	# Ht = cv.inRange(cv.GaussianBlur(H,(3,3),0), 50, 200)
	# 	Ht = cv.adaptiveThreshold(H,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	#             cv.THRESH_BINARY,9,3)
	# 	St = cv.inRange(cv.GaussianBlur(H,(3,3),0), 100, 200)
	# 	# St = cv.adaptiveThreshold(S,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	#  #            cv.THRESH_BINARY,9,3)
	# 	Vt = cv.inRange(cv.GaussianBlur(H,(3,3),0), 200, 256)
	# 	# Vt = cv.adaptiveThreshold(V,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	#  #            cv.THRESH_BINARY,9,3)
	# 	thresh = np.bitwise_and(Ht,St,Vt)
	# 	return thresh



class VisionProcessorBall(VisionProcessor):

	def __init__(self, **kwargs):
		super(VisionProcessorBall,self).__init__(**kwargs)
		self.position = [100,100]

		self.toggle_view = True

		if 'thresh' not in kwargs:
			self.lowb = np.array([115,2, 220])
			self.upb  = np.array([165,50,240])
		else:
			t = kwargs['thresh']
			self.lowb = np.array(t[0])
			self.upb = np.array(t[1])

		self.thresh = None


	def processImage(self, img):

		x_b,y_b = self.trackBall(img)
		pos = Pose2D(x=x_b, y=y_b, theta=0)
		return pos


	def trackBall(self, img):

		thresh = self.staticThresh(img, self.lowb, self.upb)
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		try:
			cs = []
			for c in cnts:
				M = cv.moments(c)
				try:
					cX = int(M['m10']/M['m00'])
					cY = int(M['m01']/M['m00'])
					area = int(M['m00'])
					if area > 3 and area < 30:
						cs.append([cX,cY,area])
				except:
					pass

			x = cs[0][0]
			y = cs[0][1]
			# print cs
			self.position = (x,y) 
		except Exception, e:
			# print e
			pass

		return self.position


	def staticThresh(self, img, lowb, upb):
		thresh = cv.medianBlur(img, 5)
		thresh = cv.inRange(thresh, lowb, upb)
		self.thresh = thresh
		return thresh


	def changeThresh(self, hsv, tolerence=[5,30,20]):
		self.lowb = np.uint8(np.maximum(0, hsv - tolerence))
		self.upb  = np.uint8(np.minimum(255, hsv + tolerence))
		print 'Ball changed threshold: [{}] to [{}]'.format(
			','.join(str(x) for x in self.lowb),
			','.join(str(x) for x in self.upb))


	# def backSub(self, img, lowb, upb):
	# 	thresh = np.subtract(img, self.background)
	# 	thresh = np.where(thresh > 20, thresh, 0)
	# 	self.thresh = thresh
	# 	return thresh


	# def otsu(self, img):
	# 	img_dist = 180-np.abs(np.int32(img[...,0]) - (self.lowb[0]-self.upb[0])/2)
	# 	_,thresh = cv.threshold(np.uint8(img_dist),150,255,cv.THRESH_BINARY)
	# 	thresh = np.uint8(thresh)
	# 	self.thresh = thresh
	# 	return thresh


class VisionManager(object):

	def __init__(self, display):
		f = '/home/robot/catkin_ws/src/whitechocolate/nodes/estimation/calibration/cal.json'
		self.params = json.load(open(f))

		self.ally1 = VisionProcessorRobot(thresh=self.params['ally1'])
		self.ball = VisionProcessorBall(thresh=self.params['ball'])
		self.pub = None
		self.des_pub = None
		self.cv_bridge = cvb.CvBridge()

		self.show_screen = display

		self.desired_x = 200
		self.desired_y = 200

		self.toggle_view = True


		self.states = {
			'ally1': 0,
			'ally2': 1,
			'opp1':  2,
			'opp2':  3,
			'ball':  4,
		}
		self.state = self.states['ally1']


	def _click_point(self,event,x,y,flags,params):
		'''Save the point that got clicked on in the image
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self._update_tracker(x, y)
			print 'Clicked: x={}, y={}'.format(x, y)

		elif event == cv.EVENT_MBUTTONDOWN:
			self.desired_x = x
			self.desired_y = y
			print 'Desired position: x={}, y={}'.format(
				self.desired_x, self.desired_y)


	def _update_tracker(self, x, y):
		hsv = np.int16(self.img[y, x, :])
		if self.state == self.states['ally1']:
			self.ally1.changeThresh(hsv)
			print 'Ally 1: {}'.format(hsv)
		elif self.state == self.states['ally2']:
			pass
		elif self.state == self.states['opp1']:
			pass			
		elif self.state == self.states['opp2']:
			pass
		elif self.state == self.states['ball']:
			self.ball.changeThresh(hsv)
			print 'Ball: {}'.format(hsv)


	def _keypress(self, k):
		if   k == 49:  # "1" key pressed
			self._switchState('ally1')
		elif k == 50:  # "2" key pressed
			self._switchState('ally2')
		elif k == 51:  # "3" key pressed
		 	self._switchState('opp1')
		elif k == 52:  # "4" key pressed
			self._switchState('opp2')
		elif k == 53:  # "5" key pressed
			self._switchState('ball')
		elif k == 116: # "T" key pressed
			self.toggle_view = not self.toggle_view


	def _switchState(self, state_str):
		self.state = self.states[state_str]
		print 'State: {}'.format(state_str)


	def processVision(self, msg):

		box = self.params['box']
		image = self.cv_bridge.imgmsg_to_cv2(msg)
		img = image[box[0][1]:box[1][1]+1, box[0][0]:box[1][0]+1]
		img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
		self.img = img

		ally1 = self.ally1.processImage(img)
		ally2 = ally1
		opp1 = opp2 = Pose2D()
		ball = self.ball.processImage(img)

		vis_msg = VisionState()
		vis_msg.ally1 = ally1
		vis_msg.ally2 = ally2
		vis_msg.opp1 = opp1
		vis_msg.opp1 = opp2
		vis_msg.ball = ball

		des_msg = Pose2D(
			x=self.desired_x,
			y=self.desired_y,
			theta=180.
		)
		self.pub.publish(vis_msg)
		self.des_pub.publish(des_msg)

		if self.show_screen:
			self.display(image)


	def display(self, image):
	
		x,y,thresh = self._getXYThresh()

		if self.toggle_view:
			cv.rectangle(self.img, (x-2,y-2), (x+2,y+2), (0,0,0), -1)
			if self.state == self.states['ally1'] and len(self.ally1.c1) > 2:
				x1,y1 = self.ally1.c0[:-1]
				x2,y2 = self.ally1.c1[:-1]
				cv.line(self.img, (x1,y1), (x2,y2), (0,0,0), 2)
				cv.rectangle(self.img, (x2-2,y2-2), (x2+2,y2+2), (0,0,0), -1)
			cv.imshow('Display',self.img)
		else:
			cv.rectangle(thresh, (x-2,y-2), (x+2,y+2), 150, -1)
			cv.imshow('Display',thresh)
		cv.setMouseCallback('Display', self._click_point)
		k = cv.waitKey(1) & 0xFF
		if k < 255:
			self._keypress(k)



	def _getXYThresh(self):

		# THIS IS RETURNING ALLY1 FOR ALL THE ROBOTS RIGHT NOW
		if self.state == self.states['ally1']:
			return (self.ally1.position[0],self.ally1.position[1],self.ally1.thresh)
		elif self.state == self.states['ally2']:
			return (self.ally1.position[0],self.ally1.position[1],self.ally1.thresh)
		elif self.state == self.states['opp1']:
			return (self.ally1.position[0],self.ally1.position[1],self.ally1.thresh)
		elif self.state == self.states['opp2']:
			return (self.ally1.position[0],self.ally1.position[1],self.ally1.thresh)
		elif self.state == self.states['ball']:
			return (self.ball.position[0],self.ball.position[1],self.ball.thresh)




def main(testing=True):
	vis_proc = VisionManager(testing)
	rospy.init_node('vision_node')

	pub = rospy.Publisher('vision_state', VisionState, queue_size=10)
	des_pub = rospy.Publisher('click_desired_position',Pose2D,queue_size=10)
	vis_proc.pub = pub
	vis_proc.des_pub = des_pub

	# We will be subscribing to vision and game state
	rospy.Subscriber('/usb_cam_away/image_raw', img_msg, vis_proc.processVision)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# parser = argparse.ArgumentParser()
	# parser.add_argument('-t', action='store_true', default=False, help='Testing mode')
	# args = parser.parse_args()	
	# main(args.t)
	main()