#!/usr/bin/env python

import cv2 as cv
import cv_bridge as cvb
import numpy as np
import rospy
import json
from sensor_msgs.msg import Image as img_msg
from geometry_msgs.msg import Pose2D
from whitechocolate.msg import VisionState


# TODO:
#
# - Create tracking for opponent(s)
#
# - Play and Calibration seperation
#
# - Our original plan was to have this in c++. Do we want to port it?


class VisionProcessor(object):
	
	def __init__(self, **kwargs):
		self.click_x = 100
		self.click_y = 100
		self.desired_x = 200
		self.desired_y = 200
		self.show_val = False

		if 'thresh' in kwargs:
			v = np.array(kwargs['thresh'])
			self.lowb = v - 20
			self.upb = v + 20


	def click_point(self,event,x,y,flags,params):
		'''Save the point that got clicked on in the image
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self.click_x = x
			self.click_y = y
			self.show_val = True
			print self.click_x, self.click_y
		if event == cv.EVENT_RBUTTONDOWN:
			self.toggle_view = not self.toggle_view
		if event == cv.EVENT_MBUTTONDOWN:
			self.desired_x = x
			self.desired_y = y
			print 'Desired position: x={}, y={}'.format(
				self.desired_x, self.desired_y)


class VisionProcessorRobot(VisionProcessor):

	def __init__(self, **kwargs):
		super(VisionProcessorRobot,self).__init__(**kwargs)
		self.position_r = [100,100,0]

		self.toggle_view = True

		if 'thresh' not in kwargs:
			self.lowb = np.array([85,85,220])
			self.upb  = np.array([106,125,250])

		self.thresh = None


	def processImage(self, img):
		# image = self.cv_bridge.imgmsg_to_cv2(msg)
		# img = cv.cvtColor(image, cv.COLOR_BGR2HSV) 
		x_r,y_r,theta_r = self.trackRobot(img.copy())
		pos = Pose2D(x=x_r, y=y_r, theta=theta_r)
		# self.pub.publish(pos)

		if self.show_val:
			print 'Robot: {}'.format(img[self.click_y, self.click_x])
			self.show_val = False
			# Change the upper and lower threshold bounds based on the
			# color that got clicked on
			self.lowb = np.array(img[self.click_y, self.click_x])-20
			self.upb = np.array(img[self.click_y, self.click_x])+20

			
		x,y = self.position_r[:2]

		if self.toggle_view:
			img[y-2:y+2,x-2:x+2] = np.tile([0,0,0], (4,4,1))
			cv.imshow('Robot',img)
		else:
			self.thresh[y-2:y+2,x-2:x+2] = np.tile(150, (4,4))
			cv.imshow('Robot',self.thresh)
		cv.setMouseCallback('Robot', self.click_point)
		cv.waitKey(1)
		return pos

	def trackRobot(self, image):
		# image = image[10:440, 120:725] #crop to only show the field
		thresh = self.staticThresh(image, self.lowb, self.upb)
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		#print 'Detected {} contours'.format(len(cnts))
		
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

			# vector from smaller to larger contour centers
			vx = c0[0] - c1[0]
			vy = c0[1] - c1[1]

			# get the angle of the vector
			# theta = np.arccos(vx/(np.sqrt(vx**2+vy**2)))
			theta = np.arctan(vy/vx)
			if vx < 0:
				theta = (np.pi - theta) 
				if theta < 0:
					theta += 2*np.pi
			x = (c0[0] + c1[0]) / 2
			y = (c0[1] + c1[1]) / 2
			self.position_r = (x,y,theta*57.3) 
		except:
			pass

		return self.position_r

	def staticThresh(self, img, lowb, upb):
		thresh = cv.inRange(img, lowb, upb)
		kernel = np.ones((5,5),np.uint8)
		thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		# thresh = cv.erode(thresh,kernal,iterations=1)
		# thresh = cv.dilate(thresh,kernal,iterations=1
		self.thresh = thresh
		return thresh

	def adaptiveThresh(self, image):
		H = image[...,0]
		S = image[...,1]
		V = image[...,2]

		# Ht = cv.inRange(cv.GaussianBlur(H,(3,3),0), 50, 200)
		Ht = cv.adaptiveThreshold(H,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	            cv.THRESH_BINARY,9,3)
		St = cv.inRange(cv.GaussianBlur(H,(3,3),0), 100, 200)
		# St = cv.adaptiveThreshold(S,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	 #            cv.THRESH_BINARY,9,3)
		Vt = cv.inRange(cv.GaussianBlur(H,(3,3),0), 200, 256)
		# Vt = cv.adaptiveThreshold(V,255,cv.ADAPTIVE_THRESH_MEAN_C,\
	 #            cv.THRESH_BINARY,9,3)
		thresh = np.bitwise_and(Ht,St,Vt)
		return thresh


	def otsu(self, img):
		pass



class VisionProcessorBall(VisionProcessor):

	def __init__(self, **kwargs):
		super(VisionProcessorBall,self).__init__(**kwargs)
		self.position_b = [100,100]

		self.toggle_view = True

		if 'thresh' not in kwargs:
			self.lowb = np.array([120,25,150])
			self.upb  = np.array([150,60,240])

		self.thresh = None

		self.background = None


	def processImage(self, img):

		if self.background is None:
			self.background = img.copy()

		x_b,y_b = self.trackBall(img)
		pos = Pose2D(x=x_b, y=y_b, theta=0)
		# self.pub.publish(pos)

		if self.show_val:
			self.lowb = np.maximum(0, np.array(
				img[self.click_y, self.click_x],np.int16)-[5,30,20])
			self.upb = np.minimum(255, np.array(
				img[self.click_y, self.click_x],np.int16)+[5,30,20])
			print 'Ball: {}'.format(img[self.click_y, self.click_x])
			# print self.lowb
			# print self.upb
			self.show_val = False
			
		# Change the upper and lower threshold bounds based on the
		# color that got clicked on
		x,y = self.position_b

		if self.toggle_view:
			img[y-2:y+2,x-2:x+2] = np.tile([0,0,0], (4,4,1))
			cv.imshow('Ball',img[...,0])
		else:
			self.thresh[y-2:y+2,x-2:x+2] = np.tile(150, (4,4))
			cv.imshow('Ball',self.thresh)
		cv.setMouseCallback('Ball', self.click_point)
		cv.waitKey(1)
		return pos

	def trackBall(self, img):

		thresh = self.staticThresh(img, self.lowb, self.upb)
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		try:
			cs = []
			for c in cnts:
				M = cv.moments(c)
				cX = int(M['m10']/M['m00'])
				cY = int(M['m01']/M['m00'])
				area = int(M['m00'])
				if area > 3:
					cs.append([cX,cY,area])
			# if cnts == 1:	
			x = cs[0][0]
			y = cs[0][1]
			# print cs
			self.position_b = (x,y) 
		except:
			pass

		return self.position_b


	def staticThresh(self, img, lowb, upb):
		# thresh = cv.GaussianBlur(img, (7,7), 0, 0)
		thresh = cv.medianBlur(img, 5)
		thresh = cv.inRange(thresh, lowb, upb)
		# kernel = np.ones((3,3),np.uint8)
		# thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		# thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		# thresh = cv.erode(thresh,kernal,iterations=1)
		# thresh = cv.dilate(thresh,kernal,iterations=1
		self.thresh = thresh
		return thresh


	def backSub(self, img, lowb, upb):
		thresh = np.subtract(img, self.background)
		thresh = np.where(thresh > 20, thresh, 0)
		self.thresh = thresh
		return thresh


	def otsu(self, img):
		img_dist = 180-np.abs(np.int32(img[...,0]) - (self.lowb[0]-self.upb[0])/2)
		_,thresh = cv.threshold(np.uint8(img_dist),150,255,cv.THRESH_BINARY)
		thresh = np.uint8(thresh)
		self.thresh = thresh
		return thresh


class VisionManager(object):

	def __init__(self):
		f = '/home/robot/catkin_ws/src/whitechocolate/nodes/estimation/calibration/cal.json'
		self.params = json.load(open(f))

		self.robot1 = VisionProcessorRobot(thresh=self.params['ally1'])
		self.ball = VisionProcessorBall(thresh=self.params['ball'])
		self.pub = None
		self.des_pub = None
		self.cv_bridge = cvb.CvBridge()



	def processVision(self, msg):

		box = self.params['box']
		image = self.cv_bridge.imgmsg_to_cv2(msg)
		img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
		img = img[box[0][1]:box[1][1]+1, box[0][0]:box[1][0]+1]

		ally1 = self.robot1.processImage(img)
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
			x=self.robot1.desired_x,
			y=self.robot1.desired_y,
			theta=0
		)

		self.pub.publish(vis_msg)
		self.des_pub.publish(des_msg)


def main():
	vis_proc = VisionManager()
	rospy.init_node('vision_node')

	pub = rospy.Publisher('wc_vision_state', VisionState, queue_size=10)
	des_pub = rospy.Publisher('wc_click_desired_position',Pose2D,queue_size=10)
	vis_proc.pub = pub
	vis_proc.des_pub = des_pub

	# We will be subscribing to vision and game state
	rospy.Subscriber('/usb_cam_away/image_raw', img_msg, vis_proc.processVision)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()