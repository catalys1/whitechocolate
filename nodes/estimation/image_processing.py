#!/usr/bin/env python

import cv2 as cv
import cv_bridge as cvb
import numpy as np
import rospy
from sensor_msgs.msg import Image as img_msg
from geometry_msgs.msg import Pose2D


class VisionProcessor(object):

	def __init__(self):
		self.pub = None
		self.position_r = [100,100,0]
		self.position_b = [100,100]
		self.cv_bridge = cvb.CvBridge()

		self.click_x = 0
		self.click_y = 0
		self.show_val = False
		self.toggle_view = True
		self.toggle_obj = True

		self.lowb_r = np.array([85,85,220])
		self.upb_r  = np.array([106,125,250])

		self.lowb_b = np.array([120,25,150])
		self.upb_b  = np.array([150,60,240])

		self.thresh = None

	def click_point(self,event,x,y,flags,params):
		'''Save the point that got clicked on in the image
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self.click_x = x
			self.click_y = y
			self.show_val = True
		if event == cv.EVENT_RBUTTONDOWN:
			self.toggle_view = not self.toggle_view
		if event == cv.EVENT_MBUTTONDOWN:
			self.toggle_obj = not self.toggle_obj

	def processImage(self, msg):
		image = self.cv_bridge.imgmsg_to_cv2(msg)
		img = cv.cvtColor(image, cv.COLOR_BGR2HSV) 
		x_r,y_r,theta_r = self.trackRobot(img.copy())
		x_r,y_r,theta_r = [0,0,0]
		x_b,y_b = self.trackBall(img)
		pos = Pose2D(x=x_r, y=y_r, theta=theta_r)
		self.pub.publish(pos)

		if self.show_val:
			print_obj = 'Robot' if self.toggle_obj else 'Ball'
			print '{}: {}'.format(print_obj,img[self.click_y, self.click_x])
			self.show_val = False
			if self.toggle_obj:	
				self.lowb_r = np.array(img[self.click_y, self.click_x])-12
				self.upb_r = np.array(img[self.click_y, self.click_x])+12
			else:
			# Ball
				self.lowb_b = np.array(img[self.click_y, self.click_x])-20
				self.upb_b = np.array(img[self.click_y, self.click_x])+20
			
		# Change the upper and lower threshold bounds based on the
		# color that got clicked on
		x,y = [0,0]
		if self.toggle_obj:
			# Robot
			x,y = self.position_r[:2]
		else:
			# Ball
			x,y = self.position_b

		if self.toggle_view:
			img[y-2:y+2,x-2:x+2] = np.tile([0,0,0], (4,4,1))
			cv.imshow('frame',img)
		else:
			self.thresh[y-2:y+2,x-2:x+2] = np.tile(150, (4,4))
			cv.imshow('frame',self.thresh)
		cv.setMouseCallback('frame', self.click_point)
		cv.waitKey(1)

	def trackBall(self, img):
		thresh = self.staticThresh(img, self.lowb_b, self.upb_b)
		if not self.toggle_obj:
			self.thresh = thresh
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		try:
			cs = []
			for c in cnts:
				M = cv.moments(c)
				cX = int(M['m10']/M['m00'])
				cY = int(M['m01']/M['m00'])
				area = int(M['m00'])
				cs.append([cX,cY,area])
				
			x = c0[0]
			y = c0[1]
			self.position_b = (x,y) 
		except:
			pass
		return 0,0

	def trackRobot(self, image):
		# image = image[10:440, 120:725] #crop to only show the field
		
		thresh = self.staticThresh(image, self.lowb_r, self.upb_r)
		kernel = np.ones((5,5),np.uint8)
		thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		if self.toggle_obj:
			self.thresh = thresh
		cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
		#print 'Detected {} contours'.format(len(cnts))
		
		try:
			cs = []
			for c in cnts:
				M = cv.moments(c)
				cX = int(M['m10']/M['m00'])
				cY = int(M['m01']/M['m00'])
				area = int(M['m00'])
				cs.append([cX,cY,area])
				
			c0 = cs[0] if cs[0][2] > cs[1][2] else cs[1]
			c1 = cs[0] if cs[0][2] < cs[1][2] else cs[1]

			vx = c0[0] - c1[0]
			vy = c0[1] - c1[1]

			theta = np.arccos(vx/(np.sqrt(vx**2+vy**2)))
			if vy > 0:
				theta = 2*np.pi - theta
			x = (c0[0] + c1[0]) / 2
			y = (c0[1] + c1[1]) / 2
			self.position_r = (x,y,theta) 
		except:
			pass

		return self.position_r

	def staticThresh(self, img, lowb, upb):
		thresh = cv.inRange(img, lowb, upb)
		# kernel = np.ones((5,5),np.uint8)
		# thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		# thresh = cv.erode(thresh,kernal,iterations=1)
		# thresh = cv.dilate(thresh,kernal,iterations=1
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



class VisionProcessorBall(object):

	def __init__(self):
		self.pub = None
		self.position_b = [100,100]
		self.cv_bridge = cvb.CvBridge()

		self.click_x = 0
		self.click_y = 0
		self.show_val = False
		self.toggle_view = True

		self.lowb_b = np.array([120,25,150])
		self.upb_b  = np.array([150,60,240])

		self.thresh = None

	def click_point(self,event,x,y,flags,params):
		'''Save the point that got clicked on in the image
		'''
		if event == cv.EVENT_LBUTTONDOWN:
			self.click_x = x
			self.click_y = y
			self.show_val = True
		if event == cv.EVENT_RBUTTONDOWN:
			self.toggle_view = not self.toggle_view

	def processImage(self, msg):
		image = self.cv_bridge.imgmsg_to_cv2(msg)
		img = cv.cvtColor(image, cv.COLOR_BGR2HSV) 
		x_b,y_b = self.trackBall(img)
		pos = Pose2D(x=x_b, y=y_b, theta=0)
		self.pub.publish(pos)

		if self.show_val:
			self.lowb_b = np.array(img[self.click_y, self.click_x])-20
			self.upb_b = np.array(img[self.click_y, self.click_x])+20
			print 'Ball: {}'.format(img[self.click_y, self.click_x])
			self.show_val = False
			
		# Change the upper and lower threshold bounds based on the
		# color that got clicked on
		# Ball
		x,y = self.position_b

		if self.toggle_view:
			img[y-2:y+2,x-2:x+2] = np.tile([0,0,0], (4,4,1))
			cv.imshow('frame',img)
		else:
			self.thresh[y-2:y+2,x-2:x+2] = np.tile(150, (4,4))
			cv.imshow('frame',self.thresh)
		cv.setMouseCallback('frame', self.click_point)
		cv.waitKey(1)

	def trackBall(self, img):
		thresh = self.staticThresh(img, self.lowb_b, self.upb_b)
		kernel = np.ones((3,3),np.uint8)
		thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		thresh = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)
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
		thresh = cv.inRange(img, lowb, upb)
		# kernel = np.ones((5,5),np.uint8)
		# thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
		# thresh = cv.erode(thresh,kernal,iterations=1)
		# thresh = cv.dilate(thresh,kernal,iterations=1
		self.thresh = thresh
		return thresh


def main():
	vis_proc = VisionProcessorBall()
	rospy.init_node('image_processor')

	pub = rospy.Publisher('wc_position', Pose2D, queue_size=10)
	vis_proc.pub = pub

	# We will be subscribing to vision and game state
	rospy.Subscriber('/usb_cam_away/image_raw', img_msg, vis_proc.processImage)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()