#!/usr/bin/env python

import cv2 as cv
import cv_bridge as cvb
import numpy as np
import rospy
from sensor_msgs.msg import Image as img_msg
from geometry_msgs.msg import Pose2D


flag = True
pub = None
g_pos = [100,100,0]

def _process_image(msg):
	global pub
	bridge = cvb.CvBridge()
	image = bridge.imgmsg_to_cv2(msg)
	x,y,theta = visionProcess(image)
	pos = Pose2D(x=x, y=y, theta=theta)
	pub.publish(pos)


def visionProcess(image):
	image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

	# cv.namedWindow('color',800)
	# nothing = lambda x: x
	# cv.createTrackbar('Hmin','color',0,179,nothing)
	# cv.createTrackbar('Hmax','color',0,255,nothing)
	# cv.createTrackbar('Smin','color',0,255,nothing)
	# cv.createTrackbar('Smax','color',0,255,nothing)
	# cv.createTrackbar('Vmin','color',0,255,nothing)
	# cv.createTrackbar('Vmax','color',0,255,nothing)

	# lowb = np.array([cv.getTrackbarPos('Hmin','color'),cv.getTrackbarPos('Smin','color'),cv.getTrackbarPos('Vmin','color')])
	# upb = np.array([cv.getTrackbarPos('Hmax','color'),cv.getTrackbarPos('Smax','color'),cv.getTrackbarPos('Vmax','color')])
	lowb = np.array([60,50,200])
	upb = np.array([100,100,255])
	thresh = cv.GaussianBlur(image,(5,5),0)
	thresh = cv.inRange(image, lowb, upb)
	cv.imshow('frame',thresh)
	cv.waitKey(1)
	cnts = cv.findContours(thresh.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[1]
	# print 'Detected {} contours'.format(len(cnts))
	try:
		cs = []
		for c in cnts:
			M = cv.moments(c)
			cX = int(M['m10']/M['m00'])
			cY = int(M['m01']/M['m00'])
			area = int(M['m00'])
			cs.append([cX,cY,area])
		# print cs
		c0 = cs[0] if cs[0][2] > cs[1][2] else cs[1]
		c1 = cs[0] if cs[0][2] < cs[1][2] else cs[1]

		vx = c0[0] - c1[0]
		vy = c0[1] - c1[1]

		theta = np.arccos(vx/(np.sqrt(vx**2+vy**2)))
		x = (c0[0] + c1[0]) / 2
		y = (c0[1] + c1[1]) / 2
		global g_pos
		g_pos = (x,y,theta) 
		return g_pos
	except:
		global g_pos
		return g_pos


def main():
	global g_pos
	g_pos = [100,100,0]
	rospy.init_node('image_processor')

	# We will be subscribing to vision and game state
	rospy.Subscriber('/usb_cam_away/image_raw', img_msg, _process_image)

	global pub
	pub = rospy.Publisher('wc_position', Pose2D, queue_size=10)
	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	# visionProcess()
	main()