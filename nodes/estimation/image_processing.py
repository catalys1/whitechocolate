#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image as img_msg


def _process_image(msg):
	print type(msg.data)


def main():
	# vision is our VisionExtimator object
	rospy.init_node('image_processor')

	# We will be subscribing to vision and game state
	rospy.Subscriber('/test/usb_cam/image_raw', img_msg, _process_image)

	# Don't exit until the node is stopped externally
	rospy.spin()


if __name__ == '__main__':
	main()