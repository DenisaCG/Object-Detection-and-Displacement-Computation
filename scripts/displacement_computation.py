#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from imutils import paths
import numpy as np
import imutils
import cv2

FOCAL_LENGTH = 190 # manually computed in cm
"""
THIS IS A CALIBRATION STEP WITH A SAMPLE IMAGE AND MEASUREMENTS
first, we get the marker (largest object in image)
focal length = (marker[0][1] * 10)/14
"""

# function to find the duckiebot (assume largest object in image)
def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)
	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our duckiebot in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

# function to get distance from object to camera
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

def manipulate(data):
    np_arr = np.fromstring(data.data, np.uint8)
    # converting image to a numpy array
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # getting (x, y)-coordinates and width and height of the duckiebot (in pixels)
    marker = find_marker(image_np)
    # computing actual distance using precomputed focal length
    actual_distance = distance_to_camera(14, FOCAL_LENGTH, marker[1][0])
    # printing distance from camera to duckiebot
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", actual_distance)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    manipulate(data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/eblan/camera_node/image/compressed", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




