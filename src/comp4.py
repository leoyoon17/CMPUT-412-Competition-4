#!/usr/bin/env python
import numpy as np
import argparse
import imutils
import glob
import cv2
import time


def detect():
	cap = cv2.VideoCapture(1)
	time.sleep(3)
	template = cv2.imread("emblem.png", 0)
	(tH, tW) = template.shape[:2]
	#template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
	while True:
		ret, img = cap.read()
		#template = cv2.Canny(template, 50, 200)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		print(gray)
		cv2.imshow("Template", template)
		found = None
		for scale in np.linspace (0.2, 1.0, 20)[::-1]:
			resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
			r = gray.shape[1] / float(resized.shape[1])
 
			# if the resized image is smaller than the template, then break
			# from the loop
			if resized.shape[0] < tH or resized.shape[1] < tW:
				break
			# detect edges in the resized, grayscale image and apply template
			# matching to find the template in the image
			edged = cv2.Canny(resized, 50, 200) 
			result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
 
			# if we have found a new maximum correlation value, then ipdate
			# the bookkeeping variable
			if found is None or maxVal > found[0]:
				found = (maxVal, maxLoc, r)
 
			# unpack the bookkeeping varaible and compute the (x, y) coordinates
			# of the bounding box based on the resized ratio
			(maxVal, maxLoc, r) = found # We might check what is the max value and can create a rectangle based on that
			(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
			(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r)) #rectangle's width and height are tW and tH
 
		# draw a bounding box around the detected result and display the image
		cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
		cv2.imshow("Image", img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
        		break
	cap.release()
	cv2.destroyAllWindows()
detect()
