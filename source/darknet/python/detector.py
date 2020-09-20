# Import Libraries
import os
import sys
dir_name = os.path.dirname(os.path.abspath(__file__))
darknet_dir = os.path.abspath(dir_name + '/../darknet')
sys.path.append(darknet_dir + '/python')
import darknet
import cv2
import random

def detect_objects(np_img, net, meta, darknet_thresh, darknet_hier_thresh, darknet_nms):
	# get frame limits
	image_height, image_width, channels = np_img.shape

	image_rgb = np_img[:,:,(2,1,0)]
	#im = darknet.nparray_to_image(np_img)
	# send frame through the network pass 
	res = darknet.detect_np(net, meta, np_img, thresh=darknet_thresh, hier_thresh=darknet_hier_thresh, nms=darknet_nms)

	# get a list of all the detections
	detections = []
	for detection_idx in range(len(res)):
		# extract detection data
		c = res[detection_idx][0]
		s = res[detection_idx][1]
		x, y, w, h = res[detection_idx][2]

		# error check values
		if x == float('inf') or y == float('inf') or h == float('inf') or w == float('inf'):
			continue
		
		if s > darknet_thresh:
			# convert to x1,y1,x2,y2
			x1 = int(x - w/2)
			y1 = int(y - h/2)
			x2 = int(x + w/2 -1)
			y2 = int(y + h/2 -1)

			# make sure coordinates aren't off screen
			if x1<0: x1=0
			if y1<0: y1=0
			if x2<0: x2=0
			if y2<0: y2=0
			if x1>=image_width:  x1=image_width-1
			if x2>=image_width:  x2=image_width-1
			if y1>=image_height: y1=image_height-1
			if y2>=image_height: y2=image_height-1

			detections.append([x1, y1, x2, y2, s, c])
		
		# debug
		#darknet.draw_results(res, image)
		#cv2.imshow('image',image)
		#cv2.waitKey(0)
		'''		
		# debug - display detections
		for detection_idx in range(len(detections)):
			x1, y1, x2, y2, s, c = detections[detection_idx]
			R = random.randint(0,255)
			G = random.randint(0,255)
			B = random.randint(0,255)

			cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (R,G,B), 4)
			cv2.putText(image, c, (int(x1)+15, int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (R,G,B),2);
		
	cv2.imshow("image", image)
	cv2.waitKey(0)
		'''

	return detections

