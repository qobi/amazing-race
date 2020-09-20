# Import Libraries
import cv2
import time
import darknet
import random

# detection in online fashion
def detect_objects(args, image, net, meta):
	# convert image to rgb
	image_rgb = image[:,:,(2,1,0)]
	frame_height, frame_width, channels = image_rgb.shape

	# send frame through the network pass
	#start_time = time.time()
	res = darknet.detect(net, meta, image_rgb, thresh=args['darknet_thresh'], hier_thresh=args['darknet_hier_thresh'], nms=args['darknet_nms'])
	#end_time = time.time()
	#elapsed = end_time - start_time
	#print elapsed

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
		
		if s > args['darknet_thresh']:
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
			if x1>=frame_width:  x1=frame_width-1
			if x2>=frame_width:  x2=frame_width-1
			if y1>=frame_height: y1=frame_height-1
			if y2>=frame_height: y2=frame_height-1

			detections.append([x1, y1, x2, y2, s, c])

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

