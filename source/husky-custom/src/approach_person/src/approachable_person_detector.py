#################################################################################################
# Import libaries
#################################################################################################
import os
import tf
import cv2
import math
import copy
import time
import rospy
import darknet
import numpy as np
import yolo_detector
from scipy.optimize import linear_sum_assignment

######################################################################################
# global variables
######################################################################################
net = None
meta = None
args = None
stationary_persons  = {}
approaching_persons = {}
next_tube_id = 0
tubes = {}

# get the transform
tf_ = tf.TransformListener()

######################################################################################
# get distance between two points
######################################################################################
def dist(point1, point2):
	x_dist = point1[0] - point2[0]
	y_dist = point1[1] - point2[1]
 	return math.sqrt(x_dist**2 + y_dist**2)
 	
######################################################################################
# create a frame of the tracks
######################################################################################
def draw_tracks_on_frame(frame, frame_idx):
	# refer to global objects
	global args, tubes, stationary_persons, approaching_persons

	# create copy of frame (so we don't spoil the original)
	temp_frame = copy.deepcopy(frame)

	# define colors 
	tube_color1 = (255,128,0)  # tube of person that just began (orange)
	tube_color2 = (51,153,255) # tube of person that is moving  (blue)
	tube_color3 = (77,255,77)  # tube of person we can approach (green)

	# write out legend
	cv2.putText(temp_frame, "Person tube (that just began)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tube_color1, 2);
	cv2.putText(temp_frame, "Person tube (that is moving)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tube_color2, 2);
	cv2.putText(temp_frame, "Person tube (that is approachable)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tube_color3, 2);
	cv2.putText(temp_frame, "Frame " + str(frame_idx), (args['frame_width']-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0),2);

	# loop over the tubes
	for tube_id, tube_data in tubes.iteritems():
		# get the coords
		x1, y1, x2, y2 = tube_data['image_coords']

		# determine whether this person is approachable
		approachable = 'WALKING AWAY ({0:0.1f} m; {1:0.1f} m/s)'.format(tube_data['dist_now'], tube_data['walk_rate'])
		if tube_id in stationary_persons:
			approachable = 'STATIONARY ({0:0.1f} m; {1:0.1f} m/s)'.format(tube_data['dist_now'], tube_data['walk_rate'])
		if tube_id in approaching_persons:
			approachable = 'APPROACHING ({0:0.1f} m; {1:0.1f} m/s)'.format(tube_data['dist_now'], tube_data['walk_rate'])
		
		# get the color of the person box
		R, G, B = tube_color2 # candidate for approachable
		
		len_tube = tube_data['frame_time'][-1] - tube_data['frame_time'][0]
		if (len_tube + 1) < args['tracker_num_seconds']: # tube that just began
			R, G, B = tube_color1
			approachable = 'PROCESSING WALK RATE'
		if 'STATIONARY' in approachable or 'APPROACHING' in approachable: # approachable person
			R, G, B = tube_color3 

		# draw box
		cv2.rectangle(temp_frame, (int(x1), int(y1)), (int(x2), int(y2)), (R,G,B), 4)
		cv2.putText(temp_frame, "person " + str(tube_id), (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (R,G,B),2)
		cv2.putText(temp_frame, approachable, (int(x1)+10, int(y2)-80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (R,G,B),2)
		
	# return frame
	return temp_frame

######################################################################################
# create a frame with object distances/locations class_label
# objects[tube_id]['class_label']           = 'label'
# objects[tube_id]['tube_id']               = tube_id
# objects[tube_id]['image_coords']          = [x1, y1, x2, y2] 
# objects[tube_id]['rel_2D_coords']         = [rel_x_dist_3d, rel_y_dist_3d] 
# objects[tube_id]['world_2D_coords']       = [world_x_dist_3d, world_y_dist_3d]
# objects[tube_id]['projected_points'][idx] = [x, y]
# objects[tube_id]['dist_now']              = distance_in_meters
######################################################################################
def draw_objects_on_frame(frame, objects, frame_idx):
	# create copy of frame (so we don't spoil the original)
	temp_frame = copy.deepcopy(frame)

	# define colors 
	tube_color = (51,153,255)

	# write out legend
	cv2.putText(temp_frame, "Object (relative and world coords)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tube_color, 2);
	cv2.putText(temp_frame, "Frame " + str(frame_idx), (args['frame_width']-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0),2);

	# loop over the objects
	for tube_id in objects.keys():
		# get the image coords
		x1, y1, x2, y2 = objects[tube_id]['image_coords']
		
		# draw box
		cv2.rectangle(temp_frame, (int(x1), int(y1)), (int(x2), int(y2)), tube_color, 4)
		cv2.putText(temp_frame, objects[tube_id]['class_label'] + " " + str(tube_id), (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, tube_color,2)

		# draw relative/world coordinates
		if 'rel_2D_coords' in objects[tube_id] and not objects[tube_id]['rel_2D_coords'] == [None, None]:
			text1 = "Rel   X: {:0.2f}".format(objects[tube_id]['rel_2D_coords'][0]) + ", Y: {:0.2f}".format(objects[tube_id]['rel_2D_coords'][1])
			text2 = "world X: {:0.2f}".format(objects[tube_id]['world_2D_coords'][0]) + ", Y: {:0.2f}".format(objects[tube_id]['world_2D_coords'][1])
			cv2.putText(temp_frame, text1, (int(x1)+10, int(y2)-20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, tube_color,2)
			cv2.putText(temp_frame, text2, (int(x1)+10, int(y2)-50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, tube_color,2)

		# draw projected points
		if 'projected_points' in objects[tube_id]: 
			for point in objects[tube_id]['projected_points']:
				cv2.line(temp_frame,(point[0],point[1]),(point[0]+1,point[1]+1),tube_color,3)
	
	# return annotated frame
	return temp_frame

######################################################################################
# resize detections to full-resolution (instead of relative to NN input height/width)
######################################################################################
def resize_detections(detections, width_refactor, height_refactor):
	# loop through detections
	for i in range(len(detections)):
		# get detection (based on NN input size)
		x1, y1, x2, y2, score, label = detections[i]
		
		# resize detection to full image scale
		resized_x1 = width_refactor * x1
		resized_x2 = width_refactor * x2
		resized_y1 = height_refactor * y1
		resized_y2 = height_refactor * y2

		# save off resized detections
		resized_detection = [resized_x1, resized_y1, resized_x2, resized_y2, score, label]
		detections[i] = resized_detection

	# return resized detections
	return detections

######################################################################################
# detect people
######################################################################################
def detect_people(frame):
	# refer to global objects
	global args

	# compute resize factors
	width_refactor  = float(args['frame_width'])  / args['input_width']
	height_refactor = float(args['frame_height']) / args['input_height']

	# resize frame
	resized_frame = cv2.resize(frame, (args['input_width'], args['input_height'])) 
	
	# get detections
	#start_time = time.time()
	detections = yolo_detector.detect_objects(args, resized_frame, net, meta)
	#end_time = time.time()
	#print "\tyolo time: ", end_time - start_time

	# resize detections
	detections = resize_detections(detections, width_refactor, height_refactor)
	
	# remove non-person detections
	persons = []
	for i in range(len(detections)):
		x1, y1, x2, y2, score, label  = detections[i]
		if label == 'person':
			persons.append(detections[i])

	# return list of persons
	return persons

#############################################################################################
# Track objects
#############################################################################################
# persons data structure: list
# persons[idx]['class_label']           = 'person'
# persons[idx]['temp_id']               = idx
# persons[idx]['image_coords']          = [x1, y1, x2, y2] (of the latest frame)
# persons[idx]['rel_2D_coords']         = [rel_x_dist_3d, rel_y_dist_3d] 
# persons[idx]['world_2D_coords']       = [world_x, world_y]
# persons[idx]['projected_points'][idx] = [x, y]
# persons[idx]['dist_now']              = distance_in_meters
#
# tubes data structure: dictionary
# tubes[tube_id]['last_active_frame_idx'] = frame_idx of last active frame
# tubes[tube_id]['frame_time'][idx]       = frame_time
# tubes[tube_id]['world_2D_coords'][idx]  = [world_x, world_y]
# tubes[tube_id]['rel_2D_coords']         = [rel_x_dist_3d, rel_y_dist_3d] (of the latest frame)
# tubes[tube_id]['dist_now']              = distance_in_meters  (of the latest frame)
# tubes[tube_id]['image_coords']          = [x1, y1, x2, y2] (of the latest frame)
#############################################################################################
def track_people(persons, frame_idx, robot_loc, frame_time):
	# refer to global objects
	global args, tubes, next_tube_id

	# convert frame_time to float
	frame_time = frame_time.to_sec()

	# prune any persons for whom we weren't able to compute their location in the map frame
	for i in persons.keys():
		if (persons[i]['world_2D_coords'][0] == None or persons[i]['world_2D_coords'][1] == None):
			del persons[i]
		
	# keep track of the translation table in the function below
	# det_translation[detection_idx] = detection_id
	# tube_translation[tube_idx] = tube_id
	det_translation = {}
	for detection_idx, detection_id in enumerate(persons.keys()):
		det_translation[detection_idx] = detection_id
	tube_translation = {}
	for tube_idx, tube_id in enumerate(tubes.keys()):
		tube_translation[tube_idx] = tube_id

	# figure out the cost of each detection to each tube_id
	# cost[detection_idx][tube_idx] = this_dist
	cost = np.zeros((len(persons.keys()), len(tubes.keys())))
	for detection_idx, detection_id in enumerate(persons.keys()):
		for tube_idx, tube_id in enumerate(tubes.keys()):
			x_dist = persons[detection_id]['world_2D_coords'][0] - tubes[tube_id]['world_2D_coords'][-1][0]
			y_dist = persons[detection_id]['world_2D_coords'][1] - tubes[tube_id]['world_2D_coords'][-1][1]
		 	this_dist  = math.sqrt(x_dist**2 + y_dist**2)
			cost[detection_idx][tube_idx] = this_dist

	# perform the hungarian matching algorithm (minimizing cost)
	row_ind, col_ind = linear_sum_assignment(cost)

	# find mappings from the detection_id to the tube_id
	# mappings[detection_id] = tube_id
	mappings = {}
	for detection_idx, detection_id in enumerate(persons.keys()):
		mappings[detection_id] = None
	for i in range(len(col_ind)):
		detection_idx = row_ind[i]
		tube_idx      = col_ind[i]
		tube_id       = tube_translation[tube_idx]
		detection_id  = det_translation[detection_idx]
		mappings[detection_id] = tube_id

	# loop over the person detections.  With each one, do either of these: 
	#  (a) fuse it with a previous tube (if it meets the merging criteria)
	#  (b) start a new tube (if it does not meet the merging criteria) 
	for detection_id, tube_id in mappings.iteritems():
		# get the person's location in the map frame
		person_loc = persons[detection_id]['world_2D_coords']
			
		# compute current distance to person
		person_loc_now  = person_loc
		dist_now        = dist(robot_loc, person_loc_now)		

		# compute the rate at which the person is moving
		walk_rate = -1000.0

		# if the tube_id exists, compute the walk_rate
		if not tube_id == None:
			# compute distance to person at (current_time - tracker_num_seconds)
			frame_time_past = tubes[tube_id]['frame_time'][0]
			person_loc_past = tubes[tube_id]['world_2D_coords'][0]

			# compute how far the person has traveled
			dist_traveled   = dist(person_loc_now, person_loc_past)
			time_elapsed    = frame_time - frame_time_past

			if time_elapsed > 0.0:
				walk_rate = dist_traveled / time_elapsed
				# set polarity on the walk_rate depending on whether the person is closer to us now than in the past
				dist_past = dist(robot_loc, person_loc_past)
				if dist_now > dist_past:
					walk_rate *= -1

		# if we meet the merging threshold (i.e. the object moved slower than the thresh specified), 
		# fuse the detection with the tube
		if abs(walk_rate) < args['same_object_thresh']:
			tubes[tube_id]['last_active_frame_idx'] = frame_idx
			tubes[tube_id]['frame_time'].append(frame_time)
			tubes[tube_id]['world_2D_coords'].append(person_loc)
			tubes[tube_id]['rel_2D_coords'] = persons[detection_id]['rel_2D_coords']
			tubes[tube_id]['image_coords']  = persons[detection_id]['image_coords']
			tubes[tube_id]['walk_rate']     = walk_rate
			tubes[tube_id]['dist_now']      = dist_now
		else: # start a new tube
			tubes[next_tube_id] = {}
			tubes[next_tube_id]['last_active_frame_idx'] = frame_idx
			tubes[next_tube_id]['frame_time'] = []
			tubes[next_tube_id]['frame_time'].append(frame_time)
			tubes[next_tube_id]['world_2D_coords'] = []
			tubes[next_tube_id]['world_2D_coords'].append(person_loc)
			tubes[next_tube_id]['rel_2D_coords'] = persons[detection_id]['rel_2D_coords']
			tubes[next_tube_id]['image_coords']  = persons[detection_id]['image_coords']
			tubes[next_tube_id]['walk_rate']     = walk_rate
			tubes[next_tube_id]['dist_now']      = dist_now
			next_tube_id = next_tube_id + 1
	
	# prune any tubes that are (a) not still active AND (b) have passed the forward_projection threshold
	tube_ids_to_delete = []
	for tube_id, tube_data in tubes.iteritems():
		time_since_last_detection = frame_time - tube_data['frame_time'][-1]
		if tube_data['last_active_frame_idx'] < frame_idx and time_since_last_detection > args['forward_projection']:
			tube_ids_to_delete.append(tube_id)
	for tube_id in tube_ids_to_delete:
		del tubes[tube_id]

	# clean up any tubes that are too long
	for tube_id, tube_data in tubes.iteritems():
		while (True):
			# compute time between the first and last frame
			len_tube = tube_data['frame_time'][-1] - tube_data['frame_time'][0]
			
			# if the tube is too long, remove the oldest element; else break
			if len_tube > args['tracker_num_seconds']:
				tube_data['frame_time'].pop(0)
				tube_data['world_2D_coords'].pop(0)
			else:
				break

#############################################################################################
# stationary person detector
# Input:
# 	tubes data structure: dictionary
# 	tubes[tube_id]['tube_id']               = id (int) for this tube
# 	tubes[tube_id]['last_active_frame_idx'] = frame_idx of last active frame
# 	tubes[tube_id]['image_coords']          = [x1, y1, x2, y2] (of the latest frame)
# 	tubes[tube_id]['world_2D_coords'][idx]  = [world_x, world_y]
# 	tubes[tube_id]['frame_time'][idx]       = frame_time
# Output:
#	app_persons: dictionary
#	app_persons[tube_id] = 1
#############################################################################################
def stationary_person_detector():
	# refer to global objects
	global args, tubes

	# keep track of approachable persons
	app_persons = {}

	# loop over the person tubes
	for tube_id, tube_data in tubes.iteritems():
		# ignore tubes that just began
		len_tube = tubes[tube_id]['frame_time'][-1] - tubes[tube_id]['frame_time'][0]
		if (len_tube + 1) < args['tracker_num_seconds']: # tube that just began
			continue
		# determine whether this person is approachable
		if abs(tube_data['walk_rate']) < args['stationary_thresh']:
			app_persons[tube_id] = 1
	
	# return approachable persons
	return app_persons

######################################################################################
# get robots position in the map frame
######################################################################################
def get_robot_position():
	# get robot's location
	now = rospy.Time.now()
	tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(1.0))
	(trans, quaternion) = tf_.lookupTransform( '/map', '/base_link', now )
	rot = tf.transformations.euler_from_quaternion(quaternion)
	return [trans, rot]

#############################################################################################
# approaching person detector
# Input:
# 	tubes data structure: dictionary
# 	tubes[tube_id]['tube_id']               = id (int) for this tube
# 	tubes[tube_id]['last_active_frame_idx'] = frame_idx of last active frame
# 	tubes[tube_id]['image_coords']          = [x1, y1, x2, y2] (of the latest frame)
# 	tubes[tube_id]['world_2D_coords'][idx]  = [world_x, world_y]
# 	tubes[tube_id]['frame_time'][idx]       = frame_time
# Output:
#	app_persons: dictionary
#	app_persons[tube_id] = 1
#############################################################################################
def approaching_person_detector(robot_loc):
	# refer to global objects
	global args, tubes

	# keep track of approachable persons
	app_persons = {}

	# loop over the person tubes
	for tube_id, tube_data in tubes.iteritems():
		# ignore tubes that just began
		len_tube = tubes[tube_id]['frame_time'][-1] - tubes[tube_id]['frame_time'][0]
		if (len_tube + 1) < args['tracker_num_seconds']: # tube that just began
			continue
		# determine whether this person is approachable
		if tube_data['walk_rate'] > args['stationary_thresh']:
			app_persons[tube_id] = 1
	
	# return approachable persons
	return app_persons

######################################################################################
# find approachable people
######################################################################################
def find_approachable_people(robot_loc):
	# refer to global objects
	global args, tubes, stationary_persons, approaching_persons

	# run the stationary person detector
	stationary_persons = stationary_person_detector()

	# run the person-approaching-robot detector
	approaching_persons = approaching_person_detector(robot_loc)

######################################################################################
# get person to approach ->
#    first option is to return the closest stationary person
#    second option is to return the closest approaching person
######################################################################################
def get_person_to_approach():
	# refer to global objects
	global tubes, stationary_persons, approaching_persons

	# if there are no approachable persons, return -1
	if len(stationary_persons) == 0 and len(approaching_persons) == 0:
		return [-1, 'None', None]

	# local variables to help us find the closest approachable person
	closest_dist = 10000
	closest_person_id = -1

	# find the closest stationary person
	for tube_id in stationary_persons.keys():
		if tubes[tube_id]['dist_now'] >  args['person_flag_dist']:
			continue
		if tubes[tube_id]['dist_now'] < closest_dist:
			closest_person_id = tube_id
			closest_dist      = tubes[tube_id]['dist_now']
	if closest_person_id > -1:
		return [closest_person_id, 'stationary', tubes[closest_person_id]]

	# find the closest approaching person
	for tube_id in approaching_persons.keys():
		if tubes[tube_id]['dist_now'] >  args['person_flag_dist']:
			continue
		if tubes[tube_id]['dist_now'] < closest_dist:
			closest_person_id = tube_id
			closest_dist      = tubes[tube_id]['dist_now']
	if closest_person_id > -1:
		return [closest_person_id, 'approaching', tubes[closest_person_id]]

	# return id of closest person
	return [closest_person_id, 'none', None]

######################################################################################
# return approachable person tube 
######################################################################################
def get_approachable_person(person_id):
	# refer to global objects
	global tubes, stationary_persons, approaching_persons
	
	if person_id in tubes.keys() and (person_id in stationary_persons.keys() or person_id in approaching_persons.keys()):
		return tubes[person_id]
	else:
		return None

######################################################################################
# get a text summary of all tubes
######################################################################################
def get_tubes_summary():
	# refer to global objects
	global tubes, stationary_persons, approaching_persons
	summary = []

	for tube_id in tubes.keys():
		len_tube = tubes[tube_id]['frame_time'][-1] - tubes[tube_id]['frame_time'][0]
		if (len_tube + 1) < args['tracker_num_seconds']: # tube that just began
			status = 'just_began'
		elif tube_id in stationary_persons.keys():
			status = 'stationary'
		elif tube_id in approaching_persons.keys():
			status = 'approaching'
		else:
			status = 'walking_away'

		summary.append('person id: {0}; status: {1}; dist: {2:0.1f} m; {3:0.1f} m/s.'.format(tube_id, status, tubes[tube_id]['dist_now'], tubes[tube_id]['walk_rate']))

	return '\n'.join(summary)

######################################################################################
# reset data structures
######################################################################################
def reset():
	global stationary_persons, approaching_persons, next_tube_id, tubes
	stationary_persons  = {}
	approaching_persons = {}
	next_tube_id = 0
	tubes = {}

######################################################################################
# setup module
######################################################################################
def setup(input_args):
	# refer to global objects
	global args, net, meta

	# save global args
	args = input_args

	# load darknet
	darknet.set_gpu(args['gpu_id'])
	net = darknet.load_net(args['darknet_model_file'], args['darknet_weights_file'], 0)
	meta = darknet.load_meta(args['darknet_meta_file'])
	
