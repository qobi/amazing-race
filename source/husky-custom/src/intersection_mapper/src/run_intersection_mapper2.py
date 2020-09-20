#!/usr/bin/env python

#########################################################
# import libraries
#########################################################
import os
import tf
import sys
import json
import copy
import math
import time
import rospy
import string
import signal
import numpy as np
from datetime import datetime
from bresenham import bresenham
from intersection_mapper.msg import Routes2
from intersection_mapper.srv import *
from nav_msgs.msg import OccupancyGrid
from scipy.optimize import linear_sum_assignment
from os.path import expanduser
home = expanduser("~")
sys.path.append(home + '/robot-slang/husky-custom/src/rviz_marker_publisher/src/')
import rviz_marker_publisher
sys.path.append(home + '/robot-slang/husky-custom/src/cartographer_submap_localizer/src/')
import cartographer_submap_localizer as csl

######################################################################################
# get distance between two points
######################################################################################
def dist(point1, point2):
	x_dist = point1[0] - point2[0]
	y_dist = point1[1] - point2[1]
 	return math.sqrt(x_dist**2 + y_dist**2)

#########################################################
# get radian angles
#########################################################
def get_radian_angles(start_radians, added_radians):
	# refer to global objects
	global args
	radian_angles = []
	# compute the desired angle
	radian_angle = (start_radians + added_radians) % (2*math.pi)
	for r in args['radians']:
		difference = abs(r-radian_angle) % (2*math.pi)
		difference = difference if difference < math.pi else (2*math.pi - difference)
		if difference < args['intersection_tolerance']:
			if not r in radian_angles:
				radian_angles.append(r)
	return radian_angles

#########################################################
# determine whether there is an obstacle between these two points
# computing points between the extremity points.  
# These intermediary points are every 2*dist_from_obstacle.
# At any of these points, if there is an obstacle at dist_from_obstacle, 
# we have satisfied our constraint and can bail.
# This will prevent us from generating false positive intersections in alcoves.
#########################################################
def obstacle_between_two_points(loc1_map, loc2_map, map_data):
	# refer to global variables
	global args
	# compute the grid locations
	loc1_grid = convert_map_xy_to_occupancy_xy(loc1_map, map_data)
	loc2_grid = convert_map_xy_to_occupancy_xy(loc2_map, map_data)
	# determine x/y dist
	x_dist = loc2_map[0] - loc1_map[0]
	y_dist = loc2_map[1] - loc1_map[1]
	# determine theta
	theta = math.atan2(y_dist, x_dist)
	x_component = math.cos(theta) * args['obstacle_radius_grid']
	y_component = math.sin(theta) * args['obstacle_radius_grid']
	# compute num_inbetween_pts
	dist_between_pts = dist(loc1_grid, loc2_grid)
	num_inbetween_pts = int(math.ceil((float(dist_between_pts)-2*args['obstacle_radius_grid'])/(2*args['obstacle_radius_grid'])))
	# get inbetween_pts_grid
	inbetween_pts_grid = []
	for i in range(1, num_inbetween_pts+1):
		x = int(loc1_grid[0] + i*2*x_component)
		y = int(loc1_grid[1] + i*2*y_component)
		inbetween_pts_grid.append([x,y])
	# check if any inbetween points are near obstacles 
	for inbetween_pt_grid in inbetween_pts_grid:
		if is_near_obstacle_or_unknown(inbetween_pt_grid, args['obstacle_radius_grid'], map_data):
			return True
	# if you got here, there is just free space between the two points
	return False

#########################################################
# determine whether there is an obstacle between these two points
# These intermediary points are every 2*dist_from_obstacle.
# At any of these points, if there is an obstacle at dist_from_obstacle, 
# we have satisfied our constraint and can bail.
# This will prevent us from generating false positive intersections in alcoves.
#########################################################
def obstacle_between_two_points2(loc1_map, loc2_map, map_data):
	# refer to global variables
	global args
	# compute the grid locations
	loc1_grid = convert_map_xy_to_occupancy_xy(loc1_map, map_data)
	loc2_grid = convert_map_xy_to_occupancy_xy(loc2_map, map_data)

	# get all the cells between these two locations
	inbetween_pts_grid = list(bresenham(loc1_grid[0], loc1_grid[1], loc2_grid[0], loc2_grid[1]))

	# check if any inbetween points are obstacles 
	for inbetween_pt_grid in inbetween_pts_grid:
		idx = convert_occupancy_grid_xy_coord_to_index(inbetween_pt_grid, map_data)
		if not idx == None and (map_data['occupancy_grid'][idx] > args['obstacle_thresh']):
			return True
	# if you got here, there is just free space between the two points
	return False
	
	
#########################################################
# find all trajectories that can be used to create ~90 degree intersections
#########################################################
def find_candidate_trajectories(map_data, trajectories):
	# refer to global variables
	global args

	# create these data structures
	# candidate_trajectories[radian_angle][path_len] = [x, y, dist_to_obstacle]
	candidate_trajectories = {}
	# pairs[idx] = [path_len, start_radians, end_radians]
	pairs = []
	# open_spaces[path_len] = [radians1, radians2, ...]
	open_spaces = {}
	for path_len in args['path_lengths']:
		open_spaces[path_len] = []

	# loop over all possible trajectories
	for start_radians in args['radians']:
		for path_len in args['path_lengths']:
			if path_len in trajectories[start_radians]:
				# get radian angles that are 90 degrees apart +/- intersection_tolerance
				possible_end_radians = get_radian_angles(start_radians, math.pi/2)
				for end_radians in possible_end_radians:
					if path_len in trajectories[end_radians]:
						# make sure there is not a free trajectory between them
						all_between_radians = get_radian_angles(start_radians, math.pi/4)
						for between_radians in all_between_radians:
							if path_len in trajectories[between_radians]:
								if not start_radians in open_spaces[path_len]:
									open_spaces[path_len].append(start_radians)
								if not between_radians in open_spaces[path_len]:
									open_spaces[path_len].append(between_radians)
								continue

						# make sure there is not free space directly between them
						loc_map1 = trajectories[start_radians][path_len][0:2]
						loc_map2 = trajectories[end_radians][path_len][0:2]
						if not obstacle_between_two_points(loc_map1, loc_map2, map_data):
							if not start_radians in open_spaces[path_len]:
								open_spaces[path_len].append(start_radians)
							if not end_radians in open_spaces[path_len]:
								open_spaces[path_len].append(end_radians)
							continue
						if not obstacle_between_two_points2(loc_map1, loc_map2, map_data):
							if not start_radians in open_spaces[path_len]:
								open_spaces[path_len].append(start_radians)
							if not end_radians in open_spaces[path_len]:
								open_spaces[path_len].append(end_radians)
							continue

						# we now have a pair that are 90 degrees apart (with some obstacle between them)
						pairs.append([path_len, start_radians, end_radians])

						if not start_radians in candidate_trajectories:
							candidate_trajectories[start_radians] = {}
						if not path_len in candidate_trajectories[start_radians]:
							dist_to_obstacle = compute_dist_to_obstacle(loc_map1, map_data)
							candidate_trajectories[start_radians][path_len] = [loc_map1[0], loc_map1[1], dist_to_obstacle]

						if not end_radians in candidate_trajectories:
							candidate_trajectories[end_radians] = {}
						if not path_len in candidate_trajectories[end_radians]:
							dist_to_obstacle = compute_dist_to_obstacle(loc_map2, map_data)
							candidate_trajectories[end_radians][path_len] = [loc_map2[0], loc_map2[1], dist_to_obstacle]

	# return candidate_trajectories
	return candidate_trajectories, pairs, open_spaces

#########################################################
# find and score each elbow intersection; return the best
#########################################################
def find_elbow_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id):
	# find pairs of two-somes
	# two_somes[idx] = [radian_angle1, radian_angle2]
	two_somes = []

	# loop over pairs
	for idx, pair in enumerate(pairs):
		pl, ra1, ra2 = pair
		# if one of these angles is part of open space, skip it
		part_of_free_space = False
		for radians in open_spaces[pl]:
			if ra1 == radians or ra2 == radians:
				part_of_free_space = True
				break
		if not part_of_free_space:
			two_somes.append([pl, ra1, ra2])

	# if no two_somes, bail
	if len(two_somes) == 0:
		return None

	# find the elbow with the best score
	best_score = 0
	best_idx   = None

	# loop over pairs
	for idx, two_some in enumerate(two_somes):
		pl, ra1, ra2 = two_some
		dist1 = candidate_trajectories[ra1][pl][2]
		dist2 = candidate_trajectories[ra2][pl][2]
		score = dist_to_obstacle + dist1 + dist2
		if score > best_score:
			best_score = score
			best_idx   = idx 

	# if no four_somes, bail
	if best_idx == None:
		return None

	# get locs_map of paths
	path_loc1_map = candidate_trajectories[two_somes[best_idx][1]][two_somes[best_idx][0]][0:2]
	path_loc2_map = candidate_trajectories[two_somes[best_idx][2]][two_somes[best_idx][0]][0:2]
	
	# convert locs_map to locs_submap
	loc_submap       = csl.get_submap_point(loc_map,       cartographer_trajectory_id, cartographer_submap_id)
	path_loc1_submap = csl.get_submap_point(path_loc1_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc2_submap = csl.get_submap_point(path_loc2_map, cartographer_trajectory_id, cartographer_submap_id)

	# create an intersection object with the best scoring elbow
	intersection = {}
	intersection['anchor_loc_submap'] = copy.deepcopy(loc_submap)
	intersection['loc_submap']        = loc_submap
	intersection['type']              = 'elbow'
	intersection['cardinality']       = 2
	intersection['score']             = best_score
	intersection['paths'] = {'A': [True, two_somes[best_idx][1], path_loc1_submap], 'B': [True, two_somes[best_idx][2], path_loc2_submap]}
	return intersection

#########################################################
# find and score each three way intersection; return the best
#########################################################
def find_three_way_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id):
	# find pairs of three-somes
	# three_somes[idx] = [radian_angle1, radian_angle2, radian_angle3]
	three_somes = []

	for first_pair in pairs:
		pl1, ra1, ra2 = first_pair
		# if one of these angles is part of open space, skip it
		part_of_free_space = False
		for radians in open_spaces[pl1]:
			if ra1 == radians or ra2 == radians:
				part_of_free_space = True
				break
		if part_of_free_space:
			continue
		# loop over the pairs again
		for second_pair in pairs:
			if first_pair == second_pair:
				continue
			pl2, ra3, ra4 = second_pair
			# if one of these angles is part of open space, skip it
			part_of_free_space = False
			for radians in open_spaces[pl2]:
				if ra3 == radians or ra4 == radians:
					part_of_free_space = True
					break
			if part_of_free_space:
				continue
			# if these form a three-way, keep it
			if pl1 == pl2 and ra2 == ra3:
				three_some = [pl1, ra1, ra2, ra4]
				if not three_some in three_somes:
					three_somes.append(three_some)

	# if no three_somes, bail
	if len(three_somes) == 0:
		return None

	# find the threesome with the best score
	best_score = 0
	best_idx   = None

	# loop over pairs
	for idx, three_some in enumerate(three_somes):
		pl, ra1, ra2, ra3 = three_some
		dist1 = candidate_trajectories[ra1][pl][2]
		dist2 = candidate_trajectories[ra2][pl][2]
		dist3 = candidate_trajectories[ra3][pl][2]
		score = dist_to_obstacle + dist1 + dist2 + dist3
		if score > best_score:
			best_score = score
			best_idx   = idx 

	# get locs_map of paths
	path_loc1_map = candidate_trajectories[three_somes[best_idx][1]][three_somes[best_idx][0]][0:2]
	path_loc2_map = candidate_trajectories[three_somes[best_idx][2]][three_somes[best_idx][0]][0:2]
	path_loc3_map = candidate_trajectories[three_somes[best_idx][3]][three_somes[best_idx][0]][0:2]
	
	# convert locs_map to locs_submap
	loc_submap       = csl.get_submap_point(loc_map,       cartographer_trajectory_id, cartographer_submap_id)
	path_loc1_submap = csl.get_submap_point(path_loc1_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc2_submap = csl.get_submap_point(path_loc2_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc3_submap = csl.get_submap_point(path_loc3_map, cartographer_trajectory_id, cartographer_submap_id)

	# create an intersection object with the best scoring elbow
	intersection = {}
	intersection['anchor_loc_submap'] = copy.deepcopy(loc_submap)
	intersection['loc_submap']        = loc_submap
	intersection['type']              = 'three-way'
	intersection['cardinality']       = 3
	intersection['score']             = best_score
	intersection['paths'] = {'A': [True, three_somes[best_idx][1], path_loc1_submap], 'B': [True, three_somes[best_idx][2], path_loc2_submap], 'C': [True, three_somes[best_idx][3], path_loc3_submap]}
	return intersection

#########################################################
# find and score each four-way intersection; return the best
#########################################################
def find_four_way_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id):
	# find pairs of four-somes
	# three_somes[idx] = [radian_angle1, radian_angle2, radian_angle3, radian_angle4]
	four_somes = []

	for first_pair in pairs:
		pl1, ra1, ra2 = first_pair
		# if one of these angles is part of open space, skip it
		part_of_free_space = False
		for radians in open_spaces[pl1]:
			if ra1 == radians or ra2 == radians:
				part_of_free_space = True
				break
		if part_of_free_space:
			continue
		# loop over the pairs again
		for second_pair in pairs:
			pl2, ra3, ra4 = second_pair
			if first_pair == second_pair or not ra2 == ra3 or not pl1 == pl2:
				continue
			# if one of these angles is part of open space, skip it
			part_of_free_space = False
			for radians in open_spaces[pl2]:
				if ra3 == radians or ra4 == radians:
					part_of_free_space = True
					break
			if part_of_free_space:
				continue
			for third_pair in pairs:
				pl3, ra5, ra6 = third_pair
				# if one of these angles is part of open space, skip it
				part_of_free_space = False
				for radians in open_spaces[pl3]:
					if ra5 == radians or ra6 == radians:
						part_of_free_space = True
						break
				if part_of_free_space:
					continue
				if first_pair == third_pair or second_pair == third_pair or not pl2 == pl3:
					continue
				if ra4 == ra5:
					four_some = [pl1, ra1, ra2, ra4, ra6]
					if not four_some in four_somes:
						four_somes.append(four_some)

	# if no four_somes, bail
	if len(four_somes) == 0:
		return None

	# find the four_some with the best score
	best_score = 0
	best_idx   = None

	# loop over pairs
	for idx, four_some in enumerate(four_somes):
		pl, ra1, ra2, ra3, ra4 = four_some
		dist1 = candidate_trajectories[ra1][pl][2]
		dist2 = candidate_trajectories[ra2][pl][2]
		dist3 = candidate_trajectories[ra3][pl][2]
		dist4 = candidate_trajectories[ra4][pl][2]
		score = dist_to_obstacle + dist1 + dist2 + dist3 + dist4
		if score > best_score:
			best_score = score
			best_idx   = idx 

	# get locs_map of paths
	path_loc1_map = candidate_trajectories[four_somes[best_idx][1]][four_somes[best_idx][0]][0:2]
	path_loc2_map = candidate_trajectories[four_somes[best_idx][2]][four_somes[best_idx][0]][0:2]
	path_loc3_map = candidate_trajectories[four_somes[best_idx][3]][four_somes[best_idx][0]][0:2]
	path_loc4_map = candidate_trajectories[four_somes[best_idx][4]][four_somes[best_idx][0]][0:2]
	
	# convert locs_map to locs_submap
	loc_submap       = csl.get_submap_point(loc_map,       cartographer_trajectory_id, cartographer_submap_id)
	path_loc1_submap = csl.get_submap_point(path_loc1_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc2_submap = csl.get_submap_point(path_loc2_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc3_submap = csl.get_submap_point(path_loc3_map, cartographer_trajectory_id, cartographer_submap_id)
	path_loc4_submap = csl.get_submap_point(path_loc4_map, cartographer_trajectory_id, cartographer_submap_id)

	# create an intersection object with the best scoring elbow
	intersection = {}
	intersection['anchor_loc_submap'] = copy.deepcopy(loc_submap)
	intersection['loc_submap']        = loc_submap
	intersection['type']              = 'four-way'
	intersection['cardinality']       = 4
	intersection['score']             = best_score
	intersection['paths'] = {'A': [True, four_somes[best_idx][1], path_loc1_submap], 'B': [True, four_somes[best_idx][2], path_loc2_submap], 'C': [True, four_somes[best_idx][3], path_loc3_submap], 'D': [True, four_somes[best_idx][4], path_loc4_submap]}
	return intersection

#########################################################
# compute intersections
# returns None, intersection_id, or a intersection_dict
#########################################################
def compute_intersection(map_data, loc_map, trajectories, open_spaces, refine=False, cartographer_trajectory_id=None, cartographer_submap_id=None):
	# refer to global objects
	global args, intersections

	# check if this location already has an intersection
	if not refine:
		for intersection_id in intersections.keys():
			#int_loc_map = csl.convert_submap_point_to_global_point(intersections[intersection_id]['anchor_loc_submap'])	
			int_loc_map = csl.convert_submap_point_to_global_point(intersections[intersection_id]['loc_submap'])	
			if dist(int_loc_map, loc_map) < args['min_dist_between_intersections']:
				return intersection_id

	# compute the distance from our loc_map to the nearest obstacle
	dist_to_obstacle = compute_dist_to_obstacle(loc_map, map_data)

	# find all trajectories that are ~90 degrees apart (and their distance to obstacles)
	candidate_trajectories, pairs, open_spaces2 = find_candidate_trajectories(map_data, trajectories)

	# merge open_spaces
	for path_len in args['path_lengths']:
		for radians in open_spaces2[path_len]:
			if not radians in open_spaces[path_len]:
				open_spaces[path_len].append(radians)

	# if you find a four-way intersection, return it
	four_way_intersection = find_four_way_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id)
	if not four_way_intersection == None:
		return four_way_intersection
	
	# if you find a three-way intersection, return it
	three_way_intersection = find_three_way_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id)
	if not three_way_intersection == None:
		return three_way_intersection

	# if you find an elbow intersection, return it
	elbow_intersection = find_elbow_intersections(map_data, trajectories, candidate_trajectories, pairs, open_spaces, loc_map, dist_to_obstacle, cartographer_trajectory_id, cartographer_submap_id)
	if not elbow_intersection == None:
		return elbow_intersection

	# return that you found no intersections
	return None

#########################################################
# display intersections in rviz
#########################################################
def display_intersections():
	global intersections, max_int_marker_id
	#print 'num intersections:', len(intersections)
	# keep track of rviz markers
	my_markers = []

	# delete existing markers
	marker_id = 8000
	for i in range(marker_id, max_int_marker_id):
		delete_marker = rviz_marker_publisher.create_delete_marker(i)
		my_markers.append(delete_marker)

	# define colors
	yellow = [1.0, 1.0, 0.0]
	blue   = [0.1, 0.1, 1.0]
	gray   = [0.75, 0.75, 0.75]
	
	# loop over our intersections.  For each one, display the routes from it
	for intersection_id in intersections.keys():
		intersection = intersections[intersection_id]

		# center of intersection
		loc_map = csl.convert_submap_point_to_global_point(intersection['loc_submap'])
		#if intersection_id == 1:
		#	print '{0: 0.5f}   {1: 0.5f}   {2:2d}   {3:2d}   {4: 0.5f}   {5: 0.5f}   {6: 0.5f}'.format(loc_map[0], loc_map[1], intersection['loc_submap'][0], intersection['loc_submap'][1], intersection['loc_submap'][2].z, intersection['loc_submap'][3], intersection['loc_submap'][4])
		this_marker = {'id': marker_id, 'x': loc_map[0], 'y': loc_map[1], 'name': 'int'+str(intersection_id), 'color': blue, 'scale': 0.4}
		text_marker = rviz_marker_publisher.create_text_marker(this_marker)
		my_markers.append(text_marker)
		marker_id += 1
		this_marker = {'id': marker_id, 'x': loc_map[0], 'y': loc_map[1], 'scale': [0.5, 0.5, 0.2], 'color': [0.4, 0.8, 1.0]}
		sphere_marker = rviz_marker_publisher.create_sphere_marker(this_marker)	
		my_markers.append(sphere_marker)
		marker_id += 1

		# outward paths from intersection
		for path_label in intersection['paths'].keys():
			path_active, path_radians, path_loc_submap = intersection['paths'][path_label]
			path_color = yellow
			text_color = blue
			if not path_active:
				path_color = gray
			path_loc_map = csl.convert_submap_point_to_global_point(path_loc_submap)
			path_name = 'int'+str(intersection_id)+'-path'+path_label
			this_marker = {'id': marker_id, 'x': path_loc_map[0], 'y': path_loc_map[1], 'name': path_name, 'color': text_color, 'scale': 0.4}
			text_marker = rviz_marker_publisher.create_text_marker(this_marker)
			my_markers.append(text_marker)
			marker_id += 1
			this_marker = {'id': marker_id, 'points': [loc_map, path_loc_map], 'color': path_color, 'scale': [0.2, 0.3, 0.3]}
			arrow_marker = rviz_marker_publisher.create_arrow_marker_with_points(this_marker)
			my_markers.append(arrow_marker)
			marker_id += 1

	max_int_marker_id = marker_id
	rviz_marker_publisher.display_markers(rviz_publisher, my_markers)

#########################################################
# display trajectories points in rviz
#########################################################
def display_trajectories(trajectories, open_spaces):
	global max_traj_marker_id

	# keep track of rviz markers
	my_markers = []

	# delete existing markers
	marker_id = 7010
	for i in range(marker_id, max_traj_marker_id):
		delete_marker = rviz_marker_publisher.create_delete_marker(i)
		my_markers.append(delete_marker)
	
	# center point
	marker = {'id': marker_id, 'x': trajectories['center'][0], 'y': trajectories['center'][1], 'scale': [0.4, 0.4, 0.4], 'color': [1.0, 0.05, 0.5]}
	sphere_marker = rviz_marker_publisher.create_sphere_marker(marker)	
	my_markers.append(sphere_marker)
	marker_id += 1
	
	purple     = [0.9, 0.25, 1.0]
	light_blue = [0.25, 0.9, 0.9]
	
	# loop over our trajectories.  For each free spot, add a sphere marker
	# trajectories[radians][ring_dist] = [x, y, free]
	for radians in args['radians']:
		for ring_dist in args['ring_distances']:
			if not ring_dist in trajectories[radians]:
				continue
			x, y, free = trajectories[radians][ring_dist]
			#print radians, ring_dist, x, y, free
			if free:
				color = purple
				if ring_dist in open_spaces and radians in open_spaces[ring_dist]:
					color = light_blue
				marker = {'id': marker_id, 'x': x, 'y': y, 'scale': [0.2, 0.2, 0.2], 'color': color}
				sphere_marker = rviz_marker_publisher.create_sphere_marker(marker)	
				my_markers.append(sphere_marker)
				marker_id += 1
			if not free:
				break

	max_traj_marker_id = marker_id

	# display all the sphere markers
	rviz_marker_publisher.display_markers(rviz_publisher, my_markers)

#########################################################
# display trajectories points in rviz
#########################################################
def display_debug_trajectories(trajectories):
	global max_debug_marker_id

	# keep track of rviz markers
	my_markers = []

	# delete existing markers
	marker_id = 9060
	for i in range(marker_id, max_debug_marker_id):
		delete_marker = rviz_marker_publisher.create_delete_marker(i)
		my_markers.append(delete_marker)

	# center point
	marker = {'id': marker_id, 'x': trajectories['center'][0], 'y': trajectories['center'][1], 'scale': [0.4, 0.4, 0.4], 'color': [0.2, 0.25, 1.0]}
	sphere_marker = rviz_marker_publisher.create_sphere_marker(marker)	
	my_markers.append(sphere_marker)
	marker_id += 1
	
	# loop over our trajectories.  For each free spot, add a sphere marker
	# trajectories[radians][ring_dist] = [x, y, free]
	for radians in args['radians']:
		for ring_dist in args['ring_distances']:
			if not ring_dist in trajectories[radians]:
				continue
			x, y, free = trajectories[radians][ring_dist]
			#print radians, ring_dist, x, y, free
			if free:
				marker = {'id': marker_id, 'x': x, 'y': y, 'scale': [0.2, 0.2, 0.2], 'color': [0.2, 0.75, 1.0]}
				sphere_marker = rviz_marker_publisher.create_sphere_marker(marker)	
				my_markers.append(sphere_marker)
				marker_id += 1
			if not free:
				break

	max_debug_marker_id = marker_id

	# display all the sphere markers
	rviz_marker_publisher.display_markers(rviz_publisher, my_markers)


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

######################################################################################
# compute whether point is near an obstacle
######################################################################################
def is_near_obstacle_or_unknown(loc, radius, map_data):
	for x in range(loc[0]-radius, loc[0]+radius):
		for y in range(loc[1]-radius, loc[1]+radius):
			idx = convert_occupancy_grid_xy_coord_to_index([x,y], map_data)
			#print x,y, idx, map_data['size_of_map']
			if not idx == None and (map_data['occupancy_grid'][idx] > args['obstacle_thresh'] or map_data['occupancy_grid'][idx] == -1):
				return True
	return False

######################################################################################
# determine whether a point is an obstacle
######################################################################################
def is_obstacle(loc, map_data):
	idx = convert_occupancy_grid_xy_coord_to_index(loc, map_data)
	if map_data['occupancy_grid'][idx] > args['obstacle_thresh']:
		return True
	return False

'''
######################################################################################
# determine whether a point is unknown
######################################################################################
def is_unknown(loc, map_data):
	idx = convert_occupancy_grid_xy_coord_to_index(loc, map_data)
	if map_data['occupancy_grid'][idx] == -1:
		return True
	return False

######################################################################################
# determine whether a point is an obstacle or unknown
######################################################################################
def is_obstacle_or_unknown(loc, map_data):
	idx = convert_occupancy_grid_xy_coord_to_index(loc, map_data)
	if not idx == None and (map_data['occupancy_grid'][idx] > args['obstacle_thresh'] or map_data['occupancy_grid'][idx] == -1):
		return True
	return False
'''
#########################################################
# convert an (x, y) coordinate to the occupancy grid index
#########################################################
def convert_occupancy_grid_xy_coord_to_index(xy_coord, map_data):
	if xy_coord[1] >= map_data['height']:
		return None
	if xy_coord[0] >= map_data['width']:
		return None
	index = xy_coord[1] * map_data['width'] + xy_coord[0] # index = y * width + x
	return index

def convert_occupancy_grid_index_to_xy_coord(index, map_data):
	grid_x = index % map_data['width']
	grid_y = index / map_data['width']
	return [grid_x, grid_y]

def convert_map_xy_to_occupancy_xy(map_xy, map_data):
	grid_x = int(round(((map_xy[0] - map_data['map_position_x']) / map_data['resolution'])))
	grid_y = int(round(((map_xy[1] - map_data['map_position_y']) / map_data['resolution'])))
	return [grid_x, grid_y]

def convert_occupancy_xy_to_map_xy(grid_xy, map_data):
	map_x = grid_xy[0] * map_data['resolution'] + map_data['map_position_x']
	map_y = grid_xy[1] * map_data['resolution'] + map_data['map_position_y']
	return [map_x, map_y]

#########################################################
# get_trajectories that are free
#########################################################
def get_trajectories(map_data, start_loc_map, need_center_free=True):
	# refer to global variables
	global args
	
	# get our trajectories
	# trajectories['center'] = start_loc_map
	# trajectories[radians][ring_dist] = [x, y, free]
	trajectories = {}
	trajectories['center'] = start_loc_map

	# get start_loc in the occupancy grid
	start_loc_grid = convert_map_xy_to_occupancy_xy(start_loc_map, map_data)

	# determine whether the center point is free 
	center_free = not is_near_obstacle_or_unknown(start_loc_grid, args['obstacle_radius_grid'], map_data)
	for radians in args['radians']:
		trajectories[radians] = {}
		if need_center_free and not center_free:
			continue
		for ring_dist in args['ring_distances']:
			x_map = start_loc_map[0] + ring_dist * math.cos(radians)
			y_map = start_loc_map[1] + ring_dist * math.sin(radians)
			loc_map = (x_map, y_map)
			loc_grid = convert_map_xy_to_occupancy_xy(loc_map, map_data)
			free = not is_near_obstacle_or_unknown(loc_grid, args['obstacle_radius_grid'], map_data)
			if not free:
				break
			trajectories[radians][ring_dist] = [x_map, y_map, free]

	return trajectories

#########################################################
# filter trajectories that have an opening wider than the hallway
#########################################################
def filter_trajectories_with_wide_opening(trajectories):
	# refer to global variables
	global args

	# get our trajectories
	# trajectories[radians][ring_dist] = [x, y, free]
	for opening_idx, radians1 in enumerate(args['radians']):
		#print 'opening_idx:', opening_idx
		for ring_dist in args['ring_distances']:
			#print '\tring_dist:', ring_dist
			# skip over this tracjectory if it doesn't have this ring_dist
			if not ring_dist in trajectories[radians1]:
				continue

			# skip over any rings whose diameter is narrower than the hallway  
			if ring_dist*2 < args['hallway_width']:
				continue

			# create a list of indeces we need to check (to go around the circle)
			idxs = range(opening_idx+1, len(args['radians'])) + range(opening_idx)
			#print '\t\tidxs:', idxs

			# keep track of the indeces in our opening and whether the opening has crossed the hallway_width thresh
			idxs_in_opening = [opening_idx]
			opening_larger_than_hallway = False

			# from our opening_idx, progress around the circle
			for idx in idxs:
				# get the radians at this idx
				radians2 = args['radians'][idx]
				# if the trajectory is free
				if ring_dist in trajectories[radians2]:
					# include the idx as part of our opening
					idxs_in_opening.append(idx)
					# compute whether the opening has surpassed the hallway_width
					if not opening_larger_than_hallway:
						# compute dist 
						opening_dist = dist(trajectories[radians1][ring_dist][0:2], trajectories[radians2][ring_dist][0:2])
						# each trajectory point is args['min_passage_size']/2 away from a wall.
						# thus two points are wider than the hallway if they are larger than this: 
						# 	(args['hallway_width'] - 2*args['min_passage_size']/2)
						if opening_dist > (args['hallway_width'] - args['min_passage_size']):
							opening_larger_than_hallway = True
				# else our opening has closed
				else:
					break
			
			#print '\t\t\t opening has closed.'
			if opening_larger_than_hallway:
				#print '\t\t\t opening was larger than hallway.  Need to delete:',  idxs_in_opening
				# if the opening was larger than our hallway_width, we delete all idxs_in_opening
				for idx	in idxs_in_opening:
					# get the radians at this idx
					radians = args['radians'][idx]
					# delete this ring_dist
					del trajectories[radians][ring_dist]
					# delete any ring_dists that are larger than this
					#for other_ring_dist in trajectories[radians].keys():
					#	if other_ring_dist > ring_dist:
					#		del trajectories[radians][other_ring_dist]					
				# break out of this loop, go to the next ring_dist
				break		
	# return filtered trajectories
	return trajectories

#########################################################
# find open spaces with an opening wider than the hallway
#########################################################
def find_open_spaces(trajectories):
	# refer to global variables
	global args

	# compute open_spaces
	# open_spaces[path_len] = [radians1, radians2, ...]
	open_spaces = {}
	num_open_spaces = 0

	# get our trajectories
	# trajectories[radians][path_len] = [x, y, free]
	for opening_idx, radians1 in enumerate(args['radians']):
		#print 'opening_idx:', opening_idx
		for path_len in args['path_lengths']:
			if not path_len in open_spaces:
				open_spaces[path_len] = []
			#print '\tpath_len:', path_len
			# skip over this trajectory if it doesn't have this path_len
			if not path_len in trajectories[radians1]:
				continue

			# skip over any rings whose diameter is narrower than the hallway  
			if path_len*2 < args['hallway_width']:
				continue

			# create a list of indeces we need to check (to go around the circle)
			idxs = range(opening_idx+1, len(args['radians'])) + range(opening_idx)
			#print '\t\tidxs:', idxs

			# keep track of the indeces in our opening and whether the opening has crossed the hallway_width thresh
			idxs_in_opening = [opening_idx]
			opening_larger_than_hallway = False

			# from our opening_idx, progress around the circle
			for idx in idxs:
				# get the radians at this idx
				radians2 = args['radians'][idx]
				# if the trajectory is free
				if path_len in trajectories[radians2]:
					# include the idx as part of our opening
					idxs_in_opening.append(idx)
					# compute whether the opening has surpassed the hallway_width
					if not opening_larger_than_hallway:
						# compute dist 
						opening_dist = dist(trajectories[radians1][path_len][0:2], trajectories[radians2][path_len][0:2])
						# each trajectory point is args['min_passage_size']/2 away from a wall.
						# thus two points are wider than the hallway if they are larger than this: 
						# 	(args['hallway_width'] - 2*args['min_passage_size']/2)
						if opening_dist > (args['hallway_width'] - args['min_passage_size']):
							opening_larger_than_hallway = True
				# else our opening has closed
				else:
					break
			
			#print '\t\t\t opening has closed.'
			if opening_larger_than_hallway:
				# if the opening was larger than our hallway_width, we mark all idxs_in_opening as open_space
				for idx	in idxs_in_opening:
					# get the radians at this idx
					radians = args['radians'][idx]
					# save this path_len as part of open_space
					if not radians in open_spaces[path_len]:
						open_spaces[path_len].append(radians)
						num_open_spaces += 1
				# break out of this loop, go to the next path_len
				break		
	# return open_spaces
	return open_spaces, num_open_spaces

#########################################################
# publish_current_intersection_and_routes
#########################################################
def publish_current_intersection_and_routes(robot_cur_angle, trajectories, intersection):
	# refer to global variables
	global directions, intersections, prev_output_msg

	# create this data structure:
	# data['intersection']             = intersection_name
	# data['intersection_id']          = intersection_id
	# data['center']                   = [x_map, y_map]
	# data['new_intersection']         = True/False
	# data['paths'][path_label]        = [path_active, direction, x, y]
	# data['dirs'][dir_name][path_len] = [x_map, y_map]
	# data['entry_path_label']         = entry_path_label
	data = {'intersection': 'undefined', 'intersection_id': -1, 'entry_path_label': 'undefined', \
	        'center': [], 'new_intersection': False, 'paths': {}, 'dirs': {}}

	# determine which radian angles correspond to which direction relative to the robot
	# rad_dir_map[radian] = direction_name
	rad_dir_map = {}
	for direction_name in directions.keys():
		angle_offset, delta_angle = directions[direction_name]
		dir_center_angle = (robot_cur_angle + angle_offset) % (2*math.pi)
		start_angle, end_angle = get_rad_angles(dir_center_angle, delta_angle)
		included_radian_angles = get_all_included_radian_angles(start_angle, end_angle)
		for radian_angle in included_radian_angles:
			rad_dir_map[radian_angle] = direction_name
		# add the available directions
		route_coords = get_route_map_coords(trajectories, start_angle, end_angle)
		for ring_dist, route_coord in route_coords.iteritems():
			if not route_coord == [None, None]:
				if not direction_name in data['dirs']:
					data['dirs'][direction_name] = {}
				data['dirs'][direction_name][ring_dist] = route_coord

	# intersection can take one of three values:
	# None: not in a predefined intersection
	# int:  the id of an existing intersection
	# dict: a newly discovered intersection
	# add the intersection and paths
	if not intersection == None:
		# if this is a new intersection, resolve it to an intersection and indicate it is new
		if isinstance(intersection, int): 
			intersection = intersections[intersection]
			data['new_intersection'] = True
		# get the center location of the intersection
		anchor_loc_map = csl.convert_submap_point_to_global_point(intersection['anchor_loc_submap'])
		data['center'] = anchor_loc_map 
		# add the intersection type
		data['intersection']    = intersection['type']
		data['intersection_id'] = intersection['id']
		for path_label, path in intersection['paths'].iteritems():
			path_active, radian_angle, path_loc_submap = path
			#print 'rad_dir_map:', rad_dir_map
			#print 'path:', path
			direction_name = rad_dir_map[radian_angle]
			path_loc_map = csl.convert_submap_point_to_global_point(path_loc_submap)
			data['paths'][path_label] = [path_active, direction_name, path_loc_map[0], path_loc_map[1]]
		# add the path that the robot entered the intersection from
		entry_path_label = get_last_int_path(intersection['id'], intersection['id'])
		data['entry_path_label'] = entry_path_label

	# summarize what is available to the robot
	output_msg = ''
	if data['intersection'] == 'undefined':
		output_msg = '---> I am not at a predefined intersection.\n'
	else:
		output_msg = '---> I am at this intersection type: ' + data['intersection'] + '.\n'
		for path_label, path in data['paths'].iteritems():
			path_active, direction_name, x, y = path
			output_msg += '     path ' + path_label + '(active=' + str(path_active) + ') is ' + direction_name + '.\n'
	output_msg += '---> I have these directions to go:\n'
	for direction_name in sorted(data['dirs'].keys()):
		output_msg += '     {0:13} at {1} meters\n'.format(direction_name, '/'.join([str(i) for i in sorted(list(data['dirs'][direction_name].keys()))]))
	if not output_msg == prev_output_msg:
		log_message(output_msg)
	prev_output_msg = output_msg

	# send message
	msg = Routes2()
	msg.data = json.dumps(data)
	msg.header.stamp = rospy.Time.now() 
	routes_pub.publish(msg)

#########################################################
# interpret the map
#########################################################
def interpret_map(data):
	# refer to global objects
	global args, intersections, next_intersection_id, global_map_data, latch_map_data

	start = time.time()

	# extract some map data
	map_data = {}
	map_data['resolution']	   = data.info.resolution
	map_data['map_position_x'] = data.info.origin.position.x
	map_data['map_position_y'] = data.info.origin.position.y
	map_data['width']		   = data.info.width
	map_data['height']		   = data.info.height
	map_data['size_of_map']	   = data.info.width * data.info.height
	map_data['occupancy_grid'] = data.data # -1=unknown, 0=free_space, 100=obstacle
	
	# save global map data
	if latch_map_data == 0:
		latch_map_data = 'interpret_map'
		global_map_data = copy.deepcopy(map_data)
		latch_map_data = 0
	
	# compute a few values needed for determining free space
	args['obstacle_radius_grid']      = int(round((args['min_passage_size'] / 2.0) / map_data['resolution']))
	args['max_dist_to_obstacle_grid'] = args['max_dist_to_obstacle'] / map_data['resolution']

	# get robot's location in the map frame
	try:
		robot_loc_map, rot = get_robot_position()
	except:
		print 'Could not get robot position.  Skipping map interpretation'
		return

	#print 'robot_loc_map, rot:', robot_loc_map, rot

	# get trajectories
	trajectories = get_trajectories(map_data, robot_loc_map)
	#trajectories = filter_trajectories_with_wide_opening(trajectories)
	open_spaces, num_open_spaces = find_open_spaces(trajectories)
	display_trajectories(trajectories, open_spaces)

	# compute intersection
	intersection = None
	if num_open_spaces == 0:
		intersection = compute_intersection(map_data, robot_loc_map, trajectories, open_spaces)
	
	# save intersection	
	if isinstance(intersection, dict):
		intersection['id'] = next_intersection_id
		intersections[next_intersection_id] = intersection
		next_intersection_id += 1

	# publish data about available directions
	publish_current_intersection_and_routes(rot[2], trajectories, intersection)

	end = time.time()
	print "interpret map time:", end - start

#########################################################
# get points around a circle 
#########################################################
def get_circle_points(center_point, radius, num_points, convert_to_int):
	# keep track of results
	# circle_points[idx] = [x, y]
	circle_points = []

	# compute slice_size
	slice_size = 2 * math.pi / num_points
	
	# add all the points in the circle
	for i in range(num_points):
		angle = slice_size * i;
		x = center_point[0] + radius * math.cos(angle)
		y = center_point[1] + radius * math.sin(angle)
		
		if convert_to_int:
			x = int(round(x))
			y = int(round(y))
		
		point = [x, y]		
		circle_points.append(point)

	# return all circle points
	return circle_points

#########################################################
# get neighbors in a ring around a cell 
#########################################################
def get_ring_cells(cell, radius, num_points):
	# keep track of all neighbors
	neighbors = []

  	# get the points around the circle
	circle_points = get_circle_points(cell, radius, num_points, True)

 	# loop over map points
	for point in circle_points:
		neighbors.append(point)

	# return neighors in ring
	return neighbors

#########################################################
# compute the distance to the nearest obstacle
#########################################################
def compute_dist_to_obstacle(refinement_loc_map, map_data):
	# refer to global variables
	global args
	refinement_loc_grid = convert_map_xy_to_occupancy_xy(refinement_loc_map, map_data)
	for dist_to_obstacle in np.arange(args['obstacle_radius_grid'],args['max_dist_to_obstacle_grid'], args['grid_cell_stride']):
		num_points = int((args['max_dist_to_obstacle_grid']+1)*2*4)/args['grid_cell_stride']
		ring_cells = get_ring_cells(refinement_loc_grid, dist_to_obstacle, num_points)
		for ring_cell in ring_cells:
			if is_obstacle(ring_cell, map_data):
				return dist_to_obstacle
	return args['max_dist_to_obstacle_grid']
'''
def compute_dist_to_obstacle_or_unknown(refinement_loc_map, map_data):
	# refer to global variables
	global args
	refinement_loc_grid = convert_map_xy_to_occupancy_xy(refinement_loc_map, map_data)
	for dist_to_obstacle in np.arange(args['obstacle_radius_grid'],args['max_dist_to_obstacle_grid'], args['grid_cell_stride']):
		num_points = int((args['max_dist_to_obstacle_grid']+1)*2*4)/args['grid_cell_stride']
		ring_cells = get_ring_cells(refinement_loc_grid, dist_to_obstacle, num_points)
		for ring_cell in ring_cells:
			if is_obstacle_or_unknown(ring_cell, map_data):
				return dist_to_obstacle
	return args['max_dist_to_obstacle_grid']
'''

#########################################################
# compute the distance between two radian angles
#########################################################
def radian_dist(radian_angle1, radians_angle2):
	diff = abs(radian_angle1 - radians_angle2)
	diff = (diff + math.pi) % (2*math.pi) - math.pi
	return abs(diff)

#########################################################
# udpate our intersection graph (when we merge two paths)
#########################################################
def update_int_graph(intersection_id, keep_path_label, delete_path_label):
	global int_graph
	print 'before:', int_graph
	for connection in int_graph.keys():
		edge_active, edge_dist = int_graph[connection]
		if connection[0] == intersection_id and connection[1] == delete_path_label:
			new_connection = (intersection_id, keep_path_label, connection[2], connection[3])
			int_graph[new_connection] = [edge_active, edge_dist]
			del int_graph[connection]
		elif connection[2] == intersection_id and connection[3] == delete_path_label:
			new_connection = (connection[0], connection[1], intersection_id, keep_path_label)
			int_graph[new_connection] = [edge_active, edge_dist]
			del int_graph[connection]
	print 'after:', int_graph

#########################################################
# for a given radians, find the one that is closest to a 
# value contained in args['radians']
#########################################################
def get_closest_path_radians(path_radians):
	closest_radians = None
	closest_radians_diff = 1000000
	for radians in args['radians']:
		difference = abs(path_radians-radians) % (2*math.pi)
		difference = difference if difference < math.pi else (2*math.pi - difference)
		if difference < closest_radians_diff:
			closest_radians_diff = difference
			closest_radians = radians
	return closest_radians

def print_int(int_to_print):
	print '\tint ' + int_to_print['type'] + ' (cardinality: ' + str(int_to_print['cardinality']) + ')'
	trajectory_id, submap_index, orientation, rel_x, rel_y = int_to_print['loc_submap']
	x = orientation.x
	y = orientation.y
	z = orientation.z
	w = orientation.w
	print '\t{0} {1} {2:0.2f} {3:0.2f} {4:0.2f} {5:0.2f} {6:0.2f} {7:0.2f}'.format(trajectory_id, submap_index, x, y, z, w, rel_x, rel_y)
	for path_label, path in int_to_print['paths'].iteritems():
		path_active, path_radians, path_loc_submap = path
		trajectory_id, submap_index, orientation, rel_x, rel_y = path_loc_submap
		x = orientation.x
		y = orientation.y
		z = orientation.z
		w = orientation.w
		print '\t\t{0} {1} {2} {3} {4:0.2f} {5:0.2f} {6:0.2f} {7:0.2f} {8:0.2f} {9:0.2f} '.format(path_label, path_active, trajectory_id, submap_index, x, y, z, w, rel_x, rel_y)
		
#########################################################
# map the paths between an old and new intersection
#########################################################
def map_paths_between_old_new(intersection_id, old_int, new_int):
	# define angle threshold for paths to be considered different
	angle_thresh = math.pi/6 

	#print 'old_int:'
	#print_int(old_int)
	#print 'new_int:'
	#print_int(new_int)
	# old_int_paths[idx] = [x, y, path_label, path_radians]
	old_int_paths = []
	for path_label in old_int['paths'].keys():
		path_active, path_radians, path_loc_submap = old_int['paths'][path_label]
		path_loc_map = csl.convert_submap_point_to_global_point(path_loc_submap)
		old_int_paths.append([path_loc_map[0], path_loc_map[1], path_label, path_radians])

	# new_int_paths[idx] = [x, y, path_label, path_radians]
	new_int_paths = []
	for path_label in new_int['paths'].keys():
		path_active, path_radians, path_loc_submap = new_int['paths'][path_label]
		path_loc_map = csl.convert_submap_point_to_global_point(path_loc_submap)
		new_int_paths.append([path_loc_map[0], path_loc_map[1], path_label, path_radians])

	# figure out the cost of each old_int_path to each new_int_path
	cost = np.zeros((len(old_int_paths), len(new_int_paths)))
	for idx1, data in enumerate(old_int_paths):
		#loc1_map     = data[0:2]
		loc1_radians = data[3]
		for idx2, data in enumerate(new_int_paths):
			#loc2_map = data[0:2]
			loc2_radians = data[3]
			#this_dist = dist(loc1_map, loc2_map)
			this_dist = radian_dist(loc1_radians, loc2_radians)
			cost[idx1][idx2] = this_dist

	# perform the hungarian matching algorithm (minimizing cost)
	row_ind, col_ind = linear_sum_assignment(cost)

	# copy new_int, then nuke the paths
	temp_int = copy.deepcopy(new_int)
	new_int['paths'] = {}
	
	#print 'lengths:', len(old_int_paths), len(new_int_paths)
	#print 'cost:'
	for idx1 in range(len(cost)):
		print cost[idx1]
	#print 'row_ind:', row_ind
	#print 'col_ind:', col_ind
	
	new_paths_idxs_added = []

	# map paths to paths
	for idx in range(len(col_ind)):
		old_int_path_idx = row_ind[idx]
		new_int_path_idx = col_ind[idx]
		# sanity check that paths are approximately the same
		this_cost = cost[old_int_path_idx][new_int_path_idx]
		#print 'old_int_path_idx:', old_int_path_idx, 'new_int_path_idx:', new_int_path_idx, 'cost:', this_cost
		if this_cost > angle_thresh:
			continue
		#print 'old_int_path_idx:', old_int_path_idx, '->', 'new_int_path_idx:', new_int_path_idx
		# get the old label and pair it with the new position
		new_paths_idxs_added.append(new_int_path_idx)
		old_int_path_label     = old_int_paths[old_int_path_idx][2]
		new_int_path_label     = new_int_paths[new_int_path_idx][2]
		_, new_int_path_radians, new_int_path_submap = temp_int['paths'][new_int_path_label]
		new_int['paths'][old_int_path_label] = [True, new_int_path_radians, new_int_path_submap]
	#print 'new_paths_idxs_added:', new_paths_idxs_added

	# for any paths that were in old_int but are not in new_int, keep them, but deactivate them	
	for path_label in old_int['paths'].keys():
		#print 'old path_label:', path_label
		if path_label in new_int['paths']:
			continue
		#print 'adding ' + path_label
		path_active, path_radians, path_loc_submap = old_int['paths'][path_label]
		path_loc_map = csl.convert_submap_point_to_global_point(path_loc_submap)
		center_loc_map = csl.convert_submap_point_to_global_point(new_int['loc_submap'])
		path_radians = math.atan2(path_loc_map[1]-center_loc_map[1], path_loc_map[0]-center_loc_map[0])
		# instead of using this precisely-computed path_radians, get the one that is closest to a value in args['radians']
		path_radians = get_closest_path_radians(path_radians)
		new_int['paths'][path_label] = [False, path_radians, path_loc_submap]

	# for any paths that were not in old_int but are now in new_int, add them in!
	for path_idx in range(len(new_int_paths)):
		#print 'new path_idx:', path_idx, len(new_int['paths'].keys())
		if path_idx in new_paths_idxs_added:
			continue
		new_int_path_label  = new_int_paths[path_idx][2]
		path_active, path_radians, path_loc_submap = temp_int['paths'][new_int_path_label]
		new_path_label = 'Z'
		path_labels = dict(enumerate(string.ascii_uppercase, 1))
		for idx in range(1, len(path_labels)+1):
			if not path_labels[idx] in new_int['paths'].keys():
				new_path_label = path_labels[idx]
				break
		#print 'adding ' + new_path_label
		new_int['paths'][new_path_label] = [True, path_radians, path_loc_submap]
	
	# if any paths have come within angle_thresh of each other, merge them.
	for path1_label in list(new_int['paths'].keys()):
		# skip over any paths that we deleted
		if not path1_label in new_int['paths']: continue
		for path2_label in list(new_int['paths'].keys()):
			# skip over any paths that we deleted
			if not path2_label in new_int['paths']: continue
			# skip over the same path
			if path1_label == path2_label: continue

			path1_active, path1_radians, path1_loc_submap = new_int['paths'][path1_label]
			path2_active, path2_radians, path2_loc_submap = new_int['paths'][path2_label]

			# if these two paths are now too near one another
			if radian_dist(path1_radians, path2_radians) < angle_thresh:
				# choose the label that is closest to A
				path_labels = dict(enumerate(string.ascii_uppercase, 1))
				path_labels_reverse = {v:k for k,v in path_labels.iteritems()}
				keep_path_label   = path_labels[min(path_labels_reverse[path1_label], path_labels_reverse[path2_label])]
				delete_path_label = path_labels[max(path_labels_reverse[path1_label], path_labels_reverse[path2_label])]
				print 'merge occurred:', keep_path_label, delete_path_label
				# we keep the data associated with the active path
				if path1_active:
					new_int['paths'][keep_path_label] = [True, path1_radians, path1_loc_submap]
				else:
					new_int['paths'][keep_path_label] = [True, path2_radians, path2_loc_submap]
				# we delete the label that is further from A
				del new_int['paths'][delete_path_label]
				# we now update the int_graph
				update_int_graph(intersection_id, keep_path_label, delete_path_label)
				break

	# preserve intersection id
	new_int['id'] = old_int['id']

	text = ''
	for path_label in new_int['paths']:
		text += path_label + ': ' + str(new_int['paths'][path_label][0]) + '.  '
	print 'Has these paths:', text
	
	return new_int

#########################################################
# publish our intersections and graph
#########################################################
def publish_intersections_and_graph():
	# refer to global objects
	global intersections, int_graph

	print 'intersection graph:'
	for key, value in int_graph.iteritems():
		print '\t', key, value

#########################################################
# main loop
#########################################################
def main_loop():
	# refer to global objects
	global intersections, latch_map_data, global_map_data, robot_position_list, last_int_id

	# loop forever
	while True:
		time1 = time.time()

		# get robot location
		#####################
		try:
			robot_loc_map, rot = get_robot_position()
		except:
			print 'Could not get robot position.  Skipping loop'
			rospy.sleep(1.0)
			continue

		# save off location
		submap_point = csl.get_submap_point(robot_loc_map)
		if not submap_point == None:
			robot_position_list.append(submap_point)

		# refine intersections
		#####################
		log_message('intersections: ' + ','.join([str(i) for i in intersections.keys()]))
		for intersection_id in intersections.keys():
			# convert start_loc_submap to loc_map
			anchor_loc_map = csl.convert_submap_point_to_global_point(intersections[intersection_id]['anchor_loc_submap'])
			loc_map = csl.convert_submap_point_to_global_point(intersections[intersection_id]['loc_submap'])

			# check if this intersection is "nearby".  If not, move on.
			if dist(loc_map, robot_loc_map) > args['refinement_dist']:
				continue

			cartographer_trajectory_id = intersections[intersection_id]['loc_submap'][0]
			cartographer_submap_id     = intersections[intersection_id]['loc_submap'][1]

			# compute possible refinement locations
			# refinement_locs_map = [[x,y], [x,y], ...]
			refinement_locs_map = []
			start_x = loc_map[0] - args['refinement_resolution']
			end_x   = loc_map[0] + args['refinement_resolution']+0.001
			start_y = loc_map[1] - args['refinement_resolution']
			end_y   = loc_map[1] + args['refinement_resolution']+0.001
			
			for x in np.arange(start_x, end_x, args['refinement_resolution']):
				for y in np.arange(start_y, end_y, args['refinement_resolution']):
					refinement_loc_map = [x, y]
					if dist(anchor_loc_map, refinement_loc_map) < args['anchor_drift_dist']:
						refinement_locs_map.append(refinement_loc_map)

			# compute intersection at each refinement location
			# refinement_locs_map = [[x,y,intersection], [x,y,intersection], ...]
			# keep track of the highest cardinality
			highest_cardinality = 0 
			for idx in range(len(refinement_locs_map)):
				refinement_loc_map = refinement_locs_map[idx][0:2]

				# if you have the latch to the global_map_data
				if latch_map_data == 0 and not global_map_data == None:
					latch_map_data = 'interpret'
				else:
					rospy.sleep(0.1)

				trajectories = get_trajectories(global_map_data, refinement_loc_map)
				#trajectories = filter_trajectories_with_wide_opening(trajectories)
				open_spaces, num_open_spaces = find_open_spaces(trajectories)
				intersection = compute_intersection(global_map_data, refinement_loc_map, trajectories, open_spaces, True, cartographer_trajectory_id, cartographer_submap_id)

				# give up latch
				latch_map_data = 0

				#print '\t', idx, intersection
				'''
				# debug
				if intersection_id == 3:
					display_debug_trajectories(trajectories)
					if intersection == None:
						raw_input("NONE.  Press Enter to continue...")
					else:
						raw_input(intersection['type'] + ".  Press Enter to continue...")
				'''
				refinement_locs_map[idx].append(intersection)
				if isinstance(intersection, dict):
					highest_cardinality = max(highest_cardinality, intersection['cardinality'])
			#print 'highest_cardinality:', highest_cardinality

			# prune out only the highest ordinal intersections (4-way, 3-way, elbow)
			# refinement_locs_map = [[x,y,intersection], [x,y,intersection], ...]
			for idx in range(len(refinement_locs_map)-1, -1, -1):
				if refinement_locs_map[idx][2] == None:
					del refinement_locs_map[idx]
				elif refinement_locs_map[idx][2]['cardinality'] < highest_cardinality:
					del refinement_locs_map[idx]

			# if there are no intersections at this location, deactivate this intersection
			if len(refinement_locs_map) == 0:
				for path_label in intersections[intersection_id]['paths']:
					intersections[intersection_id]['paths'][path_label][0] = False
				continue

			# keep the intersection with the best score
			best_score = 0 
			best_idx   = 0 
			for idx in range(len(refinement_locs_map)):
				#print idx, 'score:', refinement_locs_map[idx][2]['score']
				if refinement_locs_map[idx][2]['score'] > best_score:
					best_score = refinement_locs_map[idx][2]['score']
					best_idx   = idx
			best_intersection = refinement_locs_map[best_idx][2]

			# map paths to paths between the old and new intersection
			print '\nintersection_id:', intersection_id
			best_intersection = map_paths_between_old_new(intersection_id, intersections[intersection_id], best_intersection)

			# preserve the original anchor location
			best_intersection['anchor_loc_submap'] = intersections[intersection_id]['anchor_loc_submap']
			
			# replace existing intersection with this one
			intersections[intersection_id] = best_intersection
			
		'''
		# now prune any intersections that have converged to same location
		for intersection_id1 in intersections.keys():
			# in case we deleted one of the intersections in our list, continue
			if not intersection_id1 in intersections:
				continue
			loc_map1 = csl.convert_submap_point_to_global_point(intersections[intersection_id1]['loc_submap'])	
			for intersection_id2 in intersections.keys():
				if intersection_id1 == intersection_id2:
					continue
				loc_map2 = csl.convert_submap_point_to_global_point(intersections[intersection_id2]['loc_submap'])	
				dist_between_intersections = dist(loc_map1, loc_map2)
				if dist_between_intersections < args['min_dist_between_intersections']:
					intersections[intersection_id2]['active'] = False
		'''
		
		# maintain intersection graph
		#####################
		cur_int_id = at_intersection(robot_loc_map)
		if not cur_int_id == None and not cur_int_id == last_int_id:
			# determine the entry path of the current intersection
			entry_path_label = get_last_int_path(cur_int_id, cur_int_id)
			# determine the exit path of the last intersection
			exit_path_label = get_last_int_path(cur_int_id, last_int_id)
			
			# determine the distance between these two intersections
			if not entry_path_label == None and not exit_path_label == None:
				# add an entry to our int_graph if it's not already there
				# int_graph[('int1', 'A', 'int2', 'B')] = [edge_active, edge_dist]
				if not (cur_int_id, entry_path_label, last_int_id, exit_path_label) in int_graph:
					int_graph[(cur_int_id, entry_path_label, last_int_id, exit_path_label)] = [True, 1000.0]
			# update our last_int_id_at with our current_int_id 
			last_int_id = cur_int_id

		# update our int_graph distances and active
		for key in int_graph.keys():
			int1_id, path1_label, int2_id, path2_label = key
			# compute edge dist
			loc_map1 = csl.convert_submap_point_to_global_point(intersections[int1_id]['loc_submap'])
			loc_map2 = csl.convert_submap_point_to_global_point(intersections[int2_id]['loc_submap'])
			edge_dist = dist(loc_map1, loc_map2)
			# compute edge active
			path1_active = intersections[int1_id]['paths'][path1_label][0]
			path2_active = intersections[int2_id]['paths'][path2_label][0]
			edge_active = path1_active and path2_active
			# update the int_graph
			int_graph[key] = [edge_active, edge_dist]

		# resolve multiple edges from a single path
		# create this data structure:
		# 	edges[(int1_id, path1_label)] = [[key, dist], ...]
		edges = {}
		for key in int_graph.keys():
			int1_id, path1_label, int2_id, path2_label = key
			edge_active, edge_dist = int_graph[key]
			if not (int1_id, path1_label) in edges:
				edges[(int1_id, path1_label)] = []
			edges[(int1_id, path1_label)].append([key, edge_dist])
			if not (int2_id, path2_label) in edges:
				edges[(int2_id, path2_label)] = []
			edges[(int2_id, path2_label)].append([key, edge_dist])
		for int_path in edges.keys():
			edges_from_path = copy.deepcopy(edges[int_path])
			# if there are more than one edge
			if len(edges_from_path) > 1:
				# sort them from shortest dist to longest dist
				edges_from_path.sort(key = lambda x: x[1])
				# then delete all but the shortest edge
				for idx in range(len(edges_from_path)-1,0,-1):
					key, _ = edges_from_path[idx]
					del int_graph[key]
					# also remove it from edges (so we don't try to delete its pair later)
					for path_key in edges.keys():
						for idx2 in range(len(edges[path_key])-1,-1,-1):
							if edges[path_key][idx2][0] == key:
								del edges[path_key][idx2]
		
		# publish our int_graph
		publish_intersections_and_graph()

		time2 = time.time()

		# display our intersections
		display_intersections()

		# sleep for some time between intersection refinement
		print 'main loop time:', time2-time1
		rospy.sleep(1.0)

######################################################################################
# determine whether the robot is at an intersection
######################################################################################
def at_intersection(loc_map):
	# refer to global objects
	global intersections

	# check for all intersections you are at
	# (should always be one.  if by some chance, two are right next to each other, we get the shortest dist)
	at_ints = []
	for int_id in intersections:
		int_loc_map = csl.convert_submap_point_to_global_point(intersections[int_id]['loc_submap'])	
		dist_to_int = dist(loc_map, int_loc_map)
		if dist_to_int < args['in_intersection_dist']:
			at_ints.append([int_id, dist_to_int])

	# return the closest intersection
	if len(at_ints) == 0:
		return None
	else:
		at_ints.sort(key = lambda x: x[1])
		return at_ints[0][0]

# intersections[int_id]['anchor_loc_submap'] = submap
# intersections[int_id]['loc_submap']        = submp
# intersections[int_id]['type']              = 'elbow'
# intersections[int_id]['cardinality']       = 2
# intersections[int_id]['score']             = best_score
# intersections[int_id]['paths']             = {'A': [active, radian_angle, submap_loc], ...}

######################################################################################
# search along the robot path for the last intersection/path the robot was at
######################################################################################
def get_last_int_path(cur_int_id, int_id_to_consider):
	# refer to global objects
	global intersections, robot_position_list

	# corner case when starting out
	if not int_id_to_consider in intersections:
		return None

	# get the location of our current intersection
	cur_int_map =  csl.convert_submap_point_to_global_point(intersections[cur_int_id]['loc_submap'])

	for idx in range(len(robot_position_list)-1,-1,-1):
		position_submap = robot_position_list[idx]
		position_map = csl.convert_submap_point_to_global_point(position_submap)
		
		# if we are in the current int, and trying to find where we entered from, 
		# wait until we've backed off a bit from the center.  Otherwise, the distances 
		# to the path are too close/noisy to get the accurate entry point
		if cur_int_id == int_id_to_consider:
			dist_to_int = dist(cur_int_map, position_map)
			if dist_to_int < args['in_intersection_dist']:
				continue

		path_dists = []
		for path_label in intersections[int_id_to_consider]['paths'].keys():
			path_submap_loc = intersections[int_id_to_consider]['paths'][path_label][2]
			path_loc_map = csl.convert_submap_point_to_global_point(path_submap_loc)	
			dist_to_path = dist(position_map, path_loc_map)
			#print '\t', int_id_to_consider, path_label, 'dist_to_path:', dist_to_path
			if dist_to_path < args['in_path_dist']:
				path_dists.append([path_label, dist_to_path])
		if len(path_dists) > 0:
			path_dists.sort(key = lambda x: x[1])
			return path_dists[0][0]

	return None

#########################################################
# display_forward_route
#########################################################
def display_requested_points(result):
	# refer to global variables
	global max_routes_marker_id

	# delete existing markers
	all_markers = []
	marker_id = 7000
	for i in range(marker_id, max_routes_marker_id):
		delete_marker = rviz_marker_publisher.create_delete_marker(i)
		all_markers.append(delete_marker)

	# if there are routes to display, display them!
	for ring_dist in result:
		if len(result[ring_dist]) > 0:
			if result[ring_dist] == [None, None]:
				continue
			# add the routes to rviz
			x_map, y_map = result[ring_dist]
			marker = {'id': marker_id, 'x': result[ring_dist][0], 'y': result[ring_dist][1], 'color': [0.2, 0.6, 0.6], 'scale': [0.25, 0.25, 0.25]}
			sphere_marker = rviz_marker_publisher.create_sphere_marker(marker)
			all_markers.append(sphere_marker)
			this_marker = {'id': marker_id+1, 'x': result[ring_dist][0], 'y': result[ring_dist][1], 'name': 'requested-' + str(ring_dist), 'color': [0.0, 0.0, 1.0], 'scale': 0.2}
			text_marker = rviz_marker_publisher.create_text_marker(this_marker)
			all_markers.append(text_marker)
			marker_id += 2
	max_routes_marker_id = marker_id
	
	# plot the requested points in rviz
	rviz_marker_publisher.display_markers(rviz_publisher, all_markers)

#########################################################
# get radian angle relative to start_angle and delta
#########################################################
def get_rad_angles(start_radians, delta_radians):
	start_angle = (start_radians - delta_radians) % (2*math.pi)
	end_angle   = (start_radians + delta_radians) % (2*math.pi)
	return [start_angle, end_angle]


#########################################################
# here we compute all the radian angles between our start/end angles
# we run into one corner case if start_angle is larger than the end_angle 
# (like if you are getting a route between 350 and 10 degrees)
#########################################################
def get_all_included_radian_angles(start_angle, end_angle):
	global args
	included_radian_angles = []
	for radians in args['radians']:
		# corner case 
		if end_angle < start_angle:
			if radians <= end_angle:
				included_radian_angles.append(radians)
			if radians >= start_angle:
				included_radian_angles.append(radians)
		# base base
		elif radians >= start_angle and radians <= end_angle:
				included_radian_angles.append(radians)
	return included_radian_angles

#########################################################
# get route map coords
#########################################################
def get_route_map_coords(trajectories, start_angle, end_angle):
	# refer to global objects
	global args

	included_radian_angles = get_all_included_radian_angles(start_angle, end_angle)

	# now we determine which angles are free at each ring_dist
	# free_radian_angles[ring_dist] = [radian_angle1, ...]
	free_radian_angles = {}
	for ring_dist in args['ring_distances']:
		free_radian_angles[ring_dist] = []
		for included_radian_angle in included_radian_angles:
			if ring_dist in trajectories[included_radian_angle] and trajectories[included_radian_angle][ring_dist][2]:
				free_radian_angles[ring_dist].append(included_radian_angle)

	# create this data structure
	# route_coords[ring_dist] = [x_map_frame, y_map_frame] for the median among the free_radian_angles
	route_coords = {}
	for ring_dist in args['ring_distances']:
		# if there are no available paths
		if len(free_radian_angles[ring_dist]) == 0:
			route_coords[ring_dist] = [None, None]
		else:
			# special case when the route traverses the 360/0 border)
			if start_angle > end_angle:
				ang_0_to_pi   = [x for x in free_radian_angles[ring_dist] if x < math.pi]
				ang_pi_to_2pi = [x for x in free_radian_angles[ring_dist] if x >= math.pi]
				ang_0_to_pi.reverse()
				ang_pi_to_2pi.reverse()
				angles = ang_0_to_pi + ang_pi_to_2pi
			else:
				angles = free_radian_angles[ring_dist]

			# compute the median
			if len(free_radian_angles[ring_dist]) == 1:
				median = angles[0]
			else:
				median = angles[len(angles)/2]

			# compute the world and rel location of this frontier idx 
			x_map = trajectories[median][ring_dist][0]
			y_map = trajectories[median][ring_dist][1]
			route_coords[ring_dist] = [x_map, y_map]

	# return result
	return route_coords

#########################################################
# given a direction, return all points along that direction
#########################################################
def get_single_route(data):
	# refer to global objects
	global args, latch_map_data, global_map_data
	
	#start_time = time.time()

	# extract direction the node is requesting
	start_loc   = [data.x, data.y]
	start_angle = data.start_angle
	end_angle   = data.end_angle

	'''
	# debug
	if start_loc == [12345, 12345]:
		start_loc, rot = get_robot_position()
		print 'start_loc:', start_loc, 'rot:', rot
		start_angle, end_angle = get_rad_angles(rot[2], 0.262)
	'''
	# get latch for the global_map_data
	while not latch_map_data == 0:
		rospy.sleep(0.01)
	latch_map_data = 'single_route'

	# get trajectories
	# trajectories['center'] = start_loc_map
	# trajectories[radians][ring_dist] = [x, y, free]
	trajectories = get_trajectories(global_map_data, start_loc, False)

	# give up latch
	latch_map_data = 0

	# get map locations in direction
	route_coords = get_route_map_coords(trajectories, start_angle, end_angle)

	# display requested point
	if args['show_requested_routes']:
		display_requested_points(route_coords)

	# debug
	#display_debug_trajectories(trajectories)
	#raw_input("Press Enter to continue...")

	# setup message
	x_world = []
	y_world = []

	# populate message
	for ring_dist in args['ring_distances']:
		if not route_coords[ring_dist] == [None, None]:
			x_world.append(route_coords[ring_dist][0])
			y_world.append(route_coords[ring_dist][1])

	#end_time = time.time()
	#print 'get_single_route time:', end_time - start_time

	# return ros message with points 
	return [x_world, y_world]

# rosservice call /intersection_mapper/single_direction 3.5 0.1
# rosservice call /intersection_mapper/single_direction "{'x':-22.5, 'y':-29.0, 'start_angle':3.5, 'end_angle':0.1}"

#########################################################
# get a list of recovery points
#########################################################
def get_recovery_angle(data):
	# refer to global objects
	global latch_map_data, global_map_data

	# if we have no data yet, return ros message with no points
	if global_map_data == None:
		return -1

	# get latch for the global_map_data
	while not latch_map_data == 0:
		rospy.sleep(0.1)
	latch_map_data = 'recovery'

	# get robot's location in the map frame
	robot_loc_map, rot = get_robot_position()

	# get trajectories
	# trajectories['center'] = start_loc_map
	# trajectories[radians][ring_dist] = [x, y, free]
	trajectories = get_trajectories(global_map_data, robot_loc_map, False)

	# give up latch
	latch_map_data = 0

	# start at the farthest ring_dist
	free_rad_angle = None
	for ring_dist in reversed(args['ring_distances']):
		# incrementally search a wider and wider window
		for delta_angle in [0.524, 1.047, 1.571, 2.094, 2.618, 3.141]: # 30, 60, 90, 120, 150, 180 degrees
			start_angle, end_angle = get_rad_angles(rot[2], delta_angle)

			# if any of the radian angles between our start/end angles are free, break
			# we run into one corner case if start_angle is larger than the end_angle 
			# (like if you are getting a route between 350 and 10 degrees)
			for radians in args['radians']:
				# corner case 
				if end_angle < start_angle:
					if radians <= end_angle and ring_dist in trajectories[radians] and trajectories[radians][ring_dist][2]:
						free_rad_angle = radians
						break
					if radians >= start_angle and ring_dist in trajectories[radians] and trajectories[radians][ring_dist][2]:
						free_rad_angle = radians
						break
				# base base
				elif radians >= start_angle and radians <= end_angle and ring_dist in trajectories[radians] and trajectories[radians][ring_dist][2]:
					free_rad_angle = radians
					break
	
			if not free_rad_angle == None:
				break
		if not free_rad_angle == None:
			break
	
	# return ros message with points 
	if free_rad_angle == None:
		return -1.0
	else:
		return free_rad_angle
		
# rosservice call /intersection_mapper/get_recovery_angle



######################################################################################
# print/save a log message
######################################################################################
def log_message(msg):
	print msg
	args['output_log'].write(datetime.now().strftime("%Y-%b-%d-%H-%M-%S") + ' ' + msg + '\n')

#########################################################
# if calling from the command line...
##############################\###########################
if __name__ == '__main__':
	# import launch file params 
	args = {}
	args['obstacle_thresh']	               = rospy.get_param('intersection_mapper/obstacle_thresh')
	args['ring_distances']                 = rospy.get_param('intersection_mapper/ring_distances')
	args['points_on_ring']                 = rospy.get_param('intersection_mapper/points_on_ring')
	args['path_lengths']                   = rospy.get_param('intersection_mapper/path_lengths')
	args['min_passage_size']	           = rospy.get_param('intersection_mapper/min_passage_size')
	args['max_dist_to_obstacle']	       = rospy.get_param('intersection_mapper/max_dist_to_obstacle')
	args['grid_cell_stride']	           = rospy.get_param('intersection_mapper/grid_cell_stride')
	args['hallway_width']                  = rospy.get_param('intersection_mapper/hallway_width')
	args['anchor_drift_dist']              = rospy.get_param('intersection_mapper/anchor_drift_dist')
	args['refinement_resolution']          = rospy.get_param('intersection_mapper/refinement_resolution')
	args['refinement_dist']                = rospy.get_param('intersection_mapper/refinement_dist')
	args['min_dist_between_intersections'] = rospy.get_param('intersection_mapper/min_dist_between_intersections')
	args['in_intersection_dist']           = rospy.get_param('intersection_mapper/in_intersection_dist')
	args['in_path_dist']                   = rospy.get_param('intersection_mapper/in_path_dist')
	args['intersection_tolerance']         = rospy.get_param('intersection_mapper/intersection_tolerance')
	args['show_intersection_map']          = rospy.get_param('intersection_mapper/show_intersection_map')
	args['show_requested_routes']          = rospy.get_param('intersection_mapper/show_requested_routes')

	# convert path_lengths to list of floats
	if isinstance(args['path_lengths'], str):
		args['path_lengths'] = args['path_lengths'].split(',')
	else:
		args['path_lengths'] = [args['path_lengths']]
	args['path_lengths'] = [float(i) for i in args['path_lengths']]

	# convert ring_distances to list of floats
	if isinstance(args['ring_distances'], str):
		args['ring_distances'] = args['ring_distances'].split(',')
	else:
		args['ring_distances'] = [args['ring_distances']]
	args['ring_distances'] = [float(i) for i in args['ring_distances']]
	args['num_rings'] = len(args['ring_distances'])
	print 'ring_distances:', args['ring_distances']

	# compute radians
	args['radians'] = []
	slice_size = 2 * math.pi / args['points_on_ring']
	for i in range(args['points_on_ring']):
		radians = slice_size * i
		args['radians'].append(radians)

	# open output log
	dateTimeObj        = datetime.now()
	output_filename    = dateTimeObj.strftime("%Y-%b-%d-%H-%M-%S") + '-output-log.txt'
	home               = os.path.expanduser("~")
	log_path           = home + '/.ros/intersection_mapper/log/'
	if not os.path.exists(log_path):
		os.makedirs(log_path)
	output_log         = open(log_path + output_filename, 'w')
	args['output_log'] = output_log
	log_message('Starting intersection mapper')

	# initialize global variables
	max_routes_marker_id = 0
	max_traj_marker_id   = 0
	max_int_marker_id    = 0
	max_debug_marker_id  = 0
	intersections        = {}
	next_intersection_id = 1
	latch_map_data       = 0
	global_map_data      = None
	robot_position_list  = []
	last_int_id          = None
	int_graph            = {}
	prev_output_msg      = ''

	# specify directions and their constituent angles
	directions = {}
	directions['forward']       = [0.000, 0.524] #   0 deg +/- 30 deg
	directions['forward-left']  = [0.785, 0.262] #  45 deg +/- 15 deg
	directions['left']          = [1.571, 0.524] #  90 deg +/- 30 deg
	directions['back-left']     = [2.356, 0.262] # 135 deg +/- 15 deg
	directions['back']          = [3.142, 0.524] # 180 deg +/- 30 deg
	directions['back-right']    = [3.927, 0.262] # 225 deg +/- 15 deg
	directions['right']         = [4.712, 0.524] # 270 deg +/- 30 deg
	directions['forward-right'] = [5.498, 0.262] # 315 deg +/- 15 deg

	try:
		# initialize this node
		rospy.init_node('intersection_mapper')

		# setup cartographer submap localizer
		csl.setup()

		# setup rviz marker publisher
		rviz_publisher = rviz_marker_publisher.setup()

		# get the transform
		tf_ = tf.TransformListener()

		# setup publishers
		routes_pub = rospy.Publisher('/intersection_mapper/routes', Routes2, queue_size=1)

		# setup subscriptions
		rospy.Subscriber("/map", OccupancyGrid, interpret_map, queue_size=1)

		# setup services
		rospy.Service('/intersection_mapper/single_direction', singleRoute, get_single_route)
		rospy.Service('/intersection_mapper/get_recovery_angle', recoveryRoutes, get_recovery_angle)

		# run main loop
		# keep track of robot position
		# create intersection graph
		# refine intersections
		main_loop() 
	except rospy.ROSInterruptException:
		pass

