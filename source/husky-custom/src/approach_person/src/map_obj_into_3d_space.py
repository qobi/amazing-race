# import libraries
import tf
import math
import time
import rospy
import numpy as np
import cv2

#############################################################################################
# map objects into 3D space
# Input: 
#	objects[tube_id]['class_label']           = 'label'
#	objects[tube_id]['tube_id']               = tube_id
# 	objects[tube_id]['image_coords']          = [x1, y1, x2, y2] 
# Output:
#	objects[tube_id]['class_label']           = 'label'
#	objects[tube_id]['tube_id']               = tube_id
# 	objects[tube_id]['image_coords']          = [x1, y1, x2, y2] 
# 	objects[tube_id]['rel_2D_coords']         = [rel_x_dist_3d, rel_y_dist_3d] 
# 	objects[tube_id]['world_2D_coords']       = [world_x_dist_3d, world_y_dist_3d]
# 	objects[tube_id]['projected_points'][idx] = [x, y]
# 	objects[tube_id]['closest_dist']          = distance_in_meters
#############################################################################################
def map_objs(args, cv_image, objects, velodyne_data, camera_model, tf_):
	# if the args is empty, initialize it with some defaults
	if args == {}:
		args['pointcloud_max_dist']	  = 1000
		args['pointcloud_min_dist']   = -1000
		args['pointcloud_min_height'] = -1000
		args['pointcloud_max_height'] = 1000

	# check if there is at least one object
	if objects == {}:
		return objects

	# get image dims
	img_height = cv_image.shape[0]
	img_width  = cv_image.shape[1]

	# get the transform
	now = rospy.Time.now()
	tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(1.0))
	(trans_vel, rot_vel) = tf_.lookupTransform( '/map', '/base_link', now )

	#(trans, rot) = tf_.lookupTransform( '/axis', '/velodyne', now )
	#tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(4.0))
	# calibration in lab, with checkerboard (has alignment issues -- discovered on 2019.02.27)
	#trans = [-0.38171418, 0.03789976, -0.25209234]
	#rot = [0.556080723308, -0.426404593392, 0.433711606051, 0.566434104443]
	# calibration in hallway, with diagonal boxes in 3 positions (calibration done on 2019.02.28)
	#(trans, rot) = tf_.lookupTransform( 'camera', 'velodyne', rospy.Time( 0 ) )
	#trans = [-0.02532482, 0.28742033, -0.08615289]
	trans = [-0.03019076, 0.27373338, -0.05301323]
	#rot = [0.563540225425, -0.53427658701, 0.423247410108, 0.466725371859]
	rot = [0.544521193495, -0.512626565799, 0.452427950113, 0.485818509146]
	trans = tuple(trans) + ( 1,  )

	rotationMatrix = tf.transformations.quaternion_matrix( rot )
	# append translation to the last column of rotation matrix(4x4)
	rotationMatrix[ :, 3 ] = trans

	# do the same for velodyne to map frame transformation
	trans_vel = tuple(trans_vel) + ( 1,  )
	rotationMatrix_vel = tf.transformations.quaternion_matrix( rot_vel )
	# append translation to the last column of rotation matrix(4x4)
	rotationMatrix_vel[ :, 3 ] = trans_vel

	# populate new fields
	for tube_id in objects.keys():
		# rel_2D_coords = relative (x,y) from robot to closest lidar point in object BB
		objects[tube_id]['rel_2D_coords']    = [None, None]
		# world_2D_coords = world (x,y) of object
		objects[tube_id]['world_2D_coords']  = [None, None]
		# projected_points = these are the lidar points inside the object boxes
		objects[tube_id]['projected_points'] = [] 
		# distance to closest lidar point in object BB
		objects[tube_id]['closest_dist']     = 1000

	# TOM's numpy speed-up
	if velodyne_data:
		vel_points = np.array(velodyne_data)

		# filter out points that are outside specified parameters
		cond = (vel_points[:,0] >= args['pointcloud_min_dist']) & \
			   (vel_points[:,0] <= args['pointcloud_max_dist']) & \
			   (vel_points[:,2] >= args['pointcloud_min_height']) & \
			   (vel_points[:,2] <= args['pointcloud_max_height'])
		vel_points = vel_points[cond]
			   
		# append column of ones for transformation calculation (next)
		one_col = np.ones((vel_points.shape[0], 1))
		vel_points = np.hstack((vel_points[:,0:3], one_col))

		# transform 3D lidar points into 2D image frame 
		rotated_vel_points = rotationMatrix.dot(np.transpose(vel_points))
		uv_points = np.transpose(camera_model.P.dot(rotated_vel_points))
		uv_points = np.array(uv_points[:,0:2]/uv_points[:,2])

		# loop over the objects
		for tube_id in objects.keys():
			x1, y1, x2, y2 = objects[tube_id]['image_coords']

			# specify condition wherein the projected point must be within the bounding box 
			cond = (uv_points[:,0] >= x1) & (uv_points[:,0] <= x2) & (uv_points[:,1] >= y1) & (uv_points[:,1] <= y2)
			points_within_box = vel_points[cond]

			# compute the closest point
			for point_within_box in points_within_box:
				# compute dist to point
				dist_to_point = math.sqrt( np.sum( np.array( point_within_box[ :3 ] ) ** 2 ) )
				# find closest point in bounding box
				if dist_to_point < objects[tube_id]['closest_dist']:
					# transform from velodyne frame to map frame
					world_point = rotationMatrix_vel.dot(point_within_box)
					objects[tube_id]['world_2D_coords'] = world_point
					objects[tube_id]['closest_dist']    = dist_to_point
					objects[tube_id]['rel_2D_coords']   = point_within_box

			# save all points within box
			objects[tube_id]['projected_points'] = uv_points[cond].astype(int)

	# return data
	return objects

	'''
	# OLD WAY -- MUCH SLOWER (as much as 100x slower)!
	# compute the distances to each object
	if velodyne_data:
		# loop over point cloud data
		for i in range(0, len(velodyne_data) - 1):
			try:
				# converting to homogeneous coordinates
				point = [velodyne_data[i][0], velodyne_data[i][1], velodyne_data[i][2], 1]
			except IndexError:
				print("Index Error!!!!!")
				break
				
			# filter out pointcloud points that are outside the thresholds.
			if point[0] < args['pointcloud_min_dist'] or point[0] > args['pointcloud_max_dist'] or \
			   point[2] < args['pointcloud_min_height'] or point[2] > args['pointcloud_max_height']:
				continue

			# project 3D point to 2D uv 
			rotatedPoint = rotationMatrix.dot( point )
			uv = camera_model.project3dToPixel( rotatedPoint )
			#objects[objects.keys()[0]]['projected_points'].append([int( uv[0] ),int( uv[1] )])
			# check that projected points are valid and inside the detected bounding box
			if uv[0] >= 0 and uv[0] <= img_width and uv[1] >= 0 and uv[1] <= img_height:
				#objects[objects.keys()[0]]['projected_points'].append([int( uv[0] ),int( uv[1] )])
				# loop over the objects
				for tube_id in objects.keys():
					x1, y1, x2, y2 = objects[tube_id]['image_coords']

					if uv[0] >= x1 and uv[0] <= x2 and uv[1] >= y1 and uv[1] <= y2:
						# save closest point in bounding box
						if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) <  objects[tube_id]['closest_dist']:
							# transform from velodyne frame to map frame
							point_trans = rotationMatrix_vel.dot(point)
							# save world point
							objects[tube_id]['world_2D_coords'] = point_trans
							# save relative point
							objects[tube_id]['rel_2D_coords'] = point
							# save closest distance
							objects[tube_id]['closest_dist']  = math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) )
						# save projected point
						objects[tube_id]['projected_points'].append([int( uv[0] ),int( uv[1] )])

	# return data
	return objects
	'''

