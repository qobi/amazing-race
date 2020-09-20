from __future__ import print_function
import pickle
import tf
import cv2
import os
import numpy as np
import math
import sys
import scipy
import argparse

from mpl_toolkits.mplot3d import Axes3D

import time
from scipy.optimize import linear_sum_assignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams.update({'font.size': 10})

from scipy.spatial.distance import cdist, pdist
from scipy.cluster.hierarchy import fclusterdata
from simplification.cutil import simplify_coords#, simplify_coords_vw, simplify_coords_vwp

#set up transformation matrices
trans = [-0.03596812, 0.27371754, -0.08881337]
rot = [0.544521193495, -0.512626565799, 0.452427950113, 0.485818509146]
rot_euler = tf.transformations.euler_from_quaternion(rot)
rot = tf.transformations.quaternion_from_euler(2.68, -1.42, -1.1)
trans = tuple(trans) + ( 1,  )
rotationMatrix = tf.transformations.quaternion_matrix( rot )
rotationMatrix[ :, 3 ] = trans

lsd = cv2.createLineSegmentDetector(0)
def line_to_points(line, step):
    x1, y1, x2, y2 = line
    left = min([[x1, y1], [x2, y2]], key = lambda x:x[0])
    right = max([[x1, y1], [x2, y2]], key = lambda x:x[0])
    x1, y1 = left
    x2, y2 = right
    m = (y2-y1)/(x2-x1)
    b = -(m*x1 - y1)
    x = np.arange(x1, x2, step)
    y = m*x + b
    return zip(x, y)

def line_equation(lines):
    x1 = lines[:,0]
    y1 = lines[:,1]
    x2 = lines[:,2]
    y2 = lines[:,3]
    m = (y2-y1)/(x2-x1)
    b = -(m*x1 - y1)
    return m[:,np.newaxis], b[:,np.newaxis]

def RDP_laser(vel_points, rings, map_vel_points, camera_model, rotationMatrix, top, bot, min_dist, rdp_thresh, point_thresh, dist_thresh, bound):
    #only keep points that are at least a certain distance in front of the robot
    min_dist_cond = (vel_points[:,0] >= min_dist)
    vel_points = vel_points[min_dist_cond]
    map_vel_points = map_vel_points[min_dist_cond]
    rings = rings[min_dist_cond]

    #keep first two coordinates (x, y) of points
    des_points = vel_points[rings == 8] #just need to take first 2 coords
    des_points_map = map_vel_points[rings == 8] #just need to take first 2 coords
    rdp_time = time.time()
    
    #apply RDP algorithm to points to create line segments
    #If i take out half the points, it takes half the time
    #rdp_points = rdp(des_points[:,0:2], epsilon = rdp_thresh) #threshold for rdp
    #output are the remaining points that define the piecewise function
    #print (len(rdp_points))
    #print (rdp_points)
    #print ("RDP time", time.time() - rdp_time)

    simplify_time = time.time()
    rdp_points = simplify_coords(des_points[:,0:2], rdp_thresh)
    #find out the original indices of these points
    #this creates a binary mask of original points that are in rdp_points
    mask = np.isin(des_points[:,0:2], rdp_points)
    #this extracts the actual indices, I do a step size of 2 because it outputs duplicate indices (since 2 coordinates)
    indeces = np.nonzero(mask)[0][::2]
    #I take the difference between consecutive indices to figure out how many points lie on those segments
    indeces_diff = indeces[1:] - indeces[:-1]
    #I compute the 3d length of these line segments
    indeces_dist = np.sqrt((des_points[indeces[1:],0] - des_points[indeces[:-1],0])**2 + (des_points[indeces[1:],1] - des_points[indeces[:-1],1])**2)

    #I only keep indices whose length and # points satisfy minimum thresholds
    #The indices go from left to right (clockwise starting from the robot's left hand side)
    #indeces_left and indeces_right define the line segments, so [indeces_left[i], indeces_right[i]] is the ith line segment
    indeces_left = indeces[:-1][(indeces_diff > point_thresh) & (indeces_dist > dist_thresh)] #thresholds for filtering segments by points and length
    indeces_right = indeces[1:][(indeces_diff > point_thresh) & (indeces_dist > dist_thresh)] #thresholds for filtering segments by points and length

    #With the points extracted, I transform them into pixel coordinates
    #TODO I apply this transformation to all points, but I can just apply it to des_points[indeces_right] and des_points[indeces_left]
    one_col = np.ones((des_points.shape[0], 1))

    #transform laser points at ground level to pixels
    laser_points_top = np.hstack((des_points[:,0:2], one_col*top, one_col))
    laser_uvs_top = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(laser_points_top))))
    laser_uvs_top = np.array(laser_uvs_top[:,0:2]/laser_uvs_top[:,2])

    #transform laser points at ground level to pixels
    laser_points_bot = np.hstack((des_points[:,0:2], one_col*bot, one_col))
    laser_uvs_bot = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(laser_points_bot))))
    laser_uvs_bot = np.array(laser_uvs_bot[:,0:2]/laser_uvs_bot[:,2])

    #generate top boundary for top segments
    top_top_boundary = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((des_points[:,0:2], one_col*(top+bound), one_col))))))
    top_top_boundary = np.array(top_top_boundary[:,0:2]/top_top_boundary[:,2])

    #generate bottom boundary for top segments
    top_bot_boundary = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((des_points[:,0:2], one_col*(top-bound), one_col))))))
    top_bot_boundary = np.array(top_bot_boundary[:,0:2]/top_bot_boundary[:,2])

    #generate top boundary for bottom segments
    bot_top_boundary = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((des_points[:,0:2], one_col*(bot+bound), one_col))))))
    bot_top_boundary = np.array(bot_top_boundary[:,0:2]/bot_top_boundary[:,2]) 

    #generate bottom boundary for bottom segments
    bot_bot_boundary = np.transpose(camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((des_points[:,0:2], one_col*(bot-bound), one_col))))))
    bot_bot_boundary = np.array(bot_bot_boundary[:,0:2]/bot_bot_boundary[:,2])

    #assemble segments
    laser_segs_uv_top = np.hstack((laser_uvs_top[indeces_left,:], laser_uvs_top[indeces_right,:])).astype(int)
    laser_segs_uv_bot = np.hstack((laser_uvs_bot[indeces_left,:], laser_uvs_bot[indeces_right,:])).astype(int)

    top_top_boundary_segs = np.hstack((top_top_boundary[indeces_left,:], top_top_boundary[indeces_right,:])).astype(int)
    top_bot_boundary_segs = np.hstack((top_bot_boundary[indeces_left,:], top_bot_boundary[indeces_right,:])).astype(int)
    bot_top_boundary_segs = np.hstack((bot_top_boundary[indeces_left,:], bot_top_boundary[indeces_right,:])).astype(int)
    bot_bot_boundary_segs = np.hstack((bot_bot_boundary[indeces_left,:], bot_bot_boundary[indeces_right,:])).astype(int)


    laser_segs_3d = np.hstack((des_points[indeces_left,0:2], des_points[indeces_right,0:2]))
    laser_segs_map = np.hstack((des_points_map[indeces_left,0:2], des_points_map[indeces_right,0:2]))

    #return everything
    return laser_segs_3d, laser_segs_map, laser_segs_uv_top, laser_uvs_top, laser_segs_uv_bot, laser_uvs_bot, [top_top_boundary_segs, top_bot_boundary_segs, bot_top_boundary_segs, bot_bot_boundary_segs]

def merge_lines(lines, lines_uv_top, lines_uv_bot, boundary_segs, m_thresh, b_thresh, length_thresh):
    #goal is to merge lines that belong to the same wall
    #cluster by slope, rotate, then cluster by b

    #get boundary segs
    top_top_boundary_segs, top_bot_boundary_segs, bot_top_boundary_segs, bot_bot_boundary_segs = boundary_segs

    #convert lines into numpy array
    lines = np.array(lines).astype(float)
    lines_adj = lines
    #compute angles of all lines
    og_line_thetas = np.mod(np.arctan2(lines[:,3] - lines[:,1], lines[:,2] - lines[:,0]).reshape((lines[:,1].shape[0], 1)), 2*math.pi)
    #print ("og line thetas")
    #print (og_line_thetas)
    #initialize arrays
    merged_lines_top = []
    merged_lines_bot = []
    merged_lines_3d = []
    merged_lines_top_top = []
    merged_lines_top_bot = []
    merged_lines_bot_top = []
    merged_lines_bot_bot = []

    #only perform if there's at least 2 lines
    if lines.shape[0] > 1:
        #cluster lines based on their angle to group lines with similar angle
        clusters =  fclusterdata(og_line_thetas, m_thresh, criterion='distance')

        #iterate over each cluster
        for i in range(1,max(clusters)+1):

            #get the uv lines that correspond to this cluster
            line_set_top = lines_uv_top[clusters==i,:]
            line_set_bot = lines_uv_bot[clusters==i,:]

            #get the uv boundary lines that correspond to this cluster
            line_set_top_top = top_top_boundary_segs[clusters==i,:]
            line_set_top_bot = top_bot_boundary_segs[clusters==i,:]
            line_set_bot_top = bot_top_boundary_segs[clusters==i,:]
            line_set_bot_bot = bot_bot_boundary_segs[clusters==i,:]
            #get the 3d lines that correspond to this cluster
            line_set_3d = lines[clusters==i,:] # 3d

            
            b_old = ((line_set_3d[:,1] + line_set_3d[:,3])/2.0).reshape((line_set_3d[:,1].shape[0], 1))
            #get copy of the 3d line that correspond to this cluster
            line_set_copy = lines[clusters==i,:] # 3d copy

            #compute angles of lines in this cluster
            #TODO just index into og_line_thetas above to do this
            line_thetas = np.arctan2(line_set_3d[:,3] - line_set_3d[:,1], line_set_3d[:,2] - line_set_3d[:,0])

            #print ("Cluster %d"%i)
            #print (line_thetas)
            #print (line_set_bot)
            #rotate all of the lines by their angles so that they are all approximately parallel to the y axis
            cos_theta = np.cos(-line_thetas)
            sin_theta = np.sin(-line_thetas)
            line_set_3d[:,0] = line_set_copy[:,0]*cos_theta - line_set_copy[:,1]*sin_theta
            line_set_3d[:,1] = line_set_copy[:,0]*sin_theta + line_set_copy[:,1]*cos_theta
            line_set_3d[:,2] = line_set_copy[:,2]*cos_theta - line_set_copy[:,3]*sin_theta
            line_set_3d[:,3] = line_set_copy[:,2]*sin_theta + line_set_copy[:,3]*cos_theta

            
            #compute angles of rotated lines
            #TODO why do I do this? left over from debugging?
            new_line_thetas = np.arctan2(line_set_3d[:,3] - line_set_3d[:,1], line_set_3d[:,2] - line_set_3d[:,0])

            #compute new b by computing distance of rotated lines to y axis
            b_new = ((line_set_3d[:,1] + line_set_3d[:,3])/2.0).reshape((line_set_3d[:,1].shape[0], 1))
            
            #only perform if there's at least 2 lines
            if b_new.shape[0] > 1:

                #cluster lines in this cluster by b to separate distinct walls that are parallel
                new_clusters =  fclusterdata(b_new, b_thresh, criterion='distance')

                #iterate over these clusters, to merge the lines in each cluster
                for j in range(1, max(new_clusters)+1):
                    #print ("Sub cluster %d"%j)
                    #get the 3d lines that correspond to this cluster
                    new_line_set_3d = line_set_3d[new_clusters==j,:]
                    old_line_set_3d = line_set_copy[new_clusters==j,:]
                    #get the top uv lines that correspond to this cluster
                    new_line_set_top = line_set_top[new_clusters==j,:]
                    #print (b_new[new_clusters==j,:])
                    #print (line_set_bot[new_clusters==j,:])
                    #get leftmost and rightmost points in the uv lines (pixel coordinates)
                    left = min(new_line_set_top, key = lambda x:x[0])
                    right = max(new_line_set_top, key = lambda x:x[2])
                    #get the 3d points corresponding to the leftmost and rightmost points in pixel coordinates
                    left_idx = (np.where((new_line_set_top==left).all(axis=1))[0][0])
                    right_idx = (np.where((new_line_set_top==right).all(axis=1))[0][0])
                    left_3d = old_line_set_3d[left_idx,:]
                    right_3d = old_line_set_3d[right_idx, :]
                    #compute the potential distance of the combined line segment
                    dist_3d = math.sqrt((left_3d[0] - right_3d[2])**2 + (left_3d[1] - right_3d[3])**2)

                    #if the distnace satisfies the threshold, generate the merged lines and add them
                    if dist_3d > length_thresh:
                        #append top uv line
                        merged_lines_top.append([left[0], left[1], right[2], right[3]])
                        merged_lines_3d.append([left_3d[0], left_3d[1], right_3d[2], right_3d[3]])
                        #repeat operation for bottom uv line
                        new_line_set_bot = line_set_bot[new_clusters==j,:]
                        left = min(new_line_set_bot, key = lambda x:x[0])
                        right = max(new_line_set_bot, key = lambda x:x[2])
                        merged_lines_bot.append([left[0], left[1], right[2], right[3]])

                        new_line_set_top_top = line_set_top_top[new_clusters==j,:]
                        left = min(new_line_set_top_top, key = lambda x:x[0])
                        right = max(new_line_set_top_top, key = lambda x:x[2])
                        merged_lines_top_top.append([left[0], left[1], right[2], right[3]])

                        new_line_set_top_bot = line_set_top_bot[new_clusters==j,:]
                        left = min(new_line_set_top_bot, key = lambda x:x[0])
                        right = max(new_line_set_top_bot, key = lambda x:x[2])
                        merged_lines_top_bot.append([left[0], left[1], right[2], right[3]])

                        new_line_set_bot_top = line_set_bot_top[new_clusters==j,:]
                        left = min(new_line_set_bot_top, key = lambda x:x[0])
                        right = max(new_line_set_bot_top, key = lambda x:x[2])
                        merged_lines_bot_top.append([left[0], left[1], right[2], right[3]])

                        new_line_set_bot_bot = line_set_bot_bot[new_clusters==j,:]
                        left = min(new_line_set_bot_bot, key = lambda x:x[0])
                        right = max(new_line_set_bot_bot, key = lambda x:x[2])
                        merged_lines_bot_bot.append([left[0], left[1], right[2], right[3]])
                        
            #if there is only one line
            else:
                #add the corresponding top and bottom uv lines if this line's 3d length is greater than threshold
                if math.sqrt((line_set_3d[0,0] - line_set_3d[0,2])**2 + (line_set_3d[0,1] - line_set_3d[0,3])**2) > length_thresh:
                    merged_lines_3d.append(line_set_copy[0,:])
                    merged_lines_top.append(line_set_top[0,:])
                    merged_lines_bot.append(line_set_bot[0,:])
                    merged_lines_top_top.append(line_set_top_top[0,:])
                    merged_lines_top_bot.append(line_set_top_bot[0,:])
                    merged_lines_bot_top.append(line_set_bot_top[0,:])
                    merged_lines_bot_bot.append(line_set_bot_bot[0,:])

    #return top and bottom merged uv lines
    return np.array(merged_lines_3d), merged_lines_top, merged_lines_bot, [merged_lines_top_top, merged_lines_top_bot, merged_lines_bot_top, merged_lines_bot_bot]

def detect_all_lines_new(img, lsd):
    im_copy = img.copy()
    #Create default parametrization LSD
    
    start = time.time()
    #Detect lines in the image with line segment detector
    lines = lsd.detect(cv2.cvtColor(cv2.resize(im_copy, dsize = (1280,720)),cv2.COLOR_BGR2GRAY))[0] #Position 0 of the returned tuple are the detected lines
    #lines = lsd.detect(cv2.cvtColor(im_copy,cv2.COLOR_BGR2GRAY))[0] #Position 0 of the returned tuple are the detected lines
    #print (lines.shape)

    #create copy of image
    im_mask = img.copy()
    #im_mask_horz = img.copy()
    im_mask[:,:,:] = 0
    #im_mask_horz[:,:,:] = 0

    #convert lines into numpy array and create a copy
    if lines is None:
        return [np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([])]#
    lines = np.array(lines).reshape((len(lines), 4))
    lines = (lines * np.array([1920.0/1280.0, 1080.0/720.0, 1920.0/1280.0, 1080.0/720.0])).astype(int)
    lines_copy = lines.copy()
   # print (lines.shape)

    #lines are defined as a pair of (x,y) points
    #rearrange all lines so that the first point has the smaller x value
    flip_mask = lines[:,0] > lines[:,2]
    lines[flip_mask,0:2] = lines_copy[flip_mask, 2:4]
    lines[flip_mask,2:4] = lines_copy[flip_mask, 0:2]

    #compute angle of all lines
    line_thetas = np.arctan2(lines[:,3] - lines[:,1], lines[:,2] - lines[:,0])

    #create different logical conditions to put the lines into different groups

    #separate into lines above the center line 
    top_cond = np.logical_and(lines[:,1] <= 1080/2, lines[:,3] <= 1080/2)
    bot_cond = np.logical_and(lines[:,1] > 1080/2, lines[:,3] > 1080/2)

    
    backslash_cond = np.logical_and(line_thetas > np.pi/16, line_thetas < 3*np.pi/8) #theta_line > np.pi/8 and theta_line < 3*np.pi/8
    forwslash_cond = np.logical_and(line_thetas > -3*np.pi/8, line_thetas < -np.pi/16) #theta_line > -3*np.pi/8 and theta_line < -np.pi/8
    dashslash_cond = np.logical_and(line_thetas > -np.pi/16, line_thetas < np.pi/16) #theta_line < np.pi/8 and theta_line > -np.pi/8
    vertline_cond = np.logical_or(np.logical_and(line_thetas > -np.pi/2 -.05, line_thetas < -np.pi/2 + .05), np.logical_and(line_thetas > np.pi/2 -.05, line_thetas < np.pi/2 + .05)) 

    horz_lines_left_top =   lines[np.logical_and(top_cond, backslash_cond)] # \ above 1080/2 #backslash
    horz_lines_left_bot =   lines[np.logical_and(bot_cond, forwslash_cond)] # / below 1080/2 #forwslash
    horz_lines_right_top =  lines[np.logical_and(top_cond, forwslash_cond)] # / above 1080/2 #forwslash
    horz_lines_right_bot =  lines[np.logical_and(bot_cond, backslash_cond)] # \ below 1080/2 #backslash
    horz_lines_middle_top = lines[np.logical_and(top_cond, dashslash_cond)] # - above 1080/2 #dashslash
    horz_lines_middle_bot = lines[np.logical_and(bot_cond, dashslash_cond)] # - below 1080/2 #dashslash

    vert_lines = lines[vertline_cond]
    for vert_line in vert_lines:
        x1, y1, x2, y2 = vert_line
        cv2.line(im_mask,(x1,y1),(x2,y2),(255,255,255),2)
    
    im_mask = cv2.dilate(im_mask, np.ones((3,3), np.uint8) , iterations=1) 
    im_mask = cv2.erode(im_mask, np.ones((3,3), np.uint8) , iterations=1)
    output = cv2.connectedComponentsWithStats(cv2.cvtColor(im_mask, cv2.COLOR_RGB2GRAY), 8, cv2.CV_32S)
    vert_boxes = output[2][:,[cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT]]
    vert_boxes = vert_boxes[vert_boxes[:,2] < 1900]
    vert_lines_merge = np.array(vert_boxes).astype(float)
    vert_lines_merge[:,0] = vert_boxes[:,0] + vert_boxes[:,2]/2.0
    vert_lines_merge[:,2] = vert_boxes[:,0] + vert_boxes[:,2]/2.0
    vert_lines_merge[:,3] = vert_boxes[:,1] + vert_boxes[:,3]

    return [vert_lines.astype(int), vert_lines_merge.astype(int), horz_lines_left_top, horz_lines_left_bot, horz_lines_right_top, horz_lines_right_bot, horz_lines_middle_top, horz_lines_middle_bot]
    #change cov_thresh_x and cov_thresh_y names
def door_proposals(lines_3d, all_lines, bound_segs_top_top, bound_segs_top_bot, edge_lines, uv_points, vel_points, map_vel_points, rings, img, camera_model, rotationMatrix, length_thresh, cov_thresh_x, cov_thresh_y, top_thresh_meters):
    
    viz = False

    min_dist_cond = (vel_points[:,0] >= 3.0)
    uv_points = uv_points[min_dist_cond]
    vel_points = vel_points[min_dist_cond]
    map_vel_points = map_vel_points[min_dist_cond]
    rings = rings[min_dist_cond]

    ring_uv_points = uv_points[rings==8]
    ring_map_vel_points = map_vel_points[rings==8]
    ring_vel_points = vel_points[rings==8]
    #get edge lines
    vert_lines, vert_lines_merge, horz_lines_left_top, horz_lines_left_bot, horz_lines_right_top, horz_lines_right_bot, horz_lines_middle_top, horz_lines_middle_bot = edge_lines
    all_horz_top = np.vstack((horz_lines_left_top,horz_lines_right_top,horz_lines_middle_top))
    #all_horz_bot = horz_lines_left_bot + horz_lines_right_bot + horz_lines_middle_bot

    ring_uv_points_x = ring_uv_points[:,0]
    #get top and bottom wall (laser fitted) lines
    all_lines_x_top, all_lines_y_top, all_lines_x_bot, all_lines_y_bot = all_lines

    #initialize return variables, vertical lines, top lines, and bottom lines that belong to doors
    new_vert_lines = []
    sel_top_lines = []
    sel_bot_lines = []

    #initialize door list
    doors = []
    scores = []
    locs = []
    #iterate over the pairs of wall lines
    viz_im = img.copy()
    for line_3d, line_x_top, line_y_top, line_x_bot, line_y_bot, top_bound, bot_bound in zip(lines_3d, all_lines_x_top, all_lines_y_top, all_lines_x_bot, all_lines_y_bot, bound_segs_top_top, bound_segs_top_bot):
        #do this only if both top line and bottom line exist 
        orientation = np.arctan2(line_y_top[1] - line_y_top[0], line_x_top[1] - line_x_top[0])
        if np.logical_and(orientation > np.pi/16, orientation < 3*np.pi/8):
            all_horz_top = horz_lines_left_top
        elif np.logical_and(orientation > -3*np.pi/8, orientation < -np.pi/16):
            all_horz_top = horz_lines_right_top
        else:
            all_horz_top = horz_lines_middle_top
        loop_count = 0
        loop_count = 0
        loop_start = time.time()
        cv2.line(viz_im, (line_x_top[0], line_y_top[0]), (line_x_top[1], line_y_top[1]), (0, 255, 0))
        if line_x_top.shape[0] > 1 and line_x_bot.shape[0] > 1 and vert_lines.shape[0] > 1:

            #use both lines to determine an x range for vertical lines
            max_x = np.max(np.hstack((line_x_top, line_x_bot)))
            min_x = np.min(np.hstack((line_x_top, line_x_bot)))

            #grab x coordinates of vertical lines
              
            vert_lines_x_coords = (vert_lines[:,0].astype(float) + vert_lines[:,2].astype(float))/2.0
            
            #collect vertical lines that belong to this wall based on x coordinates
            inlier_vert_lines = vert_lines[np.logical_and(vert_lines_x_coords >= min_x, vert_lines_x_coords <= max_x)]

            #collect the x coordinates of the lines that belong to this wall
            inlier_vert_lines_x_coords = vert_lines_x_coords[np.logical_and(vert_lines_x_coords >= min_x, vert_lines_x_coords <= max_x)]
            
            #add dimension in x coordinate array
            vert_x_coords = inlier_vert_lines_x_coords #inlier_vert_lines[:,0]
            vert_x_coords = vert_x_coords[:,np.newaxis]

            #Compute equation of top line and bottom line
            m_top, b_top = line_equation(np.array([[line_x_top[0], line_y_top[0], line_x_top[1], line_y_top[1]]]).astype(float))
            m_bot, b_bot = line_equation(np.array([[line_x_bot[0], line_y_bot[0], line_x_bot[1], line_y_bot[1]]]).astype(float))

            #Compute equations for the bounds of the top line
            m_top_bound, b_top_bound = line_equation(np.array([[top_bound[0], top_bound[1], top_bound[2], top_bound[3]]]).astype(float))
            m_bot_bound, b_bot_bound = line_equation(np.array([[bot_bound[0], bot_bound[1], bot_bound[2], bot_bound[3]]]).astype(float))
           
            #extend them to the top and bottom wall lines, determine these new y coordinates
            vert_lines_top_y = m_top*vert_x_coords + b_top
            vert_lines_bot_y = m_bot*vert_x_coords + b_bot

            #Generate conditions for vertical lines, if they lie entirely outside of the wall (above or below), remove them
            y_cond1 = vert_lines_top_y[:,0] > inlier_vert_lines[:,1] #top point above
            y_cond2 = inlier_vert_lines[:,1] > vert_lines_bot_y[:,0] #top point below
            y_cond3 = vert_lines_top_y[:,0] > inlier_vert_lines[:,3] #bottom point above
            y_cond4 = inlier_vert_lines[:,3] > vert_lines_bot_y[:,0] #bottom point below
            y_cond = np.logical_not(np.logical_or(np.logical_and(y_cond1, y_cond3), np.logical_and(y_cond2, y_cond4)))

            #extract inlier (not extended) lines based on condition
            inlier_vert_lines = inlier_vert_lines[y_cond]
            inlier_vert_lines_x_coords = inlier_vert_lines_x_coords[y_cond]
            inlier_vert_lines_x_coords = inlier_vert_lines_x_coords[:,np.newaxis]

            #Now determine 3d location of these lines by finding the closest laser point to each line in the image plane
            #TODO consider using entire pointcloud to do this

            #compute x distance from laser points to lines
            uv_vert_diff = abs(ring_uv_points_x[np.newaxis,:] - inlier_vert_lines_x_coords)

            #find cloest laser point
            uv_vert_diff_min = np.argmin(uv_vert_diff, axis=1)

            #assign location of closest laser point as location of line
            og_lines_cat_loc = ring_vel_points[uv_vert_diff_min,:]

            #cluster these locations to get a smaller set of vertical lines
            if og_lines_cat_loc.shape[0] < 2:
                continue
            #clusters =  fclusterdata(og_lines_cat_loc, 0.1, criterion='distance')

            #iterate over clusters to get new x coordinates
            #new_x_coords = []
            #TODO only append clusters that are large enough
            #for i in range(1,max(clusters)+1):
            #    clustered_x = inlier_vert_lines_x_coords[clusters==i,:]
            #    if clustered_x.shape[0] > 1:
            #        new_x_coords.append(np.mean(clustered_x).astype(int))
            #new_x_coords = [np.mean(inlier_vert_lines_x_coords[clusters==i,:]) for i in range(1,max(clusters)+1)]

            
            #inlier_vert_lines_x_coords = np.array(new_x_coords)
            inlier_vert_lines_merge = vert_lines_merge[np.logical_and(vert_lines_merge[:,0] >= min_x, vert_lines_merge[:,0] <= max_x),:]
            inlier_vert_lines_x_coords = inlier_vert_lines_merge[:,0]
            inlier_vert_lines_x_coords = inlier_vert_lines_x_coords[:,np.newaxis]

            #extend them to the top and bottom wall lines
            new_lines_top_y = m_top*inlier_vert_lines_x_coords + b_top
            new_lines_bot_y = m_bot*inlier_vert_lines_x_coords + b_bot
            height_old = abs(inlier_vert_lines_merge[:,1] - inlier_vert_lines_merge[:,3]).astype(float)
            height_new = abs(new_lines_top_y - new_lines_bot_y).astype(float)[:,0]
            height_ratio = height_old/height_new
            #print (height_ratio.shape, height_old.shape, height_new.shape)
            ratio_cond = height_old/height_new > .25
            inlier_vert_lines_x_coords = inlier_vert_lines_x_coords[ratio_cond,:]
            new_lines_top_y = new_lines_top_y[ratio_cond,:]
            new_lines_bot_y = new_lines_bot_y[ratio_cond,:]
            #extract extended lines based on condition
            #print (inlier_vert_lines_x_coords.shape)
            lines_cat = np.hstack((inlier_vert_lines_x_coords, new_lines_top_y, inlier_vert_lines_x_coords, new_lines_bot_y))

            # Sort lines from left to right (in horizontal pixel coordinates)
            lines_cat = lines_cat[lines_cat[:,0].argsort()]
            lines_cat_x = lines_cat[:,0]
            lines_cat_x = lines_cat_x[:,np.newaxis]

            #get distance to uv points
            uv_vert_diff = abs(ring_uv_points_x[np.newaxis,:] - lines_cat_x)

            #compute closest laser point to each line
            uv_vert_diff_min = np.argmin(uv_vert_diff, axis=1)

            #grab those laser point locations
            lines_cat_loc = ring_vel_points[uv_vert_diff_min,:]
            lines_cat_loc_map = ring_map_vel_points[uv_vert_diff_min,:]
            #compute pairwise distances between all of the lines
            lines_cat_dist = cdist(lines_cat_loc[:,0:2],lines_cat_loc[:,0:2])#,lines_cat_loc[:,0:2])

            #compute indices of lines that are within a (approximately) door's distance of each other
            pdist_indices = np.where(np.logical_and(lines_cat_dist >= 0.5, lines_cat_dist <= 1.25))

            #compute distance between original line segments and the new clustered ones
            og_new_dist = cdist(lines_cat_loc[:,0:2], og_lines_cat_loc[:,0:2])

            # Iterate over pairs of lines from left to right, only look at possible pairs to right of line
            num_loops = 0     
            #print (pdist_indices, pdist_indices[0].shape[0])
            pdist_indices_zip = np.array(list(zip(pdist_indices[0], pdist_indices[1])))
            #print (pdist_indices_zip)
            #if pdist_indices[0].shape[0] < 1 and pdist_indices[1].shape[0] < 1:
                #continue
            pdist_indices_zip = pdist_indices_zip[pdist_indices[0] < pdist_indices[1]]

            # begin loop to iterate over pairs
            #TODO i'm repeating computations for lines
            all_vert_coverage = np.zeros((lines_cat_loc.shape[0])) - 1
            door_theta = np.arctan2(line_3d[3] - line_3d[1], line_3d[2] - line_3d[0])
            checked_list = []
            best_door = [0]*lines_cat.shape[0]
            best_score = [0]*lines_cat.shape[0]
            best_loc = [0]*lines_cat.shape[0]

            for i,j in pdist_indices_zip:
                if i in checked_list:
                    continue
                if viz:
                    im = img.copy()
                    cv2.line(im, (int(line_x_top[0]), int(line_y_top[0])), (int(line_x_top[1]), int(line_y_top[1])), (0, 255, 255), 2)
                    cv2.line(im, (int(line_x_bot[0]), int(line_y_bot[0])), (int(line_x_bot[1]), int(line_y_bot[1])), (0, 255, 255), 2)
                    cv2.line(im, (int(top_bound[0]), int(top_bound[1])), (int(top_bound[2]), int(top_bound[3])), (0, 169, 255), 2)
                    cv2.line(im, (int(bot_bound[0]), int(bot_bound[1])), (int(bot_bound[2]), int(bot_bound[3])), (0, 169, 255), 2)
                #grab lines from this pair
                line1 = lines_cat[i,:]
                line2 = lines_cat[j,:]

                #grab their locations
                line1_loc = lines_cat_loc[i,:]
                line2_loc = lines_cat_loc[j,:]

                line1_loc_map = lines_cat_loc_map[i,:]
                line2_loc_map = lines_cat_loc_map[j,:]

                
                border_dist = 0.15
                this_door_theta = np.arctan2(line2_loc[1] - line1_loc[1], line2_loc[0] - line1_loc[0])
                #print (door_theta, this_door_theta)
                left_border = border_dist*np.array([np.cos(door_theta), np.sin(door_theta)])
                right_border = border_dist*np.array([np.cos(-door_theta), np.sin(-door_theta)])
                l1_l = line1_loc[0:2] + left_border
                l1_r = line1_loc[0:2] - left_border
                l2_l = line2_loc[0:2] + left_border
                l2_r = line2_loc[0:2] - left_border
                l1_x_list = []
                #print (door_theta)
                #print (line1_loc)
                #print (line2_loc)
                #print (l1_l)
                #print (l1_r)
                #sys.exit()
                for border in [l1_l, l1_r]:
                    uv_top = camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((border, 0, 1)))))
                    uv_top = np.array(uv_top[:,0:2]/uv_top[:,2])
                    l1_x_list.append(uv_top[:,0])
                    #uv_bot = camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((border, -0.7, 1)))))
                    #uv_bot = np.array(uv_bot[:,0:2]/uv_bot[:,2])
                    #l1_x_list.append(uv_bot[:,0])
                    if viz:
                        cv2.line(im, (int(uv_top[:,0]), int(line1[1])), (int(uv_top[:,0]), int(line1[3])), (0, 127, 255), 2)
                l1_x_l = min(l1_x_list)
                l1_x_r = max(l1_x_list)
                l2_x_list = []
                for border in [l2_l, l2_r]:
                    uv_top = camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((border, 0, 1)))))
                    uv_top = np.array(uv_top[:,0:2]/uv_top[:,2])
                    l2_x_list.append(uv_top[:,0])
                    #uv_bot = camera_model.P.dot(rotationMatrix.dot(np.transpose(np.hstack((border, -0.7, 1)))))
                    #uv_bot = np.array(uv_bot[:,0:2]/uv_bot[:,2])
                    #l2_x_list.append(uv_bot[:,0])
                    if viz:
                        cv2.line(im, (int(uv_top[:,0]), int(line2[1])), (int(uv_top[:,0]), int(line2[3])), (0, 127, 255), 2)
                l2_x_l = min(l2_x_list)
                l2_x_r = max(l2_x_list)

                if viz:
                    print ("line dist: ", lines_cat_dist[i,j])
                #grab distance to original vertical line segments
                line1_dists = og_new_dist[i,:]
                line2_dists = og_new_dist[j,:]
                
                if all_vert_coverage[i] != -1:
                    vert_coverage_1 = all_vert_coverage[i]
                else:
                    vert_lines_1 = inlier_vert_lines[np.logical_and(line1_dists >= 0.0, line1_dists < 0.2)]
                    #vert_lines_1 = inlier_vert_lines[np.logical_and(inlier_vert_lines[:,0] >= l1_x_l, inlier_vert_lines[:,0] <= l1_x_r)]
                    vert_lines_1_unique = []
                    for line in vert_lines_1:
                        if viz:
                            cv2.line(im, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (0, 0, 255), 2)
                        vert_lines_1_unique.extend(range(int(min(line[1], line[3])), int(max(line[1], line[3]))))
                    vert_lines_1_unique = np.array(list(set(vert_lines_1_unique)))
                    vert_lines_1_unique = vert_lines_1_unique[np.logical_and(vert_lines_1_unique < line1[3], vert_lines_1_unique > line1[1])]
                    vert_coverage_1 = vert_lines_1_unique.shape[0]/(line1[3] - line1[1])
                    all_vert_coverage[i] = vert_coverage_1

                if all_vert_coverage[j] != -1:
                    vert_coverage_2 = all_vert_coverage[j]
                else:
                    vert_lines_2 = inlier_vert_lines[np.logical_and(line2_dists >= 0.0, line2_dists < 0.2)] 
                    #vert_lines_2 = inlier_vert_lines[np.logical_and(inlier_vert_lines[:,0] >= l2_x_l, inlier_vert_lines[:,0] <= l2_x_r)]
                    vert_lines_2_unique = []
                    for line in vert_lines_2:
                        if viz:                    
                            cv2.line(im, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (0, 0, 255), 2)
                        vert_lines_2_unique.extend(range(int(min(line[1], line[3])), int(max(line[1], line[3]))))
                    vert_lines_2_unique = np.array(list(set(vert_lines_2_unique)))
                    vert_lines_2_unique = vert_lines_2_unique[np.logical_and(vert_lines_2_unique < line2[3], vert_lines_2_unique > line2[1])]
                    vert_coverage_2 = vert_lines_2_unique.shape[0]/(line2[3] - line2[1])
                    all_vert_coverage[j] = vert_coverage_2

                #generate line that connects the tops
                top_line = np.hstack((line1[0:2], line2[0:2])).astype(float)

                #generate line that connects the bottoms    
                bot_line = np.hstack((line1[2:4], line2[2:4])).astype(float)

                #visualization for debugging
                if viz:
                    cv2.line(im, (int(line1[0]), int(line1[1])), (int(line1[2]), int(line1[3])), (0, 255, 0), 2)
                    cv2.line(im, (int(line2[0]), int(line2[1])), (int(line2[2]), int(line2[3])), (0, 255, 0), 2)
                
                #Begin filtering out horizontal lines to check if these two vertical lines form a door

                #get distance to uv points
                uv_horz_diff_0 = abs(ring_uv_points_x[np.newaxis,:] - all_horz_top[:,0][:,np.newaxis])
                uv_horz_diff_2 = abs(ring_uv_points_x[np.newaxis,:] - all_horz_top[:,2][:,np.newaxis])

                #compute closest laser point to each line
                uv_horz_diff_min_0 = np.argmin(uv_horz_diff_0, axis=1)
                uv_horz_diff_min_2 = np.argmin(uv_horz_diff_2, axis=1)

                #grab those laser point locations
                horz_loc_0 = ring_vel_points[uv_horz_diff_min_0,:]
                horz_loc_2 = ring_vel_points[uv_horz_diff_min_2,:]
                
                vert_1_horz_0_dist = cdist(horz_loc_0, line1_loc[np.newaxis,:])
                vert_2_horz_2_dist = cdist(horz_loc_2, line2_loc[np.newaxis,:])

                h_x_cond = np.logical_or(vert_1_horz_0_dist < .5, vert_2_horz_2_dist < .5)[:,0]
                #print (h_x_cond.shape)
                #compute y difference between left and right point of top line
                top_line_y_diff = abs(top_line[1] - top_line[3])

                #compute points on horizontal lines of where they would lie on the top line with their x coordinate
                y_top_0 = m_top*all_horz_top[:,0] + b_top
                y_top_2 = m_top*all_horz_top[:,2] + b_top

                #compute same thing for the bounds
                y_top_0_bound = m_top_bound*all_horz_top[:,0] + b_top_bound
                y_top_2_bound = m_top_bound*all_horz_top[:,2] + b_top_bound
 
                y_bot_0_bound = m_bot_bound*all_horz_top[:,0] + b_bot_bound
                y_bot_2_bound = m_bot_bound*all_horz_top[:,2] + b_bot_bound

                #compute height of vertical lines in pixels
                line1_h = line1[3] - line1[1]
                line2_h = line2[3] - line2[1]
                
                #Generate vertical conditions based on the residual difference and the threshold
                h_y_cond1 = np.logical_and(all_horz_top[:,1]>= y_top_0_bound, all_horz_top[:,1] <= y_bot_0_bound)
                h_y_cond2 = np.logical_and(all_horz_top[:,3] >= y_top_2_bound, all_horz_top[:,3] <= y_bot_2_bound)
                h_y_cond = np.logical_and(h_y_cond1, h_y_cond2)

                h_x_cond = np.logical_and(all_horz_top[:,0] >= l1_x_l, all_horz_top[:,2] <= l2_x_r)
                #Get inlier horizontal lines that satisfy these conditions
                inlier_horz_lines = all_horz_top[np.logical_and(h_y_cond[0,:],h_x_cond)]
                #TODO do range coverage with these lines

                #Now I compute how much coverage these edges give to the top line
                unique_points = []
                #I iterate over all of the inlier horizontal lines 
                for line in inlier_horz_lines:
                    #I grab the x range that this line covers and add it to a list
                    unique_points.extend(range(int(line[0]), int(line[2])+1))
                    if viz:
                        cv2.line(im, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (255, 0, 0), 2)

                #I compute the set of this list to remove duplicates to determine the coverage. I convert to numpy array
                unique_points = np.array(list(set(unique_points)))
                
                unique_points_outside = unique_points[np.logical_or(unique_points < top_line[0], unique_points > top_line[2])]
                #I remove any points that lie outside of the vertical lines
                unique_points_inside = unique_points[np.logical_and(unique_points > top_line[0], unique_points < top_line[2])]
                #print ("horz coverage")
                #print (unique_points_inside)
                #print (top_line[2] - top_line[0])
                #I compute the magnitude of the top line
                top_mag = np.sqrt((top_line[0] - top_line[2])**2 + (top_line[1] - top_line[3])**2)

                inside_coverage = unique_points_inside.shape[0]/(top_line[2] - top_line[0])
                outside_coverage = unique_points_outside.shape[0]/(top_line[2] - top_line[0])
                
                #If the magnitude is nonzero (i.e. same line) and the coverage is greater than threshold, I append the lines as a door
                proposal_score = vert_coverage_1 + vert_coverage_2 + inside_coverage 
                if viz:
                    print ("score:", proposal_score, vert_coverage_1, vert_coverage_2, inside_coverage, outside_coverage)
                if top_mag > 0:# and proposal_score > 2.75:
                    if viz and proposal_score > 2.5:
                        cv2.line(im, (int(top_line[0]), int(top_line[1])), (int(top_line[2]), int(top_line[3])), (0, 255, 0), 2)
                    #if proposal_score > 2.5:
                        #checked_list.append(i)
                        #new_vert_lines.append(line1)
                        #new_vert_lines.append(line2)
                        #sel_top_lines.append(list(top_line))
                    #locs.append((line1_loc_map+line2_loc_map)/2.0)
                    #scores.append(proposal_score)
                    #doors.append([line1[0], line1[1], line2[2], line2[3]])
                    if proposal_score > best_score[i]:
                        best_score[i] = proposal_score
                        best_door[i] = j
                #cv2.imwrite('im.png', im)
                if viz:
                    cv2.imshow('dst', im)
                    cv2.waitKey(0)
                #sys.exit() 
                num_loops+=1
                #pick threshold based on line length
        #print (loop_count, time.time()-loop_start, num_loops)  
            pairs_list = [] 
            for i in range(len(best_score)):
                #grab lines from this pair
                j = best_door[i]
                if set([i,j]) not in pairs_list:
                    pairs_list.append(set([i,j]))
                else:
                    continue
                line1 = lines_cat[i,:]
                line2 = lines_cat[j,:]
              
                line1_loc_map = lines_cat_loc_map[i,:]
                line2_loc_map = lines_cat_loc_map[j,:]

                #locs.append((line1_loc_map+line2_loc_map)/2.0)
                locs.append([line1_loc_map,line2_loc_map])
                scores.append(best_score[i]/3.0)
                doors.append([line1[0], line1[1], line2[2], line2[3]]) 
                if best_score[i] > 2.5:
                    new_vert_lines.append(line1)
                    new_vert_lines.append(line2)
                    top_line = np.hstack((line1[0:2], line2[0:2])).astype(float)
                    sel_top_lines.append(list(top_line))
                    this_bot_line = np.hstack((line1[2:4], line2[2:4])).astype(float)
                    sel_bot_lines.append(list(this_bot_line))
        
    #sys.exit()      
    #return the lines that correspond to doors 
    cv2.imwrite('line_test.jpg', viz_im)       
    return doors, scores, locs, new_vert_lines, sel_top_lines, sel_bot_lines
                    

def get_doors_general(camera_model, img, velodyne_data, robot_loc):
    
    im = img.copy()

    #determine initial(reference) position and compute base to map transformation
    [trans_vel_o, rot_vel, quaternion_vel]  = robot_loc

    trans_vel = tuple(trans_vel_o) + ( 1,  )
    
    rotationMatrix_vel = tf.transformations.quaternion_matrix( quaternion_vel )
    rotationMatrix_vel[ :, 3 ] = trans_vel
    map_to_base = tf.transformations.inverse_matrix(rotationMatrix_vel)

    vel_points = np.array(velodyne_data)
    #vel_points = vel_points[(vel_points[:,0] >= 3.0)]
    rings = vel_points[:,4]
    vel_points = np.hstack((vel_points[:,0:3], np.ones((vel_points.shape[0], 1))))
    rotated_vel_points = rotationMatrix.dot(np.transpose(vel_points))
    uv_points = np.transpose(camera_model.P.dot(rotated_vel_points))
    uv_points = np.array(uv_points[:,0:2]/uv_points[:,2])
    map_vel_points = np.transpose(rotationMatrix_vel.dot(np.transpose(vel_points)))

    #Apply RDP algorithm to laser points for top and bottom          
    output = RDP_laser(vel_points, rings, map_vel_points, camera_model, rotationMatrix, 1.5, -.7, 3.0, .05, 3, 0.25, 0.15) #top, bot,min_dist, rdp_thresh, point_thresh, dist_thresh, bound
    laser_segs_3d, laser_segs_map, laser_segs_uv_top, laser_uvs_top, laser_segs_uv_bot, laser_uvs_bot, boundary_segs= output


    laser_segs_3d_m, laser_segs_uv_m_top, laser_segs_uv_m_bot, boundary_segs_m = merge_lines(laser_segs_3d, laser_segs_uv_top, laser_segs_uv_bot, boundary_segs, .25, .5, .5) #theta thresh, b thresh, length thresh
    
    laser_segs_uv_m = laser_segs_uv_m_top + laser_segs_uv_m_bot
    bound_segs_top_top, bound_segs_top_bot, bound_segs_bot_top, bound_segs_bot_bot = boundary_segs_m
    all_merged_segs = laser_segs_uv_m + bound_segs_top_top + bound_segs_top_bot# + bound_segs_bot_top + bound_segs_bot_bot

    edge_lines = detect_all_lines_new(img.copy(), lsd)
    vert_lines, vert_lines_merge, horz_lines_left_top, horz_lines_left_bot, horz_lines_right_top, horz_lines_right_bot, horz_lines_middle_top, horz_lines_middle_bot = edge_lines

    all_lines_x_top = []
    all_lines_y_top = []
    all_lines_x_bot = []
    all_lines_y_bot = []
    for i in range(len(laser_segs_uv_m_top)):
        point1_top = laser_segs_uv_m_top[i][0:2]
        point2_top = laser_segs_uv_m_top[i][2:4]
        point1_bot = laser_segs_uv_m_bot[i][0:2]
        point2_bot = laser_segs_uv_m_bot[i][2:4]
        top_left  = min([point1_top, point2_top], key = lambda x:x[0])
        top_right = max([point1_top, point2_top], key = lambda x:x[0])
        bot_left  = min([point1_bot, point2_bot], key = lambda x:x[0])
        bot_right = max([point1_bot, point2_bot], key = lambda x:x[0])

        all_lines_x_top.append(np.array([point1_top[0], point2_top[0]]))
        all_lines_y_top.append(np.array([point1_top[1], point2_top[1]]))
        all_lines_x_bot.append(np.array([point1_bot[0], point2_bot[0]]))
        all_lines_y_bot.append(np.array([point1_bot[1], point2_bot[1]]))

    all_lines = [all_lines_x_top, all_lines_y_top, all_lines_x_bot, all_lines_y_bot]
    
    doors, scores, locs, new_vert_lines, sel_top_lines, sel_bot_lines = door_proposals(laser_segs_3d_m, all_lines, bound_segs_top_top, bound_segs_top_bot, edge_lines, uv_points, vel_points, map_vel_points, rings, img, camera_model, rotationMatrix, .2, .75, .75, .1)

    return doors, scores, locs

            
    
    
    

