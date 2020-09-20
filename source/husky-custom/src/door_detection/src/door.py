#!/usr/bin/env python
import math
import numpy as np
import tf

#TODO: make close goal and far goal so robot can make two attempts to catch the door tag
class Door:
    def __init__(self, x, y, w, h, base_x_close = None, base_y_close = None,base_z_close = None, close_dist = float('Inf'), base_x_far = None, base_y_far = None,base_z_far = None, far_dist = 0.0, goal=None, direction = None, dist_from_initial_pose = None, goal_close = None, goal_far = None, score = 1, door_close_point = None, door_far_point = None):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.x_list = [x]
        self.y_list = [y]
        self.w_list = [w]
        self.h_list = [h]
        self.base_x_close = base_x_close
        self.base_y_close = base_y_close
        self.base_z_close = base_z_close
        self.close_dist = close_dist
        self.base_x_far = base_x_far
        self.base_y_far = base_y_far
        self.base_z_far = base_z_far
        self.far_dist = far_dist
        self.points = []
        self.goal = goal
        self.goal_close = goal_close
        self.goal_far = goal_far
        self.goal_close_list = []
        self.goal_far_list = []
        self.direction = direction
        self.dist_from_initial_pose = dist_from_initial_pose
        self.score = score
        self.door_close_point = door_close_point
        self.door_far_point = door_far_point 
        self.door_close_list = []
        self.door_far_list = []
    def box(self):
        return self.x, self.y, self.w, self.h
    def update_score(self, val, door):
        self.score = self.score + val + 1
        self.x_list.extend(door.x_list)
        self.y_list.extend(door.y_list)
        self.h_list.extend(door.h_list)
        self.h_list.extend(door.w_list)
        self.goal_close_list.extend(door.goal_close_list)
        self.goal_far_list.extend(door.goal_far_list)
        self.door_close_list.extend(door.door_close_list)
        self.door_far_list.extend(door.door_far_list)
    def update(self,point):
        dist = math.sqrt( np.sum( np.array(point[:2] ) ** 2 ) )
        self.points.append(point)
        if dist < self.close_dist:
            self.close_dist = dist
            self.base_x_close = point[0]
            self.base_y_close = point[1]
            self.base_z_close = point[2]
        elif dist > self.far_dist:
            self.far_dist = dist
            self.base_x_far = point[0]
            self.base_y_far = point[1]
            self.base_z_far = point[2]
    def recompute_goal(self, initial_pose, trans_vel): #take median coordinates of all collected data points that correspond to same door
        dist_factor = .9
        #self.goal_close_list.sort(key = lambda x:x[0], reverse = False)
        #self.goal_close[0] = self.goal_close_list[len(self.goal_close_list)/2][0]
        #self.goal_close_list.sort(key = lambda x:x[1], reverse = False)
        #self.goal_close[1] = self.goal_close_list[len(self.goal_close_list)/2][1]
        #self.goal_far_list.sort(key = lambda x:x[0], reverse = False)
        #self.goal_far[0] = self.goal_far_list[len(self.goal_far_list)/2][0]
        #self.goal_far_list.sort(key = lambda x:x[1], reverse = False)
        #self.goal_far[1] = self.goal_far_list[len(self.goal_far_list)/2][1]

        self.door_close_list.sort(key = lambda x:x[0], reverse = False)
        self.door_close_point[0] = self.door_close_list[len(self.door_close_list)/2][0]
        self.door_close_list.sort(key = lambda x:x[1], reverse = False)
        self.door_close_point[1] = self.door_close_list[len(self.door_close_list)/2][1]
        self.door_far_list.sort(key = lambda x:x[0], reverse = False)
        self.door_far_point[0] = self.door_far_list[len(self.door_far_list)/2][0]
        self.door_far_list.sort(key = lambda x:x[1], reverse = False)
        self.door_far_point[1] = self.door_far_list[len(self.door_far_list)/2][1]
      
        x_dist = self.door_close_point[0] - initial_pose[0][0]
        y_dist = self.door_close_point[1] - initial_pose[0][1]
        x_dist_to_initial = trans_vel[0] - initial_pose[0][0]
        y_dist_to_initial = trans_vel[1] - initial_pose[0][1]
        theta = math.atan2(y_dist, x_dist)
        theta_to_initial = math.atan2(y_dist_to_initial, x_dist_to_initial)
        theta_diff = theta - theta_to_initial
        #print (theta, theta_to_initial, initial_pose[0])
        
        door_theta = math.atan2(self.door_far_point[1] - self.door_close_point[1], self.door_far_point[0] - self.door_close_point[0])
        door_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, door_theta)
        #self.goal_close[2] = door_quaternion
        #self.goal_far[2] = door_quaternion
        if theta_diff > 0 and theta_diff < math.pi:
             self.direction = 'left'
             self.goal_close = [self.door_close_point[0] + dist_factor*math.cos(door_theta-math.pi/2), self.door_close_point[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
             self.goal_far = [self.door_far_point[0] + dist_factor*math.cos(door_theta-math.pi/2), self.door_far_point[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        else:
             self.direction = 'right'
             self.goal_close = [self.door_close_point[0] + dist_factor*math.cos(door_theta+math.pi/2), self.door_close_point[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
             self.goal_far = [self.door_far_point[0] + dist_factor*math.cos(door_theta+math.pi/2), self.door_far_point[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        return self.goal_close
    def compute_goal(self,rotationMatrix_vel, trans_vel, initial_pose):
        dist_factor = 1.1
        if not self.base_x_close or not self.base_x_far:
            return False
        #print (len(self.points))
        #print ([self.base_x_close, self.base_y_close, self.base_z_close, 1])
        #print ([self.base_x_close, self.base_y_close, self.base_z_close, 1])
        close_point = rotationMatrix_vel.dot([self.base_x_close, self.base_y_close, self.base_z_close, 1])
        far_point = rotationMatrix_vel.dot([self.base_x_far, self.base_y_far, self.base_z_far, 1])
        self.door_close_list.append(close_point)
        self.door_far_list.append(far_point)
        self.door_close_point = close_point
        self.door_far_point = far_point
        x_dist = self.door_close_point[0] - initial_pose[0][0]
        y_dist = self.door_close_point[1] - initial_pose[0][1]
        x_dist_to_initial = trans_vel[0] - initial_pose[0][0]
        y_dist_to_initial = trans_vel[1] - initial_pose[0][1]
        self.dist_from_initial_pose = math.sqrt(x_dist**2 + y_dist**2)
        theta = math.atan2(y_dist, x_dist)
        theta_to_initial = math.atan2(y_dist_to_initial, x_dist_to_initial)
        quaternion = tf.transformations.quaternion_from_euler(0.0,0.0, theta)
        #print (x_dist,y_dist, theta)
        x_cent = float((far_point[0] + close_point[0])/2)
        y_cent = float((far_point[1] + close_point[1])/2)
        #TODO: use initial_quaternion instead of quaternion to assume that robot is aligned with the hallway
        initial_quaternion = initial_pose[1]
        #print (initial_quaternion)
        initial_theta = tf.transformations.euler_from_matrix(initial_quaternion)
        #TODO: clean this up to use direction to determine theta
	#if theta_to_initial < initial_theta[2]:
        #    self.direction = 'right'
        #else:
        #    self.direction = 'left'
        #theta_diff = initial_theta[2] - theta_to_initial
        #compute difference between angle of door and angle to door
        theta_diff = theta - theta_to_initial
        door_theta = math.atan2(self.door_far_point[1] - self.door_close_point[1], self.door_far_point[0] - self.door_close_point[0])
        door_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, door_theta)
        #self.goal_close[2] = door_quaternion
        #self.goal_far[2] = door_quaternion
        if theta_diff > 0 and theta_diff < math.pi:
             self.direction = 'left'
             self.goal_close = [self.door_close_point[0] + dist_factor*math.cos(door_theta-math.pi/2), self.door_close_point[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
             self.goal_far = [self.door_far_point[0] + dist_factor*math.cos(door_theta-math.pi/2), self.door_far_point[1] + dist_factor*math.sin(door_theta-math.pi/2), door_quaternion]
        else:
             self.direction = 'right'
             self.goal_close = [self.door_close_point[0] + dist_factor*math.cos(door_theta+math.pi/2), self.door_close_point[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
             self.goal_far = [self.door_far_point[0] + dist_factor*math.cos(door_theta+math.pi/2), self.door_far_point[1] + dist_factor*math.sin(door_theta+math.pi/2), door_quaternion]
        return True
        '''
        if theta_diff > 0 and theta_diff < math.pi:
             self.direction = 'right'
        else:
             self.direction = 'left'
        
        theta_perp1 = theta + math.pi/2
        theta_perp2 = theta - math.pi/2
        goal1 = [x_cent + dist_factor*math.cos(theta_perp1),y_cent + dist_factor*math.sin(theta_perp1), theta]
        goal2 = [x_cent + dist_factor*math.cos(theta_perp2),y_cent + dist_factor*math.sin(theta_perp2), theta]
        goal1_dist = math.sqrt((goal1[0] - trans_vel[0])**2 + (goal1[1] - trans_vel[1])**2)
        goal2_dist = math.sqrt((goal2[0] - trans_vel[0])**2 + (goal2[1] - trans_vel[1])**2)
        #print [x_cent, y_cent, theta]
        #return [x_cent, y_cent, quaternion]
        #TODO: check if commenting out below works. This means it will use angle of the door 
        #quaternion = tf.transformations.quaternion_from_euler(0.0,0.0, initial_theta[2])
        if goal1_dist < goal2_dist:
            self.goal = [goal1[0], goal1[1], quaternion]
            self.goal_close = [close_point[0] + dist_factor*math.cos(theta_perp1), close_point[1] + dist_factor*math.sin(theta_perp1), quaternion]
            self.goal_far = [far_point[0] + dist_factor*math.cos(theta_perp1), far_point[1] + dist_factor*math.sin(theta_perp1), quaternion]
            self.goal_close_list.append(self.goal_close)
            self.goal_far_list.append(self.goal_far)
            self.dist_from_initial_pose = math.sqrt((goal1[0] - initial_pose[0][0])**2 + (goal1[1] - initial_pose[0][1])**2)
            #return [goal1[0], goal1[1], quaternion, self.direction]
            return True
        else:
            self.goal = [goal2[0], goal2[1], quaternion]
            self.goal_close = [close_point[0] + dist_factor*math.cos(theta_perp2), close_point[1] + dist_factor*math.sin(theta_perp2), quaternion]
            self.goal_far = [far_point[0] + dist_factor*math.cos(theta_perp2), far_point[1] + dist_factor*math.sin(theta_perp2), quaternion]
            self.goal_close_list.append(self.goal_close)
            self.goal_far_list.append(self.goal_far)
            self.dist_from_initial_pose = math.sqrt((goal2[0] - initial_pose[0][0])**2 + (goal2[1] - initial_pose[0][1])**2)
            return True
        '''
        
