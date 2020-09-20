#!/usr/bin/env python

import rospy
import sys
import cv2
import math
import pickle
import message_filters
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from door_detection.srv import *
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
from door import Door
from eyes.srv import ReadBoundingBox
#########INITIALIZE GLOBAL VARIABLES#############
bridge = CvBridge()
camera_model = PinholeCameraModel()
velodyne_data = []
tf_ = None 
velodyne_data_latched = False
door_dict = {'right':[], 'left':[]}
debug = False

master_door_list = []
roihist = pickle.load(open('/home/tilyevsk/robot-slang/husky-custom/src/door_detection/src/roihist.pkl', 'rb'))
initial_pose = None
detect = False
def camera_callback(data):
    global camera_model
    if not debug:
        # get the camera info    
        camera_info = data

        # Set the camera parameters from the sensor_msgs.msg.CameraInfo message
        camera_model.fromCameraInfo(camera_info)

def velodyne_callback(data):
    global velodyne_data, velodyne_data_latched
    if not debug:
        # if your process_frame is not underway, update the velodyne data 
        if not velodyne_data_latched:
            # parse the data into points
            assert isinstance(data, PointCloud2)
            gen = pc2.read_points(data)

            # collect the points
            points = []
            for p in gen:
                points.append(p)

            # set the global variable
            velodyne_data = points

def get_map_to_base():
    now = rospy.Time.now()
    tf_.waitForTransform("/map", "/base_link", now, rospy.Duration(0.1))
    (trans_vel, rot_vel) = tf_.lookupTransform( '/base_link', '/map', now )
    # do the same for velodyne to map frame transformation 
    trans_vel_orig = trans_vel
    trans_vel = tuple(trans_vel) + ( 1,  )
    rotationMatrix_vel = tf.transformations.quaternion_matrix( rot_vel )
    # append translation to the last column of rotation matrix(4x4)
    rotationMatrix_vel[ :, 3 ] = trans_vel
    return rotationMatrix_vel, trans_vel

def get_transformations():
    #trans = [-0.02532482, 0.28742033, -0.08615289]
    trans = [-0.03019076, 0.27373338, -0.05301323]
    #rot = [0.563540225425, -0.53427658701, 0.423247410108, 0.466725371859]
    rot = [0.544521193495, -0.512626565799, 0.452427950113, 0.485818509146]
    trans = tuple(trans) + ( 1,  )
    rotationMatrix = tf.transformations.quaternion_matrix( rot )
    # append translation to the last column of rotation matrix(4x4)
    rotationMatrix[ :, 3 ] = trans
    now = rospy.Time.now()
    tf_.waitForTransform("/base_link", "/map", now, rospy.Duration(0.1))
    (trans_vel, rot_vel) = tf_.lookupTransform( '/map', '/base_link', now )
    # do the same for velodyne to map frame transformation 
    trans_vel_orig = trans_vel
    trans_vel = tuple(trans_vel) + ( 1,  )
    rotationMatrix_vel = tf.transformations.quaternion_matrix( rot_vel )
    # append translation to the last column of rotation matrix(4x4)
    rotationMatrix_vel[ :, 3 ] = trans_vel
    return rotationMatrix, rotationMatrix_vel, trans_vel

def get_door_boxes(img):
    global roihist
    
    imHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([imHSV],[0,1],roihist,[0,180,0,256],1)
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    cv2.filter2D(dst,-1,disc,dst)
    # threshold and binary AND
    ret,maskHSV = cv2.threshold(dst,50,255,0)
    #minHSV = np.array([3,46,97])
    #maxHSV = np.array([26,153,147])

    #maskHSV = cv2.inRange(imHSV, minHSV, maxHSV)
    connectivity = 8 
    # Perform the operation
    output = cv2.connectedComponentsWithStats(maskHSV, connectivity, cv2.CV_32S)
    stats = output[2]
    door_list = []
    for i in range(0,len(stats)):
        area = stats[i, cv2.CC_STAT_AREA]
        x = stats[i,cv2.CC_STAT_LEFT]
        y = stats[i,cv2.CC_STAT_TOP]
        w = stats[i,cv2.CC_STAT_WIDTH]
        h = stats[i,cv2.CC_STAT_HEIGHT]
        x1, y1, x2, y2 = x, y, x+w, y+h
        if area > args['door_box_area_min'] and area < args['door_box_area_max'] and w/h < 16/9:# and h > 300:
            #door_list.append([x,y,w,h, 0, 0, 1000])
            door_list.append(Door(x, y, w, h))
    return door_list

#TODO: rewrite check_new_door to do sorting based on object class attributes
def check_new_door(door_list, new_door):
    new_door_list = door_list[:]
    for i,door in enumerate(new_door_list):
        dist = math.sqrt((door.door_close_point[0] - new_door.door_close_point[0])**2 + (door.door_close_point[1] - new_door.door_close_point[1])**2)
        if dist < args['inter_door_dist']:
            door.update_score(0, new_door)
            return new_door_list
    new_door_list.append(new_door)
    #print new_door_list
    new_door_list.sort(key= lambda x: x.dist_from_initial_pose, reverse = False)
    
    return new_door_list

def make_marker(point, quaternion, count):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.id = count
    marker.scale.x = 0.6
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1] 
    marker.pose.position.z = point[2]
    return marker
def create_deleteall_marker():
	# create line marker
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.action = marker.DELETEALL
	return marker
def create_delete_marker(marker_id):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.action = marker.DELETE
	marker.id = marker_id
	return marker
def callback(image_sub):
    global velodyne_data, velodyne_data_latched, camera_model, master_door_list, door_dict, debug, detect, initial_pose
    #TODO: to determine the order of the doors, compute their distance relative to the original position of the robot at the start of the detecting
    if detect:
        velodyne_data_latched = True
        img = bridge.imgmsg_to_cv2(image_sub, "bgr8")
        door_list = get_door_boxes(img)
        
        img_height = img.shape[0]
        img_width  = img.shape[1]
        #print door_list
        #get the transform
        
        rotationMatrix, rotationMatrix_vel, trans_vel = get_transformations()

        if velodyne_data:
            for i in range(0, len(velodyne_data) - 1):
                try:
                    # converting to homogeneous coordinates
                    point = [velodyne_data[i][0], velodyne_data[i][1], velodyne_data[i][2], 1]
                except IndexError:
                    print("Index Error!!!!!")
                    break
                point_dist = math.sqrt(point[0]**2 + point[1]**2)
                
                if point[0] < 0.0:# or point_dist < args['door_min_dist'] or point_dist > args['door_max_dist'] or point[2] < args['door_min_height'] or point[2] > args['door_max_height']:
                    continue
                # project 3D point to 2D uv 
                rotatedPoint = rotationMatrix.dot( point )
                #print (rotatedPoint)
                uv = camera_model.project3dToPixel( rotatedPoint )
                #print (uv)
                # check that projected points are valid and inside the detected bounding box
                if uv[0] >= 0 and uv[0] <= img_width and uv[1] >= 0 and uv[1] <= img_height:
                    #objects[objects.keys()[0]]['projected_points'].append([int( uv[0] ),int( uv[1] )])
                    # loop over the objects
                    for j in range(0,len(door_list)):
                        x,y,w,h = door_list[j].box()
                        x1, y1, x2, y2 = x, y, x+w, y+h
                        if uv[0] >= x1 and uv[0] <= x2 and uv[1] >= y1 and uv[1] <= y2:
                            door_list[j].update(point)
                            cv2.line(img,(int( uv[0] ),int( uv[1] )),(int( uv[0] )+2,int( uv[1] ) +2),(255,0,0),3)
        for k in range(0,len(door_list)):
            door_goal = door_list[k].compute_goal(rotationMatrix_vel, trans_vel, initial_pose)
            x, y, w, h = door_list[k].box()
            cv2.rectangle(img,(x,y) ,(x+w, y+h),(0,0,255),6)
            if not door_goal:
                continue
            cur_door = door_list[k]
            local_door_list = check_new_door(master_door_list, cur_door)
            master_door_list = local_door_list
            
            
            #cv2.putText(img, "{:0.2f}".format(x_vel) + "," + "{:0.2f}".format(y_vel), (int(x+w/2), int(y+h/2)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0),2)
        #print ("Number of doors:%d"%len(master_door_list))
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        velodyne_data_latched = False

def request(data):
    global door_dict, detect, initial_pose, master_door_list
    if debug:
        door_dict = {'right':[1,2,[3,4]], 'left':[5,6,[7,8]]}
 
    if data.direction == 'detect':
        detect = True
        door_dict = {'right':[], 'left':[]}
        master_door_list = []
        rotationMatrix, rotationMatrix_vel, trans_vel = get_transformations()
        initial_pose = (trans_vel, rotationMatrix_vel)
        print (trans_vel)
        return doorCoordsResponse(0,0,0,0,0,0, False, 'none', 0, 0, 0, 0)
    if data.direction == 'stop_detect':
        detect = False
        return doorCoordsResponse(0,0,0,0,0,0, False, 'none', 0, 0, 0, 0)
    if data.direction == 'resume_detect':
        detect = True
        return doorCoordsResponse(0,0,0,0,0,0, False, 'none', 0, 0, 0, 0)
    #if door direction is not given or unknown, return closest door available
    print data.direction
    if data.direction == 'unknown':
        print ("Providing closest door")
        doors = [x for x in master_door_list if x.score > args['door_score']]
        print (data.direction, [x.goal for x in doors], [x.score for x in doors])
        door = doors[data.number] if len(doors) > 0 else None
        #door_options.extend(door_option)
        if door:
            rotationMatrix, rotationMatrix_vel, trans_vel = get_transformations()
            theta_angles = door.recompute_goal(initial_pose, trans_vel)
            #print (theta_angles)
            print (door.direction, door.goal, door.score)
            return doorCoordsResponse(door.goal_close[0], door.goal_close[1], door.goal_far[0], door.goal_far[1], door.goal_close[2][2], door.goal_close[2][3], True, door.direction, door.door_close_point[0], door.door_close_point[1], door.door_far_point[0], door.door_far_point[1])
        else:
            return doorCoordsResponse(0,0,0,0,0,0, False, 'none', 0, 0, 0, 0)
    if data.direction in ['right', 'left']:
        #detect = False
        print ("Providing door number %d"%(data.number))
        print ([(x.goal_close, x.score) for x in master_door_list if x.direction == data.direction and x.score > args['door_score'] ])
        for door in master_door_list:
            rotationMatrix, rotationMatrix_vel, trans_vel = get_transformations()
            theta_angles = door.recompute_goal(initial_pose, trans_vel)
            print (theta_angles)
       # print ("after", [x.goal for x in master_door_list], [x.score for x in master_door_list], [x.direction for x in master_door_list])
        doors = [x for x in master_door_list if x.direction == data.direction and x.score > args['door_score']]
        #
        #if no doors available, return that goal is invalid
        if len(doors) == 0 or data.number > len(doors) - 1:
            print ("door not found")
            return doorCoordsResponse(0,0,0,0,0,0, False, 'none', 0, 0, 0, 0)
        door = doors[data.number]
        #door.recompute_goal(initial_pose)
        
        return doorCoordsResponse(door.goal_close[0], door.goal_close[1], door.goal_far[0], door.goal_far[1], door.goal_close[2][2], door.goal_close[2][3], True, door.direction, door.door_close_point[0], door.door_close_point[1], door.door_far_point[0], door.door_far_point[1])


def text_request(data):
    if data.direction == 'unknown':
        detect = False
        print ("Reading text")
        doors = [x for x in master_door_list if x.score > args['door_score']]
        #print (data.direction, [x.goal for x in doors], [x.score for x in doors])
        #rot_base,trans_base = get_map_to_base()
        #rotationMatrix, rotationMatrix_vel, trans_vel = get_transformations()
        for door in doors:
            door.x_list.sort()
            door.y_list.sort()
            door.h_list.sort()
            door.w_list.sort()
            x1,y1,w,h = door.x_list[len(door.x_list)/2],door.y_list[len(door.y_list)/2],door.w_list[len(door.w_list)/2],door.h_list[len(door.h_list)/2]
            #theta_angles = door.recompute_goal(initial_pose, trans_vel)
            #point = [door.door_close_point[0], door.door_close_poiint[1], 1.5, 1]
            #rotatedPoint = rotationMatrix.dot( point )
            #uv = camera_model.project3dToPixel( rotatedPoint )
            #x1,y1 = uv[0], uv[1]
            #point = [door.door_far_point[0], door.door_far_poiint[1], .5, 1]
            #rotatedPoint = rotationMatrix.dot( point )
            #uv = camera_model.project3dToPixel( rotatedPoint )
            #x2,y2 = uv[0], uv[1]
            text_detection = rospy.ServiceProxy('/eyes/read_bounding_box', ReadBoundingBox)
            text_request = text_detection(x1,y1,x1+w,y1+h,1,1)
            print(text_request.texts) 
        detect = True
'''        #door_options.extend(door_option)
    if data.direction in ['right', 'left']:
        #detect = False
        print ("Providing door number %d"%(data.number))
        #print ([(x.goal_close, x.score) for x in master_door_list if x.direction == data.direction and x.score > args['door_score'] ])
        doors = [x for x in master_door_list if x.direction == data.direction and x.score > args['door_score']]
        #
        #if no doors available, return that goal is invalid
        door = doors[data.number]
        #door.recompute_goal(initial_pose)
'''
if __name__=='__main__':
    #TODO subscribe to occupancy grid to check if goal is valid i.e. not obstructed
    args = {}
    args['door_box_area_min']	          = rospy.get_param('door_detection/door_box_area_min')
    args['door_box_area_max']	          = rospy.get_param('door_detection/door_box_area_max')
    args['inter_door_dist']               = rospy.get_param('door_detection/inter_door_dist')
    args['door_min_dist']                 = rospy.get_param('door_detection/door_min_dist')
    args['door_max_dist']                 = rospy.get_param('door_detection/door_max_dist')
    args['door_min_height']               = rospy.get_param('door_detection/door_min_height')
    args['door_max_height']               = rospy.get_param('door_detection/door_max_height')
    args['door_score']                    = rospy.get_param('door_detection/door_score')
    rospy.init_node("door_detector")     
    tf_ = tf.TransformListener() 
    image_pub = rospy.Publisher("/doors", Image, queue_size=1)
    image_sub = rospy.Subscriber("/axis/image_raw_out", Image, queue_size=1,callback = callback)
    s1 = rospy.Service('door_service', doorCoords, request)
    s2 = rospy.Service('text_sweep', doorText, text_request)
    rospy.Subscriber("/axis/camera_info", CameraInfo, callback=camera_callback)
    velodynePoint = rospy.Subscriber('/velodyne_points', PointCloud2, callback = velodyne_callback)
    #doorMarker = rospy.Publisher('/door_markers', MarkerArray, queue_size = 1)
    rospy.spin()
    # Use cv_bridge() to convert the ROS image to OpenCV format



    
