-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- default: 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- default: 0.025

----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------

-- https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
-- To tune cartographer, we can do the following: 
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1 -- default: 1
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 -- default: 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1 -- default: 40
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.use_nonmonotonic_steps = false -- default: false
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.max_num_iterations = 20 -- default: 20
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.num_threads = 4 -- default: 1

--POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5 -- default: 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 0 -- default: 1e5
--POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5 -- default: 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0 -- default: 1e5

----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------
-- https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
-- To reduce global SLAM latency, we can do the following:
-- decrease optimize_every_n_nodes
POSE_GRAPH.optimize_every_n_nodes = 1 -- default = 90
-- increase MAP_BUILDER.num_background_threads up to the number of cores
MAP_BUILDER.num_background_threads = 12 -- default = 4
-- decrease global_sampling_ratio
--POSE_GRAPH.global_sampling_ratio = 0.003 -- default = 0.003
-- decrease constraint_builder.sampling_ratio
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 -- default = 0.3
-- increase constraint_builder.min_score
--POSE_GRAPH.constraint_builder.min_score = 0.55 -- default = 0.55
-- for the adaptive voxel filter(s), decrease .min_num_points, .max_range, increase .max_length
-- increase voxel_filter_size, submaps.resolution, decrease submaps.num_range_data
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30 -- default: 90
	--> using 90 causes a few "big jumps" from time to time.
	--> changing this from 30 -> 10 causes the local map to jump around a lot more (in small amounts, but it is still jumpy).
	--> 30 seems to be a decent compromise on quality
-- decrease search windows sizes, .linear_xy_search_window, .linear_z_search_window, .angular_search_window
-- increase global_constraint_search_after_n_seconds
--POSE_GRAPH.global_constraint_search_after_n_seconds = 10. -- default = 10
-- decrease max_num_iterations

----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------

-- min_z, max_z, min_range, and max_range refer to the pointcloud points.
-- we only keep points that are within the "min_z < pointcloud_point_height < max_z" range
-- here is two tables with some guidance on which values to select for min_z and max_z

-- min_z
-- three objects are placed 7 feet from LIDAR.  LIDAR is 26 inches off of ground.
-- with various min_z settings, are those objects detected/included in the map cartographer makes?
--			object height off ground
-- min_z 	floor	3.5" 	5.5"	9"
-- 	-0.2	YES		YES		YES 	YES
--	-0.15	faintly	NO		YES		YES	
--	-0.1	faintly	NO		NO		YES
--	-0.05	faintly	NO		NO		YES
--	 0.0	NO		NO		NO		NO
-- 	 0.1 	NO		NO		NO		NO
--	* The YES/NO/faintly classifications for floor/3.5" appear to contradict themselves.  
--	I think the discrepancies arises from how many rays at that particular angle hit the obstacle 
--	and how many hit the far wall.  When it hits the obstacle, it puts an obstacle in the map.
--	When it hits the far wall, it erases all obstacles between the two.  Thus, if there are LIDAR
--	points from the same angle (just at different heights) that contradict each other, you get
--	competing classifications of that space.

-- max_z
-- ceiling is 8 feet tall.  exit sign is 7 feet tall and 20 feet away.
-- LIDAR is 26 inches off of ground.
-- with various max_z settings, are those objects detected/included in the map cartographer makes?
--	max_z	exit	ceiling
--	2.5		YES		YES
--	2.3		YES		YES
--	2.2		YES		YES
--	2.1		YES		faintly
--	2.0		YES		faintly
--	1.9		NO		NO
-- 	1.7		NO		NO
--	1.5		NO		NO
TRAJECTORY_BUILDER_2D.min_z = 0.1 -- default: -0.8
TRAJECTORY_BUILDER_2D.max_z = 1.5 -- default: 2.0
-- we only keep points, whose distance are within the "min_range < pointcloud_point_distance < max_range" range.
-- all pointcloud points that fall outside of these ranges are discarded.
-- if min_range is < 1.1, it detects the support beams at the back of the robot as obstacles
TRAJECTORY_BUILDER_2D.min_range = 1.1 -- default: 0.0
-- max_range is similar in function to obstacle_range (if you prefer to think of it that way).  Any obstacle that it
-- detects that is less than max_range (and bigger than min_range) will be added to the map.
TRAJECTORY_BUILDER_2D.max_range = 50.0 -- default: 30.0
	--> Changing max_range to 30 meant that for long hallways, the space immediately in front of the robot is unknown
	--	space.  This would prevent the directionGenerator from finding forward paths.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.0 -- default: 5.0



return options



-- 2020.02.14 -- Jared comments
-- Let me leave some comments here to remind myself what I've discovered over the last couple of days with 
-- attempting to tune cartogapher.
-- The local SLAM works well with the current settings.  It's fantastic.  It's fast and accurate.
-- The global SLAM is now fast (on par with local slam).  That means that the global map doesn't shift underneath
--	us very much when it does shift.  The exception to this is when loop closure occurs.  That is the one time when 
--	the map can shift by a few meters, leaving (previosuly detected) objects floating in unknown space...or now 
--	sitting on the opposite side of the hallway. 
-- This behavior is a problem for anyone that saves off a position in the map in an online fashion.  The [x,y] 
-- 	location will be the same at some distant future point in time...but the map will have shifted beneath them.
--	I haven't found a way to get information from Cartographer about how the map shifts.
--	It's tricky because different portions of the map shift by different amounts.
--	In some places, no shifting occurs.  In other places, significant shifting occurs.
--	There would have to be some mapping between a grid cell at time t and where that grid cell has shifted to at 
--	time t'.
--	I don't think cartographer publishes anything like that.  
--	They may have the ability to create that mapping deep in the bowels of cartographer, but it would be a lot of 
--	work to discover whether such an ability exists or not...and more work to produce it.
-- 	Last I checked, cartographer had over 120 source code files.
-- For the time being, some observations:
-- 	POSE_GRAPH.optimize_every_n_nodes = 1 is the biggest impact
--	It runs global slam more frequently.  The default is 90.
--	So, global slam would run infrequently.  When it would run and determine some change in SLAM were needed, it 
--	would shift the whole map.
--	Now that shifting happens in lock step with the local SLAM, so it doesn't have the large changes in position.
--	num_range_data affects the size of your submaps.  Too few (10) means that fewer samples are used to create a 
--	submap and it is a little more noisy.  So there is more jitter in your global map.  Too many (90) creates larger 
--	submaps with more substantial shifts.  I found 30 works decently well.
--	Depressing how much weight rotation plays in the maps has seemed to help.
--	Also, tightening the laser range scan also seems to have helped somewhat.  Tightening too much (e.g. 30) meant 
-- 	that for long hallways (like in physics), the space immediately in front of the robot is unknown
--	space.  This would prevent the directionGenerator from finding forward paths.
--	num_accumulated_range_data means that multiple scans are averaged to get a better estimate of the environment.  
--	The higher this number, the better the estimates and better the map.  The only problem is you get fewer samples 
--	to update your map...so your map turns from gray to white a LOT slower.
--	For my tests, the driving speed was 0.4 and rotation was 0.2.  Cartographer performed well at those speeds.





