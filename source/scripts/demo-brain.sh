#!/bin/bash

# check if input argument passed in
if [[ $# -eq 0 ]] ; then
	#output_dir=/media/$USER/ExtraDrive1/trial-unknown
	output_dir=$HOME/trial-unknown
else
	#output_dir=/media/$USER/ExtraDrive1/$1
	output_dir=$HOME/$1
fi

if [ ! -d $output_dir ] 
then
	mkdir -p $output_dir
fi

echo 'Saving logs to' $output_dir

start_cartographer () {
	output_dir=$1
	command='roslaunch cartographer_ros hosh_robot_2d.launch'
	cd $HOME/robot-slang/workspaces/cartographer-ws
	source devel_isolated/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/cartographer_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/cartographer_log.txt $command &
	#watch -n1 tail -8 $output_dir/cartographer_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_move_base () {
	output_dir=$1
	command='roslaunch husky_navigation move_base.launch'
	cd $HOME/robot-slang/husky-env
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/move_base_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/move_base_log.txt $command &
	#watch -n1 tail -8 $output_dir/move_base_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_approach_person () {
	output_dir=$1
	command='roslaunch approach_person start_approach_person.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/approach_person_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/approach_person_log.txt $command &
	#watch -n1 tail -8 $output_dir/approach_person_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_intersection_mapper () {
	output_dir=$1
	source $HOME/robot-slang/husky-custom/devel/setup.bash
	source $HOME/robot-slang/workspaces/cartographer-ws/devel_isolated/setup.bash --extend
	command='roslaunch intersection_mapper start_intersection_mapper2.launch'
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/intersection_mapper_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/intersection_mapper_log.txt $command &
	#watch -n1 tail -8 $output_dir/intersection_mapper_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_follow_directions () {
	output_dir=$1
	command='roslaunch follow_directions start_follow_directions.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/follow_directions_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/follow_directions_log.txt $command &
	#watch -n1 tail -8 $output_dir/follow_directions_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_mouth_and_ears () {
	output_dir=$1
	command='roslaunch mouth_and_ears start_mouth_and_ears.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	export GOOGLE_APPLICATION_CREDENTIALS=$HOME/robot-slang/husky-custom/src/dialog/src/Dialog.json
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/mouth_and_ears_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/mouth_and_ears_log.txt $command &
	#watch -n1 tail -8 $output_dir/mouth_and_ears_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_wander () {
	output_dir=$1
	command='roslaunch wander start_wander.launch'
	cd $HOME/robot-slang/husky-custom
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/wander_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/wander_log.txt $command &
	#watch -n1 tail -8 $output_dir/wander_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_hold_conversation () {
	output_dir=$1
	command='roslaunch hold_conversation start_hold_conversation.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/hold_conversation_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/hold_conversation_log.txt $command &
	#watch -n1 tail -8 $output_dir/hold_conversation_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_command_listener () {
	output_dir=$1
	command='roslaunch command_listener start_command_listener.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/command_listener_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/command_listener_log.txt $command &
	#watch -n1 tail -8 $output_dir/command_listener_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_brain () {
	output_dir=$1
	command='roslaunch brain start_brain.launch'
	cd $HOME/robot-slang/husky-custom/
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/brain_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/brain_log.txt $command &
	#watch -n1 tail -8 $output_dir/brain_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_husky_viz () {
	output_dir=$1
	command='roslaunch husky_viz view_robot.launch'
	cd $HOME/robot-slang/husky-env
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/husky_viz_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/husky_viz_log.txt $command &
	#watch -n1 tail -8 $output_dir/husky_viz_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_door_navigation () {
	output_dir=$1
	command='python door_navigatev4.py'
	cd $HOME/robot-slang/husky-custom/src/door_navigation/src
	export GOOGLE_APPLICATION_CREDENTIALS=$HOME/robot-slang/husky-custom/src/dialog/src/Dialog.json
	source $HOME/robot-slang/husky-custom/devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/door_navigation_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/door_navigation_log.txt $command &
	#watch -n1 tail -f $output_dir/door_navigation_log.txt
	echo $command
	$SHELL
}

start_axis() {
	output_dir=$1
	command='roslaunch my_axis_camera axis.launch'
	cd $HOME/robot-slang/husky-custom
	source devel/setup.bash
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/my_axis_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/my_axis_log.txt $command &
	#watch -n1 tail -8 $output_dir/my_axis_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_narration () {
	output_dir=$1
	cd $HOME/robot-slang/husky-custom/
	command='python src/narration/src/run_narration.py'
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/narration_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/narration_log.txt $command &
	#watch -n1 tail -8 $output_dir/narration_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_publishing_path () {
	output_dir=$1
	source $HOME/robot-slang/husky-custom/devel/setup.bash
	source $HOME/robot-slang/workspaces/cartographer-ws/devel_isolated/setup.bash
	cd $HOME/robot-slang/husky-custom/
	command='python src/position_markers/src/publish_robot_path2.py'
	#$command
	/home/jjohanse/bin/unbuff $command | tee $output_dir/robot_path_log.txt
	#/home/jjohanse/bin/unbuff > $output_dir/robot_path_log.txt $command &
	#watch -n1 tail -8 $output_dir/robot_path_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_debug1 () {
	output_dir=$1
	command='iwevent'
	/home/jjohanse/bin/unbuff $command | tee iwevent_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_debug2 () {
	output_dir=$1
	command='ping 128.46.214.183'
	/home/jjohanse/bin/unbuff $command | tee $output_dir/debug2_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_keep_alive () {
	output_dir=$1
	command='bash keep-network-alive.sh'
	/home/jjohanse/bin/unbuff $command | tee $output_dir/keep_network_alive_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

start_rosbag_record () {
	output_dir=$1
	#trap "rosnode kill /bagger" SIGTERM
	#trap "rosnode kill /bagger" SIGINT
	source $HOME/robot-slang/husky-custom/devel/setup.bash
	source $HOME/robot-slang/workspaces/cartographer-ws/devel_isolated/setup.bash
	trialname=`basename "$output_dir"`
	cd /media/$USER/ExtraDrive1/
	command="rosbag record -O ${trialname}.bag -a -x '/axis/image_raw|/axis/image_raw_out|/axis/image_raw/compressed|/axis/image_raw_out/theora|/door_navigation/detections|/approach_person/tracks|/approach_person/location|/velodyne_points|/velodyne_packets|/scan_matched_points2|'"
	$command
	#/home/jjohanse/bin/unbuff $command | tee $output_dir/rosbag_record_log.txt
	echo $command
	$SHELL # keep the terminal open after the previous commands are executed
}

#!/bin/bash
export -f start_cartographer
export -f start_move_base
export -f start_approach_person
export -f start_intersection_mapper
export -f start_follow_directions
export -f start_mouth_and_ears
export -f start_wander
export -f start_hold_conversation
export -f start_command_listener
export -f start_brain
export -f start_husky_viz
export -f start_door_navigation
export -f start_axis
export -f start_narration
export -f start_publishing_path
export -f start_debug1
export -f start_debug2
export -f start_keep_alive
export -f start_rosbag_record

# geometry = width (in characters) x height (in lines) + x-offset + y-offset
# hosh fits ~60 lines (height-wise) and ~210 columns (width-wise) 
# hosh is 1080 pixels tall x 1920 pixels wide
# that means one line is ~18 pixels (height-wise) and one column is ~9 pixels
#gnome-terminal --geometry=80x15+800+0    -e "bash -c 'trap \"rosnode kill /bagger\" SIGINT; start_rosbag_record $output_dir'"
#gnome-terminal --geometry=80x15+800+0    -e "bash rosbag-record-with-trap.sh $output_dir"
gnome-terminal --geometry=80x15+800+0    -e "bash -c 'start_rosbag_record $output_dir'"

gnome-terminal --geometry=80x15+800+310	 -e "bash -c 'start_command_listener $output_dir'"
gnome-terminal --geometry=80x13+800+860	 -e "bash -c 'start_publishing_path $output_dir'"

gnome-terminal --geometry=80x12+1000+000 -e "bash -c 'start_debug1 $output_dir'"
gnome-terminal --geometry=80x12+1000+200 -e "bash -c 'start_debug2 $output_dir'"
gnome-terminal --geometry=80x12+1000+400 -e "bash -c 'start_cartographer $output_dir'"
gnome-terminal --geometry=80x12+1000+600 -e "bash -c 'start_axis $output_dir'"
gnome-terminal --geometry=80x20+1000+800 -e "bash -c 'start_move_base $output_dir'"

gnome-terminal --geometry=100x14+260+000 -e "bash -c 'start_intersection_mapper $output_dir'"
gnome-terminal --geometry=100x10+260+280 -e "bash -c 'start_wander $output_dir'"
gnome-terminal --geometry=100x10+260+500 -e "bash -c 'start_approach_person $output_dir'"
gnome-terminal --geometry=100x10+260+680 -e "bash -c 'start_hold_conversation $output_dir'"
gnome-terminal --geometry=100x10+260+910 -e "bash -c 'start_follow_directions $output_dir'"

gnome-terminal --geometry=80x15+0+000    -e "bash -c 'start_mouth_and_ears $output_dir'"
gnome-terminal --geometry=80x15+0+300    -e "bash -c 'start_keep_alive $output_dir'"
gnome-terminal --geometry=80x12+0+590    -e "bash -c 'start_brain $output_dir'"
gnome-terminal --geometry=80x13+0+860    -e "bash -c 'start_door_navigation $output_dir'"

gnome-terminal --geometry=80x12+1000+500 -e "bash -c 'start_husky_viz $output_dir'"
sleep 4
gnome-terminal --geometry=50x50+0+0 --zoom=1.5 -e "bash -c 'start_narration $output_dir'"

# create an empty narration file
#rm $HOME/narration.txt
#touch $HOME/narration.txt
#gnome-terminal --geometry=50x50+0+0 --zoom=1.5 -e '#watch -n0.1 tail ~/narration.txt'

