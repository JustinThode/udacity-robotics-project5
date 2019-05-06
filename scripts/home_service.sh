#!/bin/bash

WORKSPACE_DIR="/export/bulk/tmp/Udacity/project5/udacity"
source "$WORKSPACE_DIR/devel/setup.bash"
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$WORKSPACE_DIR/src/worlds/myworld.world " &
sleep 5
xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch map_file:=$WORKSPACE_DIR/src/map/map.yaml 3d_sensor:=r200 initial_pose_a:=4.7124" &
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers" &
sleep 2
xterm -e " rosrun pick_objects pick_objects"

