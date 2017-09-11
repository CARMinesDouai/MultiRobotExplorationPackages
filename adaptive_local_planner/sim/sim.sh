#!/bin/sh
ws=/home/mrsang/ros_ws
export TURTLEBOT_STAGE_MAP_FILE=$ws/src/adaptive_local_planner/sim/diamap.yaml
export TURTLEBOT_STAGE_WORLD_FILE=$ws/src/adaptive_local_planner/sim/diamap.world
roslaunch adaptive_local_planner turtlebot_in_stage.launch 
