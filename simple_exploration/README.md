#simple_exploration

This is a ROS catkin package that contains specific launch files (and other resources) to use autonomously explore and build a map

##Prerequisites / Installation

2 catkin packages (available on github/CARMinesDouai): 
- turtlebot_car
- pose_path_publisher

1 *PhaROS* package [PhaROS install notes](http://car.mines-douai.fr/2014/06/how-to-install-pharos/)
- PhrontierExploration 
	
	```bash
	pharos create phrontier_exploration
	rosrun edit phrontier_exploration
	# then load in Pharo execute
	# Gofer it url: 'http://smalltalkhub.com/mc/CAR/PhrontierExploration/main'; package: 'Phrontier_explorationPackage'; load
	```
##How to use

### Autonomous exploration with Turtlebot

	# the following command runs phrontier_exploration
	# which publishes accessible frontiers to be explored 
	roslaunch simple_exploration simple_exploration_minimal.launch
	
	roslaunch simple_exploration simple_exploration_turtlebot.launch
	

