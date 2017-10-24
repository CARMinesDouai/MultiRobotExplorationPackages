# MultiRobotExplorationPackages
### Description of ROS packages
- map_merging: Greedy map merging = first wins
- multi_master_bridge : base protocol for multimaster robot communication
- multi_merge : greedy and probabilistic merging algorithm implementations using multi_master_bridge for map exchange
- pharos_probabilistic_merging: Pharos Implementation of probabilistic merging algorithm

### Dependencies
- Library SDL2 (gfx, ttf, image...)
  - sudo apt-get install libsdl2-dev libsdl2-gfx-dev 
- Library BOOST 1.58
- Some packages may require cartographer for SLAM mapping. Follows these documents for installation
  - Cartographer for [ROS here](http://google-cartographer-ros.readthedocs.io/en/latest/)
  - Cartographer for [turtlebot here](https://google-cartographer-ros-for-turtlebots.readthedocs.io/en/latest/)

### Install
Clone the repository in src/ directory
- git clone repository src

shunt pharos_probabilistic_merging package 
- > touch src/pharos_probabilistic_merging/CATKIN_IGNORE

generate messages on multi_master_bridge and torob_msgs:
- > catkin_make multi_master_bridge_generate_messages
- > catkin_make torob_msgs_generate_messages

Build the ohter packages: (debug enable)
- > catkin_make -DCMAKE_BUILD_TYPE=Debug
