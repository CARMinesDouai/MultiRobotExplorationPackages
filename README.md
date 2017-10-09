# MultiRobotExplorationPackages
### Description of ROS packages
- map_merging: Greedy map merging = first wins
- multi_master_bridge : base protocol for multimaster robot communication
- multi_merge : greedy and probabilistic merging algorithm implementations using multi_master_bridge for map exchange
- pharos_probabilistic_merging: Pharos Implementation of probabilistic merging algorithm

### Dependancy
- Library SDL2 (gfx, ttf, image...)
- Library BOOST 1.58

### Install
Clone the repository in src/ directory
- git clone repository src

shunt pharos_probabilistic_merging package 
- > echo src/pharos_probabilistic_merging/CATKIN_IGNORE

generate messages on multi_master_bridge and torob_msgs:
- > catkin_make multi_master_bridge_generate_messages
- > catkin_make torob_msgs_generate_messages

Build the ohter packages:
- > catkin_make -DCMAKE_BUILD_TYPE=Debug