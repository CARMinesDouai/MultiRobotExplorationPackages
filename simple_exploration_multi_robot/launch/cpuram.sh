#!/bin/bash

#while true; 
#do (echo "%CPU %MEM ARGS)" && ps -A -o pcpu,pmem,args --sort=pcpu | cut -d" " -f1-5 | tail) >> ps.log; sleep 5; 
#done

#record_loop() {}
pross="$1"
#while true; #==============================================================================================================
#do ps --no-headers -u $USER -o pcpu,rss | awk '{cpu += $1; rss += $2} END {print cpu, rss}' >> /home/turtle1/catkin_ws/src/simple_exploration_multi_robot/exp_new_3/cpuexp_onepro_18_05.log; sleep 5;
#do ps --no-headers -u $USER -o pcpu,rss | awk '{cpu += $1; rss += $2} END {print cpu, rss}' >> /home/turtle1/catkin_ws/src/simple_exploration_multi_robot/exp_new_3/cpuexp_onepro_18_05.log; sleep 5;
#done #=====================================================================================================================

while true;
do ps -p $pross -o pcpu,pmem,args --sort=pcpu | cut -d" " -f1-5 | tail  >> /home/turtle1/catkin_ws/src/simple_exploration_multi_robot/exp_new_3/cpuexp_onepro_18_05.log; sleep 5;
done
