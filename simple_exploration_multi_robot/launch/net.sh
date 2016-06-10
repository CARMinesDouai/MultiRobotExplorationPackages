#!/bin/bash

#while true; 
#do (echo "%CPU %MEM ARGS)" && ps -A -o pcpu,pmem,args --sort=pcpu | cut -d" " -f1-5 | tail) >> ps.log; sleep 5; 
#done

#record_loop() {}
#while true; 
sudo iptraf -i wlan0 -L /home/turtle1/catkin_ws/src/simple_exploration_multi_robot/exp_new_3/exp_n06_05.txt
#done

