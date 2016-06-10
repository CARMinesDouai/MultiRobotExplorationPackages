#!/bin/bash

#while true; 
#do (echo "%CPU %MEM ARGS)" && ps -A -o pcpu,pmem,args --sort=pcpu | cut -d" " -f1-5 | tail) >> ps.log; sleep 5; 
#done

#record_loop() {}
#do iperf -c 192.168.1.9 -t 20 -p 5001 -w 130k;
for (( i=1; i<100; i++ )) 

do iperf -c 10.1.16.75 -t 5 -p 45678;
sleep 2

done 
