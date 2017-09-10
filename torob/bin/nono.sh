#!/bin/sh
# launch web server
echo "Start nono:"
cd `dirname $0`/../web && node app.js &

# launch ROS
roslaunch torob gmapping.launch

