#ifndef MAP_PUBLISHER_H
#define MAP_PUBLISHER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>

#include "rosvmap.h"

void initialize_map_publisher(ros::NodeHandle &node, ros::NodeHandle &node_private, std::string map_topic);
void initialize_map_publisher(ros::NodeHandle &node, ros::NodeHandle &node_private);

void publishMapAsGraph(const mia::VisiMap & visimap, float robot_x, float robot_y);
void tf_Callback(const tf::tfMessage::ConstPtr& mes);
void bhm_line(int x1,int y1,int x2,int y2,int c);
void clearMap();
void putpixel(int x, int y, int c);
bool mapUpdate();

#endif // MAP_PUBLISHER_H