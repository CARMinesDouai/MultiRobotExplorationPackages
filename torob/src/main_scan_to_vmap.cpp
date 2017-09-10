#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "rosvmap.h"

using namespace std;

// global element :
RosVmap * _vmap;

ros::Publisher _vmap_publisher;

// Callback :
void scan_subscriber(const sensor_msgs::LaserScan & scan){
  _vmap->scan_subscriber(scan);
  _vmap->publish_vmap( _vmap_publisher, scan.header );
}

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob scan interpreter: " << endl;
    ros::init( argc, argv, "scan_interpreter" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string scan_topic, vmap_topic;
    float robot_radius, perception_distance;
    
    if( !node_private.getParam("scan_topic", scan_topic) ) scan_topic= "/scan";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/scan_vmap";
    if( !node_private.getParam("robot_radius", robot_radius) ) robot_radius= 0.3f;
    if( !node_private.getParam("perception_distance", perception_distance) ) perception_distance= 2.0f;

    // Moinag: 
    _vmap= new RosVmap();
    _vmap->setEpsilon( robot_radius );
    _vmap->a_min_scan_distance= robot_radius*0.75f;
    _vmap->a_max_scan_distance= perception_distance;
    
    // Callback function:
    
    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( scan_topic, 1, scan_subscriber );

    // publisher function:
    _vmap_publisher= node.advertise<torob_msgs::VectorMap>( vmap_topic, 1 );
    
    // get the hand to ros:
    cout << "run torob scan interpreter" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close scan interpreter" << endl;

    delete _vmap;
    return 0;
}
