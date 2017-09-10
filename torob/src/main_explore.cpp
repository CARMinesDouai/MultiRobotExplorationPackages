/**
 * grid to vmap
 * 
 * Subscribe to map topic convert it in vectorial and plublish in a vmap topic.
**/
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>

#include "rosvmap.h"

using namespace std;

// global element :
RosVmap * _vmap;
ros::Publisher _frontier_publisher;
float _robot_radius;

// Callback :
void vmap_subscriber(const torob_msgs::VectorMap& vmap){
  _vmap->vmap_subscriber(vmap);
  _vmap->publish_frontier( _frontier_publisher, vmap.header, _robot_radius );
}

int main(int argc, char **argv)
{
    // ROS:
    time_t timer;
    time(&timer);
    
    cout << "Initialize torob explore" << endl;
    ros::init( argc, argv, "explore" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string vmap_topic, frontier_topic;
    
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
    if( !node_private.getParam("frontier_topic", frontier_topic) ) frontier_topic= "/frontier";
    if( !node_private.getParam("robot_radius", _robot_radius) ) _robot_radius= 0.2f;

    // Moinag: 
    _vmap= new RosVmap();
    
    // Callback function:

    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( vmap_topic, 1, vmap_subscriber );
    
    // publisher function:
    _frontier_publisher= node.advertise<sensor_msgs::PointCloud>( frontier_topic, 1);
    
    // get the hand to ros:
    cout << "run torob explore" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close torob explore" << endl;

    delete _vmap;

    return 0;
}
