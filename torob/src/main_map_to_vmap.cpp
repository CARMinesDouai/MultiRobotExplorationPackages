/**
 * grid to vmap
 * 
 * Subscribe to map topic convert it in vectorial and plublish in a vmap topic.
**/
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>

#include "rosvmap.h"

#include <torob_msgs/Node.h>

using namespace std;

// global element :
RosVmap * _vmap;
ros::Publisher _vmap_publisher;
ros::Publisher _frontier_publisher;
float _robot_radius;

// Callback :
void map_subscriber(const nav_msgs::OccupancyGrid& map){
  _vmap->map_subscriber(map);
  _vmap->publish_vmap( _vmap_publisher, map.header );
  _vmap->publish_frontier( _frontier_publisher, map.header, _robot_radius );
}

int main(int argc, char **argv)
{
    // ROS:
    time_t timer;
    time(&timer);
    
    std::stringstream node_name;
    node_name << "map_to_vmap" << timer%1000;
    cout << "Initialize torob map_to_vmap: " << node_name.str() << endl;
    ros::init( argc, argv, node_name.str() );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string map_topic, vmap_topic, frontier_topic;
    
    if( !node_private.getParam("map_topic", map_topic) ) map_topic= "/map";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
    if( !node_private.getParam("frontier_topic", frontier_topic) ) frontier_topic= "/frontier";
    if( !node_private.getParam("robot_radius", _robot_radius) ) _robot_radius= 0.2f;

    // Moinag: 
    _vmap= new RosVmap();
    
    // Callback function:

    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( map_topic, 1, map_subscriber );
    
    // publisher function:
    _vmap_publisher= node.advertise<torob_msgs::VectorMap>( vmap_topic, 1);
    _frontier_publisher= node.advertise<sensor_msgs::PointCloud>( frontier_topic, 1);
    
    // get the hand to ros:
    cout << "run torob " << node_name.str() << " (send vmap on " << vmap_topic << ")" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close torob" << node_name.str() << endl;

//    _moinag->save_visibility( "out_map.vm" );

    delete _vmap;

    return 0;
}
