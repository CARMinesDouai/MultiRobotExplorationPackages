/**
 * grid to vmap
 * 
 * Subscribe to map topic convert it in vectorial and plublish in a vmap topic.
**/
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>

#include "rosvmap.h"
#include "map_publisher.h"

using namespace std;

// global element :
RosVmap * _vmap;
//ros::Publisher _map_publisher;

// Callback :
void vmap_subscriber(const torob_msgs::VectorMap& vmap){
  _vmap->vmap_subscriber(vmap);
  
  cout << "publish_map..." << endl;
  
  publishMapAsGraph( _vmap->visimap, 0.f, 0.f);
  mapUpdate();
//  _vmap->publish_map( _vmap_publisher, map.header );
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
    std::string map_topic, vmap_topic;
    
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
    if( !node_private.getParam("map_topic", map_topic) ) map_topic= "/map";

    // Moinag: 
    _vmap= new RosVmap();
    
    // Callback function:

    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( vmap_topic, 1, vmap_subscriber );
    
    // publisher function:
//    _map_publisher= node.advertise<nav_msgs::OccupancyGrid>( map_topic, 1);
    initialize_map_publisher(node, node_private, map_topic);

    // get the hand to ros:
    cout << "run torob " << node_name.str() << " (send map on " << map_topic << ")" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close torob" << node_name.str() << endl;

//    _moinag->save_visibility( "out_map.vm" );

    delete _vmap;

    return 0;
}
