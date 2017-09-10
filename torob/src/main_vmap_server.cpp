#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <rosvmap.h>

using namespace std;

RosVmap _vmap;
ros::Publisher _vmap_publisher;

void process( const ros::TimerEvent& ){
  cout << "send map..." << endl;
  std_msgs::Header h;
  _vmap.publish_vmap( _vmap_publisher, h );
}

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob vmap_server: " << endl;
    ros::init( argc, argv, "vmap_server" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string vmap_file, vmap_topic;
    float duration;
    
    if( !node_private.getParam("vmap_file", vmap_file) ) vmap_file= "./map.vmap";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
    if( !node_private.getParam("duration", duration) ) duration= 1.0f;

    // Moinag: 
    _vmap.visimap.load( vmap_file );

    // Callback function:
    ros::Timer loopTimer= node.createTimer( ros::Duration( duration ), process );
    
    // subscribers function:
    
    // publisher function:
    _vmap_publisher= node.advertise<torob_msgs::VectorMap>( vmap_topic, 1 );
    
    // get the hand to ros:
    cout << "run torob vmap_server" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close sandbox" << endl;

    return 0;
}
