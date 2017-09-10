#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

//ros::Publisher _vmap_publisher;

void process( const ros::TimerEvent& ){
  cout << "send map..." << endl;
}

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob sandbox: " << endl;
    ros::init( argc, argv, "sandbox" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string vmap_file, vmap_topic;
    float duration;
    
    if( !node_private.getParam("vmap_file", vmap_file) ) vmap_file= "./map.vmap";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "";
    if( !node_private.getParam("duration", duration) ) duration= 0.1f;

    // Moinag: 
    
    // Callback function:
    ros::Timer loopTimer;
    loopTimer= node.createTimer( ros::Duration( duration ), process );
    
    // subscribers function:
    
    // publisher function:
    _cmd_publisher= node.advertise<geometry_msgs::Twist>( cmd_topic, 1 );
    
    // get the hand to ros:
    cout << "run torob sandbox" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close sandbox" << endl;

    return 0;
}
