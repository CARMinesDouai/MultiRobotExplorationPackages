#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


#include "float2.h"

using namespace std;

ros::Publisher _goal_publisher;
float _min_distance, _max_distance, _max_angle;
std::string _frame_id;

void process( const ros::TimerEvent& ){

    float step= (_max_distance-_min_distance)*0.0001f;  
    float distance= _min_distance + (float)(rand()%10000) * step;
  
    step= (2.0f*_max_angle)*0.0001f;
    float angle= -_max_angle + (float)(rand()%10000) * step;
    
    mia::Float2 goal( distance, 0.f );
    goal.rotate(angle);
    
    cout << "new random goal at " << distance << " meters and " << angle << " rad. coord: " << goal << endl;

    geometry_msgs::PoseStamped g;

    g.header.stamp= ros::Time::now();
    g.header.frame_id= _frame_id; 

    g.pose.position.x= goal.x;
    g.pose.position.y= goal.y; 
    g.pose.position.z= 0.f;
    
    _goal_publisher.publish( g );
}

int main(int argc, char **argv)
{
    // ROS:
    
    cout << "Initialize torob random target: "<< argc << endl;
    for(int i= 0; i < argc ; ++i )
      cout << "\targ-" << i << ": " << argv[i];

    ros::init( argc, argv, "random target" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration :
    std::string goal_topic;
    float duration;
    
    if( !node_private.getParam("goal_topic", goal_topic) ) goal_topic= "move_base_simple/goal";
    if( !node_private.getParam("frame_id", _frame_id) ) _frame_id= "/base_link";
    
    if( !node_private.getParam("duration", duration) ) duration= 1.f;
    if( !node_private.getParam("min_distance", _min_distance) ) _min_distance= 1.f;
    if( !node_private.getParam("max_distance", _max_distance) ) _max_distance= 2.f;
    if( !node_private.getParam("max_angle", _max_angle) ) _max_angle= 3.14/4.f;
    
    // Moinag: 
    
    // initialize random seed:
    srand (time(NULL));
  
    // Callback function:
    ros::Timer loopTimer;
    loopTimer= node.createTimer( ros::Duration( duration ), process );
    
    // subscribers function:
    
    // publisher function:
    _goal_publisher= node.advertise<geometry_msgs::PoseStamped>( goal_topic, 1 );
    
    // get the hand to ros:
    cout << "run torob random target" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close random target" << endl;

    return 0;
}
