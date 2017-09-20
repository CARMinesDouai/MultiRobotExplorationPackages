
#include <string>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <torob_msgs/VectorMap.h>
#include <torob_msgs/Data.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "ring.h"
#include "impact.h"

//TODO: RingState messages ? :
//TODO: SDL Viewing:

using namespace std;

// global element :
tf::Vector3 _goal;
std::string _next_goal_frame_id, _goal_frame_id;
std::string _cmd_frame_id;

ros::Publisher _cmd_publisher, _ring_publisher;

tf::TransformListener * _listener;

double _dmax_l_speed, _max_a_speed;
float _robot_radius, _safe_distance;

Ring * _ring;

// Callback :
void goal_subscriber(const geometry_msgs::PoseStamped & g);
void scan_subscriber(const sensor_msgs::LaserScan & scan);

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize Torobo::Move: " << endl;
    ros::init( argc, argv, "move" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    _listener= new tf::TransformListener(node);

    // Configuration Scan:
    std::string scan_topic, ring_topic;
//    float perception_distance;
    int ring_size;

    if( !node_private.getParam("scan_topic", scan_topic) ) scan_topic= "/scan";
    if( !node_private.getParam("robot_radius", _robot_radius) ) _robot_radius= 0.3f;
    if( !node_private.getParam("safe_distance", _safe_distance) ) _safe_distance= 2.f*_robot_radius;
//    if( !node_private.getParam("perception_distance", perception_distance) ) perception_distance= 3.0f*_safe_distance;
    if( !node_private.getParam("ring_size", ring_size) ) ring_size= 13;

    // Configuration Movement:
    std::string goal_topic, cmd_topic;
    float goal_x, goal_y;

    if( !node_private.getParam("goal_topic", goal_topic) ) goal_topic= "move_base_simple/goal";
    if( !node_private.getParam("goal_frame_id", _next_goal_frame_id) ) _next_goal_frame_id= "odom";
    if( !node_private.getParam("init_goal_x", goal_x) ) goal_x= 0.f;
    if( !node_private.getParam("init_goal_y", goal_y) ) goal_y= 0.f;
    if( !node_private.getParam("init_goal_frame_id", _goal_frame_id) ) _goal_frame_id= "odom";

    if( !node_private.getParam("cmd_topic", ring_topic) ) ring_topic= "/ring_state";
    if( !node_private.getParam("cmd_topic", cmd_topic) ) cmd_topic= "/cmd_vel_mux/input/navi";
    if( !node_private.getParam("cmd_frame_id", _cmd_frame_id) ) _cmd_frame_id= "base_link";

    if( !node_private.getParam("linear_speed", _dmax_l_speed) ) _dmax_l_speed= 0.2;
    if( !node_private.getParam("angular_speed", _max_a_speed) ) _max_a_speed= 1.2;

    // goal initialization:
    _goal.setX( goal_x );
    _goal.setY( goal_y );
    _goal.setZ( 0.0f );

    // ring initialization:
    _ring= new Ring(_safe_distance, _robot_radius, ring_size, _cmd_frame_id.c_str());

    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( scan_topic, 1, scan_subscriber );
    ros::Subscriber sub2 = node.subscribe( goal_topic, 1, goal_subscriber );

    // publisher function:
    _ring_publisher= node.advertise<torob_msgs::Ring>( ring_topic, 1 );
    _cmd_publisher= node.advertise<geometry_msgs::Twist>( cmd_topic, 1 );

    // get the hand to ros:
    cout << "run torob move" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close move" << endl;

    delete _listener;
    delete _ring;

    return 0;
}

void goal_subscriber(const geometry_msgs::PoseStamped & g){

  cout << "goal_subscriber" << endl;

  _goal_frame_id= _next_goal_frame_id;

  _goal.setX( g.pose.position.x );
  _goal.setY( g.pose.position.y );
  _goal.setZ( g.pose.position.z );

  cout << "goal in " << g.header.frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;

  if ( _listener->waitForTransform( _goal_frame_id, g.header.frame_id, g.header.stamp, ros::Duration(0.5) ) )
  {
    tf::StampedTransform toRefGoalFrame;

    try{
      _listener->lookupTransform( _goal_frame_id, g.header.frame_id, g.header.stamp,  toRefGoalFrame );
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    _goal= toRefGoalFrame * _goal;

    cout << "goal in " << _goal_frame_id  << " frame :" << _goal.x() << ", " << _goal.y() << endl;
  }
  else{
    cerr << "Transform " <<  g.header.frame_id << "-" << _goal_frame_id << " unvailable" <<  endl;
  }
}

void scan_subscriber(const sensor_msgs::LaserScan & scan){

//  cout << "scan_subscriber" << endl;
  list<mia::Float2> target;
  list<mia::Float2> dot= _ring->scan_subscriber(scan, _listener);

  // Wait for appropriate Transform :
  bool transform_ok= true;
  mia::Float2 localGoal(_goal.x(), _goal.y());
  bool is_intermediate= false;

  if ( _goal_frame_id.compare(_cmd_frame_id) != 0 ){
    if ( !_listener->waitForTransform( _cmd_frame_id, _cmd_frame_id, scan.header.stamp, ros::Duration(0.5) ) )
    {
      cerr << "Command transform: " << scan.header.frame_id
           << " -> " << _cmd_frame_id << " unvailable." << endl;
      transform_ok= false;
    }

    // Transforms goal into commande frames:
    tf::StampedTransform goalToCmd;
    tf::Vector3 rosGoal(localGoal.x, localGoal.y, 0.f);

    try{
      _listener->lookupTransform( _cmd_frame_id, _goal_frame_id,
                                scan.header.stamp, goalToCmd );
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      transform_ok= false;
    }

    if( transform_ok ){
      rosGoal= goalToCmd*rosGoal;
      localGoal= mia::Float2( rosGoal.x(), rosGoal.y() );
    }
  }

  //Correct local goal:
  target.push_front( localGoal );

  if(!transform_ok){
    localGoal= mia::Float2(0.f, 0.f);
    is_intermediate= true;
    target.push_front( localGoal );
  }
  else{
    // If target on the back:
    if( localGoal.x < -_robot_radius ){
      if( localGoal.y < 0.f )
        localGoal= mia::Float2( 0.f, -_safe_distance*1.5f );
      else
        localGoal= mia::Float2( 0.f, -_safe_distance*1.5f );
      is_intermediate= true;
      target.push_front( localGoal );
    }

    // Check free line to target:
    bool freeLine= true;
    mia::Segment2 refSegment( mia::Float2(0.f, 0.f), localGoal );
    mia::Float2 contact;
    for( std::list<mia::Float2>::iterator it(dot.begin()), itEnd(dot.end()); freeLine && it != itEnd ; ++it ){
        if( mia::impact( refSegment, mia::Circle2(*it, _robot_radius), contact ) )
          freeLine= false;
    }

    // If no free line to target choose an other one:
    if( !freeLine ){
      localGoal= _ring->wayOut(localGoal);
      is_intermediate= true;
      target.push_front( localGoal );
    }
  }

  _ring->publish_ring(_ring_publisher, scan.header, target);

  cout << "\t-> move to (" << _goal.x() << ", " << _goal.y()
    <<  ") in " << _goal_frame_id << " -> "
    << localGoal <<  " in " << _cmd_frame_id << endl;

  // generate appropriate commande message :
  mia::Float2 norm_goal(localGoal);
  float d= norm_goal.normalize();

  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  cout << "\t-> generate the command: distance "
    << d  << " vs " << _robot_radius*0.94;

  if( d > _robot_radius*0.94 )// !stop condition :
  {
    if( localGoal.x > _robot_radius*0.94 )
    {
      cmd.linear.x= _dmax_l_speed + min( 0.5*localGoal.x*_dmax_l_speed, _dmax_l_speed );

      if( 0.005f < norm_goal.y )
        cmd.angular.z= min( norm_goal.y*2.0*_max_a_speed, _max_a_speed );

      if( -0.005f > norm_goal.y )
        cmd.angular.z= -(min( norm_goal.y*-2.0*_max_a_speed, _max_a_speed ));
    }
    else
    {
      cmd.angular.z= _max_a_speed;
    }
  }
  else
    cout << " stop !";

  cout << "\n\t-> commande linear: " << cmd.linear.x << ", angular: " << cmd.angular.z << endl;

  _cmd_publisher.publish( cmd );

  cout << "\t-> end generate the command: " << endl;
}
