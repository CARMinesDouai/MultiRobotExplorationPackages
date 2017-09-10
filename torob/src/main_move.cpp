#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "rosvmap.h"

using namespace std;

// global element :
RosVmap * _vmap;
tf::Vector3 _goal;
std::string _goal_frame_id;
std::string _cmd_frame_id;

ros::Publisher _vmap_publisher;
ros::Publisher _cmd_publisher;

tf::TransformListener * _listener;

double _dmax_l_speed, _max_a_speed;

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
    std::string scan_topic, vmap_topic;
    float robot_radius, perception_distance;
    
    if( !node_private.getParam("scan_topic", scan_topic) ) scan_topic= "/scan";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/scan_vmap";
    if( !node_private.getParam("robot_radius", robot_radius) ) robot_radius= 0.3f;
    if( !node_private.getParam("perception_distance", perception_distance) ) perception_distance= 2.0f;
    
    // Configuration Movement: 
    std::string goal_topic, cmd_topic;
    float goal_x, goal_y;

    if( !node_private.getParam("goal_topic", goal_topic) ) goal_topic= "move_base_simple/goal";
    if( !node_private.getParam("goal_frame_id", _goal_frame_id) ) _goal_frame_id= "odom";
    if( !node_private.getParam("init_goal_x", goal_x) ) goal_x= 0.f;
    if( !node_private.getParam("init_goal_y", goal_y) ) goal_y= 0.f;

    if( !node_private.getParam("cmd_topic", cmd_topic) ) cmd_topic= "/cmd_vel_mux/input/navi";
    if( !node_private.getParam("cmd_frame_id", _cmd_frame_id) ) _cmd_frame_id= "base_link";
    
    if( !node_private.getParam("linear_speed", _dmax_l_speed) ) _dmax_l_speed= 0.2;
    if( !node_private.getParam("angular_speed", _max_a_speed) ) _max_a_speed= 1.2;

//    _dmax_l_speed*= 0.5;
    
    // Moinag: 
    _vmap= new RosVmap();
    _vmap->setEpsilon( robot_radius );
    _vmap->a_min_scan_distance= 0.5*robot_radius;
    _vmap->a_max_scan_distance= perception_distance;

    _goal.setX( goal_x );
    _goal.setY( goal_y );
    _goal.setZ( 0.0f );

    // subscribers function:
    ros::Subscriber sub1 = node.subscribe( scan_topic, 1, scan_subscriber );
    ros::Subscriber sub2 = node.subscribe( goal_topic, 1, goal_subscriber );

    // publisher function:
    _vmap_publisher= node.advertise<torob_msgs::VectorMap>( vmap_topic, 1 );
    _cmd_publisher= node.advertise<geometry_msgs::Twist>( cmd_topic, 1 );

    // get the hand to ros:
    cout << "run torob move" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close move" << endl;

    delete _listener;
    delete _vmap;

    return 0;
}

void goal_subscriber(const geometry_msgs::PoseStamped & g){
    
  cout << "goal_subscriber" << endl;
  
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

  cout << "scan_subscriber" << endl;
  
  _vmap->scan_subscriber(scan, true);

  // Wait for appropriate Transform :
  
  if ( _goal_frame_id.compare( scan.header.frame_id ) != 0 /* ros::Time(0) */
    && !_listener->waitForTransform( scan.header.frame_id, _goal_frame_id,
                                     scan.header.stamp, ros::Duration(0.5) ) 
  )
    cerr << "Scan transform: " << _goal_frame_id
         << " -> " << scan.header.frame_id << " unvailable." << endl;
    
  if ( _goal_frame_id.compare(_cmd_frame_id) != 0
     && _cmd_frame_id.compare( scan.header.frame_id ) != 0
     && !_listener->waitForTransform( _cmd_frame_id, scan.header.frame_id,
                                      scan.header.stamp, ros::Duration(0.5) )
  )
    cerr << "Command transform: " << scan.header.frame_id
         << " -> " << _cmd_frame_id << " unvailable." << endl;

  bool transform_ok= true;

  // Get Transforms (goal -> scan frames and scan -> commande frames):

  tf::StampedTransform goalToScan;
  try{
    _listener->lookupTransform( scan.header.frame_id, _goal_frame_id,
                                scan.header.stamp, goalToScan );
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    transform_ok= false;
  }
  
  tf::StampedTransform scanToCmd;
  try{
    _listener->lookupTransform( scan.header.frame_id, _cmd_frame_id,
                                scan.header.stamp, scanToCmd );
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    transform_ok= false;
  }

  tf::Vector3 localGoal(0.f, 0.f, 0.f);

  // If transforms exist move goal in scan frame:
  if( transform_ok )
  {
    localGoal= goalToScan * _goal;

    mia::Float2 goalF2( localGoal.x(), localGoal.y() );
    _vmap->add_vertex( goalF2, mia::Node2::type_free );

    cout << "\tinitial goal in scan frame :" << goalF2 << endl;
    
    mia::Float2 obs;

    // If goal not in direct area get closest frontier node:
    if( !_vmap->free_segment( mia::Float2(0.f, 0.f), goalF2 )
      || _vmap->is_vertex_on_segment( mia::Float2(0.f, 0.f), goalF2,
                                      mia::Node2::type_obstacle, obs) )
    {
      goalF2= _vmap->get_closest_vertex( goalF2, mia::Node2::type_frontier );
      cout << "\tget frontier vertex: " << goalF2
           << "(" << _vmap->add_vertex( goalF2, mia::Node2::type_free )
           << ")" << endl;
    }

//     cout << "\trafine goal: ";
//     if( _vmap->is_vertex_on_segment(mia::Float2(0.f, 0.f), goalF2, mia::Node2::type_obstacle, obs) )
//     {
//       if( mia::crossProduct(obs, goalF2) > 0.f )
// 	goalF2= obs + obs.orthonormal() * (_vmap->visimap.getEpsilon()*1.2f);
//       else
// 	goalF2= obs + obs.orthonormal() * (-_vmap->visimap.getEpsilon()*1.2f);
// 
//       cout << goalF2;
//       _vmap->add_vertex( goalF2, mia::Node2::type_free );
//     }
//     cout << endl;

    // trafine goal: ";
    if( _vmap->is_vertex_on_segment( mia::Float2(0.f, 0.f), goalF2,
                                     mia::Node2::type_obstacle, obs) )
    {
//      cout << "(" << _vmap->is_vertex_on_segment( mia::Float2(0.f, 0.f), goalF2,
//                                               mia::Node2::type_obstacle, obs)
//           <<  " " << obs << ") ";

      float d( obs.normalize() );
      goalF2= obs * (d - _vmap->visimap.getEpsilon());

      cout << "\tget safe obstacle position :" << goalF2
           << "(" << _vmap->add_vertex( goalF2, mia::Node2::type_free )
           << ")" << endl;
    }
    
    localGoal.setX( goalF2.x );
    localGoal.setY( goalF2.y );
    
    localGoal= scanToCmd * localGoal;

    cout << "\tmove to (" << _goal.x() << ", " << _goal.y() <<  ") -> (" << localGoal.x() << ", " << localGoal.y() <<  ")" << endl;
  }
  else
    cout << "\tstop ! (" << _goal.x() << ", " << _goal.y() <<  ") -> (" << localGoal.x() << ", " << localGoal.y() <<  ")" << endl;

  // generate appropriate commande message :
  mia::Float2 norm_goal(localGoal.x(), localGoal.y());
  float d= norm_goal.normalize();
  
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  if( d > _vmap->a_min_scan_distance*0.2 )// !stop condition :
  {
    if( localGoal.x() > _vmap->a_min_scan_distance*0.2 )
    {
      cmd.linear.x= _dmax_l_speed + min( 0.5*localGoal.x()*_dmax_l_speed, _dmax_l_speed );

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

  if( !_vmap->safe_move( mia::Float2(0.f, 0.f) ) ){
    cout << "\tlinear movement desabled" << endl;
    cmd.linear.x= 0.f;
  }

  cout << "\tcommande linear: " << cmd.linear.x << ", angular: " << cmd.angular.z << endl;

  _cmd_publisher.publish( cmd );
  _vmap->publish_vmap( _vmap_publisher, scan.header );
}

