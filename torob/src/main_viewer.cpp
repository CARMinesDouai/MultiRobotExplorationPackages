#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "rosvmap.h"
#include "frame.h"
#include "roscontrol.h"

using namespace std;

// global element :
RosVmap * _vmap;
//bool verbose;
bool node_info;

mia::Frame * _frame; // SDL viewer
mia::RosControl * _control; // SDL interaction
std::list<mia::Float2> _scan;

// Callback :
void vmap_subscriber(const torob_msgs::VectorMap & vmap){
  if( !_control->pause() )
    _vmap->vmap_subscriber( vmap );
}

void scan_subscriber(const sensor_msgs::LaserScan & scan);

void process(const ros::TimerEvent& time){
  _control->process();
  
  if( _control->end() ){ ros::shutdown(); }
  
  _frame->drawVisiMap( _vmap->visimap, mia::Float2(0.f, 0.f), node_info );
  _frame->drawPoints( _scan, mia::Color(0xFFFF00FF) );
  _frame->refresh();
}

void process_only_obstacle(const ros::TimerEvent& time){
  _control->process();
  
  if( _control->end() ){ ros::shutdown(); }
  
  _frame->drawVisiMapObstacle( _vmap->visimap, mia::Float2(0.f, 0.f) );
  _frame->drawPoints( _scan, mia::Color(0xFFFF00FF) );
  _frame->refresh();
}

int main(int argc, char **argv)
{
  // ROS:
  cout << "Initialize torob copy_map" << endl;
  ros::init(argc, argv, "torob_viewer");
  ros::NodeHandle node;
  ros::NodeHandle node_private("~");

  // Configuration : 
  std::string vmap_topic, scan_topic;
  bool only_obstacle;
  int size_x, size_y; 
  float scale;

  if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
  if( !node_private.getParam("scan_topic", scan_topic) ) scan_topic= "/no_scan";
  if( !node_private.getParam("node_info", node_info) ) node_info= true;
  if( !node_private.getParam("only_obstacle", only_obstacle) ) only_obstacle= false;
  if( !node_private.getParam("size_x", size_x) ) size_x= 1200;
  if( !node_private.getParam("size_y", size_y) ) size_y=  800;
  if( !node_private.getParam("scale", scale) ) scale= 120.0;

  
//  if( !node_private.getParam("verbose", verbose) ) verbose= false;

  // Moinag: 
  _vmap= new RosVmap();
  
  // SDL 2:
  cout << "Initialize SDL window" << endl;
  _frame= new mia::Frame("Torob Viewer", size_x, size_y, scale);
  _control= new mia::RosControl(_frame);
  
  // Callback function:
  ros::Timer loopTimer;
  if( only_obstacle )
  {
    cout << "Initialize vewer process oo" << endl;
    loopTimer= node.createTimer( ros::Duration( 0.03 ), process_only_obstacle );
  }
  else
  {
    cout << "Initialize vewer process" << endl;
    loopTimer= node.createTimer( ros::Duration( 0.03 ), process );
  }
  
  // subscribers function:
  ros::Subscriber sub1 = node.subscribe( vmap_topic, 1, vmap_subscriber );
  ros::Subscriber sub2 = node.subscribe( scan_topic, 1, scan_subscriber );

  // publisher function:
  
  // get the hand to ros:
  cout << "Run torob viewer on "  << vmap_topic << endl;
  ros::spin();
  ros::spin();

  // Properly stop the program:
  cout << "Close program" << endl;

  return 0;
}

void scan_subscriber(const sensor_msgs::LaserScan & scan){
  _scan.clear();
  float angle( scan.angle_min );
  
  for(int i(0), size(scan.ranges.size()); i < size; ++i )
  {
    _scan.push_back( mia::Float2::direction( angle ) * scan.ranges[i] );
    angle+= scan.angle_increment;
  }
}
