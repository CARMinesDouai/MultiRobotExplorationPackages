#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "rosvmap.h"
#include "ring.h"
#include "frame.h"
#include "roscontrol.h"

using namespace std;

// global element :
RosVmap * _vmap;
//bool verbose;
bool node_info;

mia::Frame * _frame; // SDL viewer
std::string _frame_id;
mia::RosControl * _control; // SDL interaction
Ring * _ring;
std::list<mia::Float2> _scan;
std::list<mia::Float2> _target;

tf::TransformListener * _listener;

// Callback :
void vmap_subscriber(const torob_msgs::VectorMap & vmap){
  if( !_control->pause() )
    _vmap->vmap_subscriber( vmap );
}

void ring_subscriber(const torob_msgs::Ring & ring){
  if( !_control->pause() )
    _target= _ring->ring_subscriber( ring );
}

void scan_subscriber(const sensor_msgs::LaserScan & scan);
void process(const ros::TimerEvent& time);

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

  _listener= new tf::TransformListener(node);

  // Configuration :
  std::string vmap_topic, scan_topic, ring_topic;
  bool only_obstacle;
  int size_x, size_y, ring_size;
  float scale;

  if( !node_private.getParam("frame_id", _frame_id) ) _frame_id= "/base_link";
  if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap";
  if( !node_private.getParam("scan_topic", scan_topic) ) scan_topic= "/no_scan";
  if( !node_private.getParam("ring_topic", ring_topic) ) ring_topic= "/ring_state";

  if( !node_private.getParam("node_info", node_info) ) node_info= true;
  if( !node_private.getParam("only_obstacle", only_obstacle) ) only_obstacle= false;
  if( !node_private.getParam("size_x", size_x) ) size_x= 1200;
  if( !node_private.getParam("size_y", size_y) ) size_y=  800;
  if( !node_private.getParam("scale", scale) ) scale= 120.0;


//  if( !node_private.getParam("verbose", verbose) ) verbose= false;

  // Moinag:
  _vmap= new RosVmap();
  _ring= new Ring();

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
  ros::Subscriber sub0 = node.subscribe( scan_topic, 1, scan_subscriber );
  ros::Subscriber sub1 = node.subscribe( vmap_topic, 1, vmap_subscriber );
  ros::Subscriber sub2 = node.subscribe( ring_topic, 1, ring_subscriber );

  // publisher function:

  // get the hand to ros:
  cout << "Run torob viewer on "  << vmap_topic << endl;
  ros::spin();
  ros::spin();

  // Properly stop the program:
  delete _vmap;
  delete _ring;
  delete _frame;
  delete _control;

  cout << "Close program" << endl;

  return 0;
}

void scan_subscriber(const sensor_msgs::LaserScan & scan){
  _scan.clear();
  Ring tmp_ring(1.f, 0.2f, 3, _frame_id.c_str());
  _scan= tmp_ring.scan_subscriber_blanck(scan, _listener);
}

void process(const ros::TimerEvent& time){
  _control->process();

  if( _control->end() ){ ros::shutdown(); }

  if( _vmap->size() > 1 )
    _frame->drawVisiMap( _vmap->visimap, mia::Float2(0.f, 0.f), node_info );
  _frame->drawRing(*_ring);
  int i= 0;
  for(std::list<mia::Float2>::iterator it(_target.begin()), itEnd(_target.end()) ; it != itEnd ; ++it ){
    _frame->drawCircle( *it, 0.15, 0xAA0000FF );
    _frame->write( std::to_string( i++ ).c_str(), *it, 0xAA0000FF);
  }
  _frame->drawPoints( _scan, mia::Color(0xFFFF00FF) );
  _frame->refresh();
}
