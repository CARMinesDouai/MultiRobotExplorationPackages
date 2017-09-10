/**
 * grid to vmap
 * 
 * Subscribe to map topic convert it in vectorial and plublish in a vmap topic.
**/

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
//#include <tf/Matrix3x3.h>

#include "rosvmap.h"

using namespace std;

// global element :
RosVmap * _vmap;
const int _maxVmapToMerge= 4;
RosVmap * _vmapToMerge;

ros::Publisher _vmap_publisher;
ros::Publisher _frontier_publisher;

mia::Float2 _translation[_maxVmapToMerge];

// Callback :
void vmap_subscriber(const torob_msgs::VectorMap& vmap){
  // reinitialize map:
  _vmap->vmap_subscriber(vmap);
  
  // Merge with other maps:
  for( int i(0) ; i < _maxVmapToMerge ; ++i )
    _vmap->merge( _vmapToMerge[i], _translation[i] );
  
  // Publish:
  _vmap->publish_vmap( _vmap_publisher, vmap.header );
  _vmap->publish_points( _frontier_publisher, vmap.header, mia::Node2::type_frontier );
}

void merge_subscriber1(const torob_msgs::VectorMap& vmap){ _vmapToMerge[0].vmap_subscriber(vmap); }
void merge_subscriber2(const torob_msgs::VectorMap& vmap){ _vmapToMerge[1].vmap_subscriber(vmap); }
void merge_subscriber3(const torob_msgs::VectorMap& vmap){ _vmapToMerge[2].vmap_subscriber(vmap); }
void merge_subscriber4(const torob_msgs::VectorMap& vmap){ _vmapToMerge[3].vmap_subscriber(vmap); }

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob merge: " << endl;
    ros::init(argc, argv, "merge");
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration :
    std::string vmap_topic, merged_topic, frontier_topic;
    std::string merge_topic[_maxVmapToMerge];
    
    if( !node_private.getParam("merged_vmap_topic", merged_topic) ) merged_topic= "/merged_vmap"; // published map
    if( !node_private.getParam("merged_frontier_topic", frontier_topic) ) frontier_topic= "/merged_frontier";
    if( !node_private.getParam("vmap_topic", vmap_topic) ) vmap_topic= "/vmap"; // reference map
    
    if( !node_private.getParam("vmap1_topic", merge_topic[0]) ) merge_topic[0]= "/vmap1"; // map 1 to add.
    if( !node_private.getParam("vmap2_topic", merge_topic[1]) ) merge_topic[1]= "/vmap2"; // map 2 to add.
    if( !node_private.getParam("vmap3_topic", merge_topic[2]) ) merge_topic[2]= "/vmap3"; // map 3 to add.
    if( !node_private.getParam("vmap4_topic", merge_topic[3]) ) merge_topic[3]= "/vmap4"; // map 4 to add.
    
    if( !node_private.getParam("translation1x", _translation[0].x) )  _translation[0].x= 0.f;
    if( !node_private.getParam("translation1y", _translation[0].y) )  _translation[0].y= 0.f;
    
    if( !node_private.getParam("translation2x", _translation[1].x) )  _translation[1].x= 0.f;
    if( !node_private.getParam("translation2y", _translation[1].y) )  _translation[1].y= 0.f;

    if( !node_private.getParam("translation3x", _translation[2].x) )  _translation[2].x= 0.f;
    if( !node_private.getParam("translation3y", _translation[2].y) )  _translation[2].y= 0.f;
    
    if( !node_private.getParam("translation4x", _translation[3].x) )  _translation[3].x= 0.f;
    if( !node_private.getParam("translation4y", _translation[3].y) )  _translation[3].y= 0.f;

    // Moinag: 
    _vmap= new RosVmap();
    _vmapToMerge= new RosVmap[_maxVmapToMerge];

    // Callback function:

    // subscribers function:
    ros::Subscriber sub = node.subscribe( vmap_topic, 1, vmap_subscriber );
    
    ros::Subscriber sub1 = node.subscribe( merge_topic[0], 1, merge_subscriber1 );
    ros::Subscriber sub2 = node.subscribe( merge_topic[1], 1, merge_subscriber2 );
    ros::Subscriber sub3 = node.subscribe( merge_topic[2], 1, merge_subscriber3 );
    ros::Subscriber sub4 = node.subscribe( merge_topic[3], 1, merge_subscriber4 );

    // publisher function:
    _vmap_publisher= node.advertise<torob_msgs::VectorMap>( merged_topic, 1);
    _frontier_publisher= node.advertise<sensor_msgs::PointCloud>( frontier_topic, 1);
    
    // get the hand to ros:
    cout << "run torob map_to_vmap" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close torob map_to_vmap" << endl;

//    _moinag->save_visibility( "out_map.vm" );

    delete _vmap;
    delete [] _vmapToMerge;

    return 0;
}
