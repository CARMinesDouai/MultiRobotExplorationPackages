#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "rosvmap.h"

#include "net.h"

using namespace std;

int _nbAddr;
string * _addr;

std::string _vmap_topic;
RosVmap _vmap;
int _port;

int _fileId(0);
int _resources_free(true);

void vmap_subscriber( const torob_msgs::VectorMap& vmap ){

	cout << "Go send the map : " << endl;
	if( !_resources_free )
	{
		cout << "\t/!\\ resource occupied" << endl;
		return ;
	}
	_resources_free= false;

  _vmap.vmap_subscriber( vmap );

  mia::Data d= _vmap.visimap.save2();
  
  d.a_mesage_size= _vmap_topic.length();
  delete [] d.a_mesage;
  d.a_mesage= new char [d.a_mesage_size+1];
  d.a_mesage[d.a_mesage_size]= 0;
  strcpy( d.a_mesage, _vmap_topic.c_str() );
  
  NetData net(_port, false);
  for( int i(0); i < _nbAddr ; ++i )
    if( !net.send( _addr[i].c_str(), d) )
  {
    cout << "/!\\ Torob network communication to " << _addr[i].c_str() << " error: " << net.getError() << endl;
  }
  else cout << "map sent !" << endl;

  //cout << "data:" << d << endl;

  cout << "save map !" << endl;
	std::string file_name( "map_"+ std::to_string(++_fileId) + ".vmap" );

	_vmap.visimap.save( file_name );

  _vmap.visimap.load2(d);

//   NetData net;
//   if( !net.send( "192.168.0.29", test) )
//   {
//       cout << net.getError() << endl;
//       return 1;
//   }
//   
	_resources_free= true;
}

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob net_send: " << endl;
    ros::init( argc, argv, "net_send" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration :
    if( !node_private.getParam("port", _port) ) _port= 1414;
    if( !node_private.getParam("vmap_topic", _vmap_topic) ) _vmap_topic= "/vmap";
    if( !node_private.getParam("number_addr", _nbAddr) ) _nbAddr= 1;
    
    _addr= new string[_nbAddr];
    
    std::string add= "addr_";
    for( int i(0); i < _nbAddr ; ++i ){
      if( !node_private.getParam( add + std::to_string(i+1), _addr[i]) ) _addr[i]= "10.1.10.195";
      cout << add + std::to_string(i+1) << ": " << _addr[i] << endl;
    }

    // Moinag: 
    
    // Callback function:
    ros::Subscriber sub0 = node.subscribe( _vmap_topic, 1, vmap_subscriber );
    
    // subscribers function:
    
    // publisher function:
    
    // get the hand to ros:
    cout << "run torob net_send" << endl;
    ros::spin();

    // Properly stop the program:
    cout << "close net_send" << endl;

    delete [] _addr;
    
    return 0;
}
