#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "rosvmap.h"

// Margent:
#include "net.h"

using namespace std;

RosVmap _vmap;
std::map<std::string,ros::Publisher> _vmap_publisher;

int main(int argc, char **argv)
{
    // ROS:
    cout << "Initialize torob net_receive: " << endl;
    ros::init( argc, argv, "net_receive" );
    ros::NodeHandle node;
    ros::NodeHandle node_private("~");

    // Configuration : 
    std::string ns_topic, frame_id;
    int port;
    
    if( !node_private.getParam("port", port) ) port= 1414;
    if( !node_private.getParam("ns_topic", ns_topic) ) ns_topic= "";
    if( !node_private.getParam("frame_id", frame_id) ) frame_id= "/map";

    // Moinag: 
    
    // Callback function:
    
    // subscribers function:
    
    // publisher function:
    
    NetData net(port, true);
    cerr << "Server initialization: " << net.getError() << endl;
    
    std_msgs::Header h;
    // get the hand to ros:
    cout << "run torob net_receive" << endl;

    do{
      ros::spinOnce();
      
      mia::Data d;
      if( !net.wait( d ) )
      {
        cerr << net.getError() << endl;
      }
      else{

//        cout << "data:" << d << endl;

        _vmap.visimap.load2( d );
        
        std::string vmap_topic( ns_topic + d.mesage() );
      
        cout << "publish on:" << vmap_topic << endl;

        if( _vmap_publisher.find(vmap_topic) == _vmap_publisher.end() ){
          cout << "create:" << vmap_topic << " publisher." << endl;
          _vmap_publisher[vmap_topic]= node.advertise<torob_msgs::VectorMap>( vmap_topic, 1 );
        }
        
        h.frame_id= frame_id;
        h.stamp= ros::Time::now();
        _vmap.publish_vmap( _vmap_publisher[vmap_topic], h );
      }
    }while( node.ok() );
    
//    net.close_server();
    
    if( node.ok() ){ 
      cout << "shutdown the node" << endl;
      ros::spin();
    }
    
    // Properly stop the program:
    cout << "close net_receive" << endl;

    return 0;
}
