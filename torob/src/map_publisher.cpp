#include "map_publisher.h"

#include <sstream>

#define trb_Unknown_Cell  -1
#define trb_Occupied_Cell 1
#define trb_Free_Cell     0
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))// compute linear index for given map coords
//#define trb_map_width_metre  50   // map width [metre]
//#define trb_map_height_metre 50   // map height [metre]
using namespace geometry_msgs;
using namespace std;

ros::Publisher grap_to_map_publisher;            // graph of the map
ros::Publisher robot_model_publisher;            // robot model
ros::Publisher _map_publisher;                   // grid of the map
ros::Publisher _map_publisher_m;                 //grid of the map metadata
ros::Subscriber scanSub;                         // scan
nav_msgs::GetMap::Response map_;                 // our map
double trb_resolution_;                           // map resolution
std::string trb_map_frame_;                      // map resolution
bool is_first_map_;                              // map resolution
int trb_offset_x, trb_offset_y;                  // map resolution
int trb_map_width_pixel, trb_map_height_pixel;   // map resolution
double trb_map_width_metre;
double trb_map_height_metre;

void initialize_map_publisher(ros::NodeHandle &node, ros::NodeHandle &node_private){
  initialize_map_publisher( node, node_private, "/map_2");
}

void initialize_map_publisher(ros::NodeHandle &node, ros::NodeHandle &node_private, std::string map_topic)
{
    //publisher for graph and robot model
    grap_to_map_publisher = node.advertise<visualization_msgs::MarkerArray>("graph_map",1);
    robot_model_publisher = node.advertise<visualization_msgs::Marker>("robot_torob",1);
    //======================================================================================MAP
    //map to be seen in arviz
    _map_publisher   = node.advertise<nav_msgs::OccupancyGrid>( map_topic, 1, true);
    stringstream mapmeta_topic;
    mapmeta_topic << map_topic << "_metadata";
    _map_publisher_m = node.advertise<nav_msgs::MapMetaData>( mapmeta_topic.str(), 1, true);

    //other param.
    //node_private.getParam("trb_resolution",trb_resolution_);
    node_private.getParam("trb_map_width_metre",trb_map_width_metre);
    node_private.getParam("trb_map_height_metre",trb_map_height_metre);
    trb_map_frame_ = "map";// grid map frame
    trb_resolution_ = 0.05;// grid map resolution
    trb_map_width_metre=50;
    trb_map_height_metre=50;
    is_first_map_=true;    // if the first map calculate it all, otherwise, we just update out graph lines on the map
    
//     cout<<trb_resolution_<<endl;
//     cout<<trb_map_width_metre<<endl;
//     cout<<trb_map_height_metre<<endl;
    
    trb_map_width_pixel = (float)trb_map_width_metre/trb_resolution_;   // torop map width [pixel]
    trb_map_height_pixel = (float)trb_map_height_metre /trb_resolution_;// torop width [pixel]
    trb_offset_x = - trb_map_width_metre/2;              // torop map origin on X [pixel]
    trb_offset_y = - trb_map_height_metre/2;             // torop map origin on Y [pixel]
    //cout<<trb_map_width_pixel<<" :: "<<trb_map_height_pixel<<endl;cout<<trb_offset_x<<" :: "<<trb_offset_y<<endl;
    //======================================================================================MAP
}

void publishMapAsGraph(const mia::VisiMap & visimap, float robot_x, float robot_y)//&(_moinag.torob->visibility()->a_visibility);
{
//  cout<<"publish graph: " << endl;

  //std::vector<float> torob_graph;
  //read the graph
  
  
  
   int nbVertex= boost::num_vertices(visimap.a_map);

  visualization_msgs::MarkerArray marray_g;

  //graph nodes
  visualization_msgs::Marker map_nd;
  map_nd.header.frame_id = "map";
  map_nd.header.stamp = ros::Time::now();
  map_nd.id = 0;
  map_nd.ns = "totob";
  map_nd.type = visualization_msgs::Marker::SPHERE;
  map_nd.pose.position.x= 0.0;
  map_nd.pose.position.y= 0.0;
  map_nd.pose.position.z= 0.0;
  map_nd.scale.x= 0.1;
  map_nd.scale.y= 0.1;
  map_nd.scale.z = 0.1;
  map_nd.color.r = 1.0;
  map_nd.color.g= 0;
  map_nd.color.b = 0.0;
  map_nd.color.a = 1.0;
  map_nd.lifetime = ros::Duration(0);
  map_nd.action = visualization_msgs::Marker::ADD;

  int iv= 0;
  mia::Graph2::vertex_iterator vt, vtEnd;
  for( boost::tie(vt, vtEnd)= boost::vertices(visimap.a_map) ; vt != vtEnd ; ++vt )
    {
        map_nd.id= iv;
        ++iv;

        map_nd.pose.position.x = visimap.a_map[*vt].x;
        map_nd.pose.position.y = visimap.a_map[*vt].y;

        marray_g.markers.push_back( visualization_msgs::Marker(map_nd) );
    }

  //graph edges
  visualization_msgs::Marker map_edge;
  map_edge.header.frame_id = "map";
  map_edge.header.stamp = ros::Time::now();
  map_edge.action = visualization_msgs::Marker::ADD;
  map_edge.ns = "totob";
  map_edge.id = 0;
  map_edge.type = visualization_msgs::Marker::LINE_STRIP;
  map_edge.scale.x = 0.1;
  map_edge.scale.y = 0.1;
  map_edge.scale.z = 0.1;
  map_edge.color.r = 0.0;
  map_edge.color.g = 0.0;
  map_edge.color.b = 1.0;
  map_edge.color.a = 1.0;

  clearMap();
  
  geometry_msgs::Point min, max;  
  {
    boost::tie(vt, vtEnd)= boost::vertices(visimap.a_map);

    min.x= visimap.a_map[*vt].x;
    min.y= visimap.a_map[*vt].y;
    max.x= visimap.a_map[*vt].x;
    max.y= visimap.a_map[*vt].y;

    for( ++vt; vt != vtEnd; ++vt)
    {
        if( visimap.a_map[*vt].x < min.x )
            min.x= visimap.a_map[*vt].x;
        if( visimap.a_map[*vt].y < min.y )
            min.y= visimap.a_map[*vt].y;
        
        if( visimap.a_map[*vt].x > max.x )
            max.x= visimap.a_map[*vt].x;
        if( visimap.a_map[*vt].y > max.y )
            max.y= visimap.a_map[*vt].y;
    }

    trb_map_width_pixel= 2+(int)((max.x-min.x)/trb_resolution_);
    trb_map_height_pixel= 2+(int)((max.y-min.y)/trb_resolution_);
  }
  
  int ie= nbVertex;
  
  mia::Graph2::edge_iterator et, etEnd;
  int nbEdges(0);

  for( boost::tie(et, etEnd)= boost::edges( visimap.a_map ) ; et != etEnd ; ++et )
    {
        int source= boost::source( *et, visimap.a_map );
        int target= boost::target( *et, visimap.a_map );

        if( visimap.a_map[source].type == mia::Node2::type_obstacle &&  visimap.a_map[target].type == mia::Node2::type_obstacle )
        {
            ++nbEdges;

            map_edge.id = ie;
            ++ie;

            map_edge.points.clear();
            geometry_msgs::Point p;
            
            p.x= visimap.a_map[source].x;
            p.y= visimap.a_map[source].y;
            p.z= 0.0;
            map_edge.points.push_back(p);

            //====================================>>>>>>>>>>=========================MAP RELATED CODE
            //first point of the line
            //p.x = rand() % (int)trb_map_width_metre;
            //p.y = rand() % (int)trb_map_height_metre;
            int x1( 1+(int)((p.x-min.x)/trb_resolution_) ), y1( 1+(int)((p.y-min.y)/trb_resolution_) );
            //cout<<" x1 "<<p.x<<endl;
            //cout<<" y1 "<<p.y<<endl;
            //====================================<<<<<<<<<<=========================MAP RELATED CODE
            p.x= visimap.a_map[target].x;
            p.y= visimap.a_map[target].y;
            map_edge.points.push_back(p);

            //====================================>>>>>>>>>>=========================MAP RELATED CODE
            //p.x = rand() % (int)trb_map_width_metre;
            //p.y = rand() % (int)trb_map_height_metre;
            //second point of the line
            int x2( 1+(int)((p.x-min.x)/trb_resolution_) ), y2( 1+(int)((p.y-min.y)/trb_resolution_) );

            //cout<<trb_map_width_pixel<<" trb_map_width_metre  x1 "<<p.x<<endl;
            //cout<<" y2 "<<x1<<endl;
            //cout<<" x2 "<<x2<<endl;
            //cout<<trb_map_height_pixel<<"trb_map_height_metre y2 "<<y2<<endl;
            //Shifting to the midle of the map //bhm_line(int(x1), int(y1), int(x2), int(y2), 1);

            int int_x1(x1);//+trb_map_width_pixel/2);
            int int_y1(y1);//+trb_map_height_pixel/2);
            int int_x2(x2);//+trb_map_width_pixel/2);
            int int_y2(y2);//+trb_map_height_pixel/2);

            if( int_x1 < 0 || int_y1 < 0 || int_x2 < 0 || int_y2 < 0 )
            {
            cerr << "ERROR edge [" << source << ", " << target << "] n°" << ie;
            cerr << " segment (" << x1 << ", " << y1 << ") -> (" << x2 << ", " << y2 << ")";
            cerr << " pixel (" << int_x1 << ", " << int_y1 << ") -> (" << int_x2 << ", " << int_y2 << ")" << endl;
            }
            else bhm_line( int_x1, int_y1, int_x2, int_y2, 1);//ex. fixed line: bhm_line( 0, 0, 300, 700, 1);

            //========================================================================MAP RELATED CODE
            marray_g.markers.push_back( visualization_msgs::Marker(map_edge) );
        }
    }

    //====================================>>>>>>>>>>=========================MAP RELATED CODE
    // Set the header information on the map
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = trb_map_frame_;

    _map_publisher.publish(map_.map);
    _map_publisher_m.publish(map_.map.info);

    //cout<<trb_map_width_pixel<<" :mmmmmm: "<<trb_map_height_pixel<<endl;cout<<trb_offset_x<<" :mmmmmmmmmm: "<<trb_offset_y<<endl;
    //====================================<<<<<<<<<<=========================MAP RELATED CODE
    //robot model as a cube
    visualization_msgs::Marker robot_md;
    robot_md.header.frame_id = "odom";
    robot_md.header.stamp = ros::Time::now();
    robot_md.id = 0;
    robot_md.ns = "totob";
    robot_md.type = visualization_msgs::Marker::CUBE;
    robot_md.pose.position.x = robot_x;
    robot_md.pose.position.y = robot_y;
    robot_md.pose.position.z = 0.0;
    robot_md.scale.x = 0.9;robot_md.scale.y = 0.9;robot_md.scale.z = 0.9;
    robot_md.color.r = 0.0;  robot_md.color.g = 1.0;robot_md.color.b = 0.0;robot_md.color.a = 1.0;
    robot_md.lifetime = ros::Duration(0);


    map_nd.action = visualization_msgs::Marker::DELETE;
    /*for (; id < marker_count_; id++)
    {
    map_nd.id = id;
    marray_g.markers.push_back(visualization_msgs::Marker(map_nd));
    }*/
    //marker_count_ = marray_g.markers.size();
    //we publish the graph

    grap_to_map_publisher.publish(marray_g);
    //we publish the robot model as a cube
    robot_model_publisher.publish(robot_md);


//   cout<<"\t-publishing graph size: "<<marray_g.markers.size();
//   cout<<"\npublish graph end." << endl;
}
void bhm_line(int x1,int y1,int x2,int y2,int c)
{
 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
 dx=x2-x1;
 dy=y2-y1;
 dx1=fabs(dx);
 dy1=fabs(dy);
 px=2*dy1-dx1;
 py=2*dx1-dy1;
 if(dy1<=dx1)
 {
    if(dx>=0){
        x=x1;
        y=y1;
        xe=x2;
    }else{
        x=x2;
        y=y2;
        xe=x1;
    }
    putpixel(x,y,c);
    for(i=0;x<xe;i++)
    {
        x=x+1;
        if(px<0){
            px=px+2*dy1;
        }else{
        if((dx<0 && dy<0) || (dx>0 && dy>0)){
            y=y+1;
        }else{
            y=y-1;
        }
        px=px+2*(dy1-dx1);
        }
        //delay(0);
        putpixel(x,y,c);
    }
 }else{
    if(dy>=0){
        x=x1;
        y=y1;
        ye=y2;
    }else{
        x=x2;
        y=y2;
        ye=y1;
    }
    putpixel(x,y,c);
    for(i=0;y<ye;i++)
    {
        y=y+1;
        if(py<=0){
            py=py+2*dx1;
        }else{
        if((dx<0 && dy<0) || (dx>0 && dy>0))
        {
            x=x+1;
        }else{
            x=x-1;
        }
        py=py+2*(dx1-dy1);
        }
        //delay(0);
        putpixel(x,y,c);
    }
 }
}

void clearMap()
{
    for (int y=0; y<map_.map.info.height; y++)
        for (int x=0; x< map_.map.info.width; x++)
    {
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
}

void putpixel(int x, int y, int c)
{
    if( 0 <= x && x < map_.map.info.width && 0 <= y && y < map_.map.info.height )
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
}

bool mapUpdate()
{
// cout<<"from map reseting up param before 0"<<endl;
    
  if(is_first_map_) {
    map_.map.info.resolution = trb_resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
    is_first_map_=false;
  }


  //we change the output map position if it does not corresponds to our origins
  if(map_.map.info.width != (unsigned int) trb_map_width_pixel ||
     map_.map.info.height != (unsigned int) trb_map_height_pixel ||
     map_.map.info.origin.position.x != trb_offset_x ||
     map_.map.info.origin.position.y != trb_offset_y )
  {
    map_.map.info.origin.position.x = trb_offset_x;
    map_.map.info.origin.position.y = trb_offset_y;
    map_.map.info.width = trb_map_width_pixel;
    map_.map.info.height = trb_map_height_pixel;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
//     cout<<"from map reseting up param "<<map_.map.info.width<<endl;
//     cout<<"from map reseting up param "<<map_.map.info.origin.position.y<<endl;
  }
//cout<<"from map reseting up param before 2"<<endl;
if(is_first_map_== false) return true;

  for (int y=0; y<trb_map_height_pixel; y++)//we may select only specific cells
  {
    for (int x=0; x< trb_map_width_pixel; x++)
    {
      // Getting the value at position x,y
      //kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      int trb_value = rand() % 3;
      if(trb_value == 2)
          trb_value=-1;
      trb_value=-1;
      switch (trb_value)
      {
        case -1://trb_Unknown_Cell:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case 1://trb_Occupied_Cell:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case 0://trb_Free_Cell:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = trb_map_frame_;

  _map_publisher.publish(map_.map);
  _map_publisher_m.publish(map_.map.info);

  //delete occ_grid;

  return true;
}
