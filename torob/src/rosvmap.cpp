#include "rosvmap.h"

#include "ogmap.h"
#include "shape.h"
#include "impact.h"

#include <iostream>

using namespace std;
using namespace mia;

/*
 * constructor:
*/

RosVmap :: RosVmap():
  a_min_scan_distance(0.01),
  a_max_scan_distance(100)
{
  setEpsilon( 0.1 );
}

RosVmap :: ~RosVmap()
{
}

/*
 * setter:
*/
int RosVmap :: add_vertex(const mia::Float2 & position, mia::Node2::EType t )
{
  int i= boost::add_vertex( visimap.a_map );
  visimap.a_map[i]= Node2( position, t );
  return i;
}

/*
 * getter:
*/

bool RosVmap :: free_segment(const mia::Float2 & A, const mia::Float2 & B )
{
  list<mia::Graph2::edge_descriptor> l= visimap.getEdges( mia::Node2::type_obstacle,
                                                          mia::Node2::type_obstacle );
  list<mia::Graph2::edge_descriptor>::iterator it, itEnd( l.end() );
  Segment2 ref(A, B);

  for( it= l.begin(); it != itEnd; ++it ){
    int src( boost::source( *it, visimap.a_map ) );
    int trg( boost::target( *it, visimap.a_map ) );

    Segment2 s(visimap.a_map[src], visimap.a_map[trg]);

    if( mia::impact( ref, s ) )
    {
//      cout << "\nfs(no, " << src << ", " << trg  << "): ";
      return false;
    }
  }

  return true;
}

bool RosVmap :: safe_move( const mia::Float2 & pos )
{
  Circle2 ref(pos, visimap.getEpsilon());

  list<mia::Graph2::vertex_descriptor> l;
  l = visimap.getVertices( mia::Node2::type_obstacle );
  
  list<mia::Graph2::vertex_descriptor>::iterator it, itEnd( l.end() );

  for( it= l.begin() ; it != itEnd ; ++it )
  {
    Float2 p( visimap.a_map[*it] );
    if( p.x > pos.x && impact( p, ref ) )
      return false;
  }
  return true;
}

bool RosVmap :: is_vertex_on_segment(const mia::Float2 & A, const mia::Float2 & B, mia::Node2::EType t, int & iNode )
{
  list<mia::Graph2::vertex_descriptor> l= visimap.getVertices( t );
  list<mia::Graph2::vertex_descriptor>::iterator it, itEnd( l.end() );
  
  float minDist2;
  mia::Graph2::vertex_descriptor selection= -1;

  for( it= l.begin() ; it != itEnd ; ++it )
  {
    if( impact( Segment2(A, B), Circle2(visimap.a_map[*it], visimap.getEpsilon()) ) )
    {
      float dist2= A.distance2(visimap.a_map[*it]);
      if( selection == -1 || dist2 < minDist2 )
      {
        minDist2= dist2;
        selection= *it;
        iNode= *it;
      }
    }
  }
  
  return (selection != -1);
}

mia::Float2 RosVmap :: avoid_obstacle(int & iNode, const mia::Float2 & from )const
{
  // Get all obstacle
  int nb_neibor= 0;
  mia::Float2 avoid(0.f, 0.f);
  
  mia::Graph2::out_edge_iterator ie, ieEnd;
  boost::tie(ie, ieEnd)= boost::out_edges( iNode, visimap.a_map );
  while( ie != ieEnd )
  {
    int neibor= boost::source( *ie, visimap.a_map );
    if( neibor == iNode )
      neibor= boost::target( *ie, visimap.a_map );

    if( visimap.a_map[neibor].type == Node2::type_obstacle ){
      cout << "\t\t(" << neibor << ", " << iNode << ")" << endl;
      
      avoid+= mia::Float2(visimap.a_map[neibor], visimap.a_map[iNode]).normal();
      ++nb_neibor;
    }
    ++ie;
  }

  mia::Float2 bet(from, visimap.a_map[iNode]);
  float d( bet.normalize() );
  
  float eps= visimap.getEpsilon();
  float refDist= sqrt( d*d - eps*eps );
  
  mia::Float2 solution2= from + (bet*refDist);
  bet= bet.orthogonal();
  mia::Float2 solution1= solution2 + (bet*1.1f*eps);
  solution2= solution2 + (bet*-1.1f*eps);

//   float safeDist= 1.1f*visimap.getEpsilon();
//   
//   bet= bet.orthogonal();
//   mia::Float2 solution1= visimap.a_map[iNode] + (bet*safeDist);
//   mia::Float2 solution2= visimap.a_map[iNode] + (bet*-safeDist);

  
  if( nb_neibor == 0 )
    return solution1;

  avoid= visimap.a_map[iNode] + avoid*(eps/nb_neibor);
  
  if( solution1.distance(avoid) < solution2.distance(avoid) )
    return solution1;
  
  return solution2;
}

mia::Float2 RosVmap :: get_closest_vertex(const mia::Float2 & A, mia::Node2::EType t)
{
  list<mia::Graph2::vertex_descriptor> l= visimap.getVertices( mia::Node2::type_frontier );
  list<mia::Graph2::vertex_descriptor>::iterator it( l.begin() ), itEnd( l.end() );
  
  if( it == itEnd )
    return A;

  float minDist2= A.distance2( visimap.a_map[*it] );
  mia::Graph2::vertex_descriptor selection= *it;

  for( ++it; it != itEnd ; ++it )
  {
    float dist2= A.distance2( visimap.a_map[*it] );
    if( dist2 < minDist2 )
    {
      minDist2= dist2;
      selection= *it;
    }
  }
  
  return (Float2)visimap.a_map[selection];
}

/*
 * subscriber:
*/
void RosVmap :: map_subscriber(const nav_msgs::OccupancyGrid& map )
{
//     cout << "map_subscriber : " << map.header.frame_id << " " << map.info.width << "x" << map.info.width << endl;
    
    mia::Data d("map", 2+(map.info.width*map.info.height), 4);
    
    d.a_flag[0]= map.info.width;
    d.a_flag[1]= map.info.height;

    int countN(0), count0(0), count50(0), count100(0), sum(0);
    for( int end(map.info.width*map.info.height), i(0) ; i < end ; ++i)
    {
        int celVal= (int)( map.data[i] );
        d.a_flag[2+i]= celVal; //(celVal > 50?100:0);

        countN+= (celVal < 0?1:0);
        count0+= ((celVal == 0)?1:0);
        count50+= ((0 < celVal && celVal < 50)?1:0);
        count100+= ((50 <= celVal && celVal <= 100)?1:0);
        sum+= celVal;
    }

//     cout << "\t nb cells:" << countN << ", " << count0 << ", " << count50 << ", " << count100 << "= " << sum <<  endl;

    d.a_value[0]= map.info.resolution;
    
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quat[0]= map.info.origin.orientation.x;
    quat[1]= map.info.origin.orientation.y;
    quat[2]= map.info.origin.orientation.z;
    quat[3]= map.info.origin.orientation.w;
    tf::Matrix3x3( quat ).getRPY(roll, pitch, yaw);
    
    d.a_value[1]= map.info.origin.position.x;
    d.a_value[2]= map.info.origin.position.y;
    d.a_value[3]= yaw;

//     cout << "\torigine: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0. | "
//         << roll << ", " << pitch << ", "<< yaw << "]" << endl;

    std::ofstream of;
    of.open("/home/guillaume/tmp/map.log", std::ofstream::out);
    d.log( of );
    of.close();
    
//     cout << "\tdata: " << d.mesage() << " (" << d.flag_size() << ", " << d.value_size() << "), [" << d.flag(0) << ", "<< d.flag(1) << ", "<< d.flag(2) << " ... ], [" << d.value(0);
//     for( int i = 1; i < d.value_size() ; ++i )
//       cout << ", " << d.value(i);
//     
//     cout << "]" << endl;
    
//     cout << "\tgridmap..." << endl;
    mia::OGMap gridmap;
    gridmap.initialize( d );
    
//     cout << "\tvisimap..." << endl;
    visimap.initialize( gridmap );
//     cout << "\tok" << endl;
}
void RosVmap :: data_subscriber( const torob_msgs::Data& vmap ){
//     cout << "vmap_subscriber in data format:" << endl;
  
    mia::Data d;
    toMargentData( vmap, &d );
    
//     cout << "\tload vmap..." << endl;
    visimap.load2(d); 
}

void RosVmap :: vmap_subscriber( const torob_msgs::VectorMap& vmap ){
    visimap.a_map.clear();
    
    unsigned int nbNode( vmap.nodes.size() ), nbEdge( vmap.edges.size() );
    visimap.setEpsilon( vmap.resolution );
    for( unsigned int i(0) ; i < nbNode ; ++i )
    {
        assert( i == boost::add_vertex( visimap.a_map) );
        visimap.a_map[i].x= vmap.nodes[i].x;
        visimap.a_map[i].y= vmap.nodes[i].y;
        visimap.a_map[i].type= (Node2::EType)(vmap.nodes[i].type);
    }

    for( unsigned int i(0) ; i < nbEdge ; ++i )
    {
        boost::add_edge( vmap.edges[i].src,  vmap.edges[i].trg, visimap.a_map );
    }
    visimap.initialize_frame();
}

std::list<mia::Float2> RosVmap :: scan_subscriber( const sensor_msgs::LaserScan& scan, bool closed ){
  std::list<mia::Float2> lscan;
  int size= scan.ranges.size();
  float angle( scan.angle_min );
  bool first= true;
  float first_dist(a_max_scan_distance), last_dist(a_max_scan_distance);
  
  for(int i(0); i < size; ++i )
  {
      if( a_min_scan_distance < scan.ranges[i] && scan.ranges[i] < a_max_scan_distance )
      {
        lscan.push_back( mia::Float2::direction( angle ) * scan.ranges[i] );
	if( first )
	{
	  first_dist= scan.ranges[i];
	  first= false;
	}
	last_dist= scan.ranges[i];
      }
      angle+= scan.angle_increment;
  }

  if( closed )
  {
    mia::Float2 pivot( mia::Float2::direction( mia::_PI ) * a_max_scan_distance );
    float step_dist= 0.9 * visimap.getEpsilon();

    {
      mia::Float2 last( mia::Float2::direction( scan.angle_max ) * last_dist );
    
      cout << "close " << last << "->" << pivot << endl;
    
      mia::Float2 step= pivot-last;
      float end_dist= step.normalize();
      for( float dist(step_dist); dist < end_dist; dist+= step_dist )
	lscan.push_back( last + step*dist );
    }
    
    lscan.push_back( pivot );
    
    {
      mia::Float2 first( mia::Float2::direction( scan.angle_min ) * first_dist );
    
      cout << "close " << pivot << "->" << first << endl;
    
      mia::Float2 step= first-pivot;
      float end_dist= step.normalize();
      for( float dist(step_dist); dist < end_dist; dist+= step_dist )
	lscan.push_back( pivot + step*dist );
    }
  }

  visimap.initialize( lscan, mia::Float2(0.0f, 0.0f), visimap.getEpsilon() );
  
  return lscan;
}

/*
 * publisher:
*/
void RosVmap :: publish_data( ros::Publisher & publisher, const std_msgs::Header& header ){
  torob_msgs::Data td;
  
  td.header.frame_id= header.frame_id;
  td.header.stamp= header.stamp;
  
  mia::Data d( visimap.save2() );
  
  toTorobData( d, &td );
  
  publisher.publish( td );
}

void RosVmap :: publish_vmap( ros::Publisher & publisher, const std_msgs::Header& header ){
    torob_msgs::VectorMap vmap;
    
    vmap.header.frame_id= header.frame_id;
    vmap.header.stamp= header.stamp;
    
    int vSize( boost::num_vertices(visimap.a_map) );
    int eSize( boost::num_edges(visimap.a_map) );
    
    //os << a_origin.x << " " << a_origin.y << " " << a_size.x << " " << a_size.y << "\n";
    
    vmap.nodes.resize( vSize );
    vmap.edges.resize( eSize );
    vmap.resolution= visimap.a_epsilon;
    
    if( vSize > 0 )
    {
        Graph2::vertex_iterator it, itEnd;
        boost::tie(it, itEnd)= boost::vertices(visimap.a_map);

        int i(0);
        for( ; it!=itEnd ; ++it )
        {
            assert( i == (int)(*it) );
            
            vmap.nodes[i].x= visimap.a_map[*it].x;
            vmap.nodes[i].y= visimap.a_map[*it].y;
            vmap.nodes[i].type= visimap.a_map[*it].type;
            ++i;
        }
    }
    
    if( eSize > 0 )
    {
        Graph2::edge_iterator it, itEnd;
        boost::tie(it, itEnd)= boost::edges(visimap.a_map);
        int i(0);
        for( ; it!=itEnd ; ++it )
        {
          vmap.edges[i].src= (int)(boost::source(*it, visimap.a_map));
          vmap.edges[i].trg= (int)(boost::target(*it, visimap.a_map));
          ++i;
        }
    }

    publisher.publish( vmap );
}

void RosVmap :: publish_poses( ros::Publisher & publisher, const std_msgs::Header& h, mia::Node2::EType type )
{
    std::list<mia::Graph2::vertex_descriptor> targets= visimap.getVertices( type );
    std::list<mia::Graph2::vertex_descriptor>::reverse_iterator itarget( targets.rbegin() );
    int size= targets.size();
    
    geometry_msgs::PoseArray pa;
    
    pa.header.frame_id= h.frame_id;
    pa.header.stamp= h.stamp;
    pa.poses.resize( size );
    
    for(int i(0) ; i < size ; ++i )
    {
        pa.poses[i].position.x= visimap.a_map[*itarget].x;
        pa.poses[i].position.y= visimap.a_map[*itarget].y;
        pa.poses[i].position.z= 0.f;
        
        pa.poses[i].orientation.x= 1.f;
        pa.poses[i].orientation.y= 0.f;
        pa.poses[i].orientation.z= 0.f;
        pa.poses[i].orientation.w= 1.f;
        
        ++itarget;
    }

    publisher.publish( pa );
}

void RosVmap :: publish_points( ros::Publisher & publisher, const std_msgs::Header& h, mia::Node2::EType type )
{
    std::list<mia::Graph2::vertex_descriptor> targets= visimap.getVertices( type );
    std::list<mia::Graph2::vertex_descriptor>::reverse_iterator itarget( targets.rbegin() );
    int size= targets.size();
    
    sensor_msgs::PointCloud pc;
    
    pc.header.frame_id= h.frame_id;
    pc.header.stamp= h.stamp;
    pc.points.resize( size );
    
    for(int i(0) ; i < size ; ++i )
    {
        pc.points[i].x= visimap.a_map[*itarget].x;
        pc.points[i].y= visimap.a_map[*itarget].y;
        pc.points[i].z= 0.f;
        
        ++itarget;
    }

    publisher.publish( pc );
}

void RosVmap :: publish_frontier( ros::Publisher & publisher, const std_msgs::Header& h, float radius)
{
    std::list<mia::Graph2::vertex_descriptor> targets, alltargets= visimap.getVertices( mia::Node2::type_frontier );
    std::list<mia::Graph2::vertex_descriptor>::iterator it( alltargets.begin() ), itEnd( alltargets.end() );
    
    while( it != itEnd )
    {
      bool cool= true;
      
      mia::Graph2::out_edge_iterator ie, ieEnd;
      boost::tie(ie, ieEnd)= boost::out_edges( *it, visimap.a_map );
      while( cool && ie != ieEnd )
      {
        mia::Graph2::vertex_descriptor src= boost::source( *ie, visimap.a_map );
        mia::Graph2::vertex_descriptor trg= boost::target( *ie, visimap.a_map );
        if( visimap.a_map[src].distance( visimap.a_map[trg] ) < radius )
          cool= false;
        ++ie;
      }
      
      if( cool )
        targets.push_back( *it );
      ++it;
    }
    
    int size= targets.size();
    it= targets.begin();
    
    sensor_msgs::PointCloud pc;
    
    pc.header.frame_id= h.frame_id;
    pc.header.stamp= h.stamp;
    pc.points.resize( size );
    
    for(int i(0) ; i < size ; ++i )
    {
        pc.points[i].x= visimap.a_map[*it].x;
        pc.points[i].y= visimap.a_map[*it].y;
        pc.points[i].z= 0.f;
        
        ++it;
    }

    publisher.publish( pc );
}

/*
 * Map manipulation:
*/
void RosVmap :: merge( const RosVmap & toMerge, const mia::Float2 & translation )
{
  mia::Transform t( translation );
  visimap.add( toMerge.visimap, t );
  visimap.clean_frontier();
}
void RosVmap :: extraFrontierNodes( float dist ){
  std::cout << "-> subdivide edge" << std::endl;
  std::list<Graph2::edge_descriptor> frontiers
    = visimap.getEdges( Node2::type_frontier, Node2::type_obstacle );
  for( std::list<Graph2::edge_descriptor>::iterator it( frontiers.begin() ), itEnd( frontiers.end() ) ;
        it != itEnd ; ++it ){

    unsigned int s(boost::source( *it, visimap.a_map ));
    unsigned int t(boost::target( *it, visimap.a_map ));
  
    std::cout << "\ttest edge " << s << ", " << t << std::endl;
    std::cout << "\tType: " << visimap.a_map[s].type
      << ", " << visimap.a_map[t].type << std::endl;
    
    Float2 between( visimap.a_map[t], visimap.a_map[s] );
    
    if( between.length() > dist ){
      between.normalize(dist);
      unsigned int vf= boost::add_vertex(visimap.a_map);
      visimap.a_map[vf]= Node2(
        visimap.a_map[t] + between,
        Node2::type_frontier
      );
    }
  }
  
  std::cout << "-> end" << std::endl;
}


/*
 * Margent interface:
*/
void RosVmap :: toTorobData( const mia::Data & d, torob_msgs::Data * td ){
  td->mesage= "torob_msgs::Data";
  
  td->flag.resize( d.flag_size() );
  for( int i= 0; i < d.flag_size() ; ++i )
    td->flag[i]= d.flag(i);

  td->value.resize( d.value_size() );
  for( int i= 0; i < d.value_size() ; ++i )
    td->value[i]= d.value(i);
}

void RosVmap :: toMargentData( const torob_msgs::Data & td, mia::Data * d ){
  
  d->initialize( "torob_msgs::Data", td.flag.size(), td.value.size() );
  
  for( int i= 0; i < d->flag_size() ; ++i )
    d->flag(i, td.flag[i] );

  for( int i= 0; i < d->value_size() ; ++i )
    d->value(i, td.value[i] );
}
