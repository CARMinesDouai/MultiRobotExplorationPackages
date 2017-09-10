/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2015  Guillaume L. <guillaume.lozenguez@mines-douai.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "visimap.h"
#include "shape.h"
#include "impact.h"

#include <fstream>

using namespace mia;
using namespace std;


// Constructor/Destructor:
VisiMap::VisiMap():
    a_epsilon(0.001f),
    a_map(),
    a_origin(0.f, 0.f),
    a_size(1.f, 1.f)
{
    // Initialize Free Position :
    Graph2::vertex_descriptor vd= boost::add_vertex(a_map);
    a_map[vd]= Float2(0.f, 0.f);
    assert( (int)(vd) == 0 );
}


VisiMap::VisiMap( const VisiMap & vm ):
    a_epsilon(vm.a_epsilon),
    a_map(vm.a_map),
    a_origin(vm.a_origin),
    a_size(vm.a_size)
{
    
//     Graph2::vertex_iterator vi, viEnd;
//     for( boost::tie(vie, viEnd)= boost::vertex(a_map);
//     a_map[vd]= Float2(0.f);
//     assert( (int)(vd) == 0 );
}

VisiMap::~VisiMap()
{
}

// Agent Function:


// getter:
std::list<Graph2::vertex_descriptor> VisiMap::getVertices( Node2::EType type )const
{
    std::list<Graph2::vertex_descriptor> lvertices;

    Graph2::vertex_iterator it, itEnd;
    for( boost::tie(it, itEnd)= boost::vertices(a_map) ; it != itEnd ; ++it )
    {
        if( a_map[*it].type == type )
        {
            lvertices.push_back( *it );
        }
    }
    
    return lvertices;
}

list<Graph2::vertex_descriptor> VisiMap::getVertices( Float2 pos, float dist )const
{
    std::list<Graph2::vertex_descriptor> lvertices;
    float dist2= dist*dist;
    
    Graph2::vertex_iterator it, itEnd;
    for( boost::tie(it, itEnd)= boost::vertices(a_map) ; it != itEnd ; ++it )
    {
        if( a_map[*it].distance2(pos) < dist2 )
        {
            lvertices.push_back( *it );
        }
    }
    
    return lvertices;
}

std::list<Graph2::edge_descriptor> VisiMap::getEdges( Node2::EType srcT, Node2::EType trgT )const
{
    std::list<Graph2::edge_descriptor> ledge;

    Graph2::edge_iterator it, itEnd;
    for( boost::tie(it, itEnd)= boost::edges(a_map) ; it != itEnd ; ++it )
    {
        if( a_map[ boost::source( *it, a_map ) ].type == srcT
                && a_map[boost::target( *it, a_map )].type == trgT )
        {
            ledge.push_back( *it );
        }
    }
    return ledge;
}

void VisiMap::initialize_frame(){// Generate origine and size frame coordinates.
    
    Graph2::vertex_iterator vi, viEnd;
    boost::tie(vi, viEnd)= boost::vertices(a_map);
    
    a_origin= a_map[*vi];
    a_size= a_map[*vi];
    ++vi;
    while(vi != viEnd){
        a_origin.x= min( a_origin.x, a_map[*vi].x );
        a_origin.y= min( a_origin.y, a_map[*vi].y );
        a_size.x= max( a_size.x, a_map[*vi].x );
        a_size.y= max( a_size.y, a_map[*vi].y );
        ++vi;
    }
    Float2 epsilon2d(a_epsilon, a_epsilon);
    a_origin= a_origin - epsilon2d;
    a_size= a_size - a_origin + epsilon2d;
}

// Node managment :
void VisiMap::remove_node(Node2::EType)
{
    Graph2::vertex_iterator vi, viEnd;
    bool ok= false;
    while( !ok )
    {
        ok= true;
        for( boost::tie(vi, viEnd)= boost::vertices(a_map) ; ok && vi != viEnd ; ++vi )
        {
            boost::clear_vertex( *vi, a_map );
            boost::remove_vertex( *vi, a_map );

            ok= false;
        }
    }
}

// 2D transformation :
void VisiMap::transform( const Transform & t)
{
    Graph2::vertex_iterator vi, viEnd;
    for( boost::tie(vi, viEnd)= boost::vertices(a_map) ; vi != viEnd ; ++vi )
    {
        a_map[*vi]= t( a_map[*vi] );
    }
}

// Simple goal detection :
Float2 VisiMap::goal_closest( const Float2 &ref )const
{
    std::list<Graph2::edge_descriptor> front= VisiMap::getEdges( Node2::type_frontier, Node2::type_frontier );
    typedef std::list<Graph2::edge_descriptor>::const_iterator edIt;
    
    Float2 goal;
    float dist2;
    bool selected(false);

    for( edIt it( front.begin() ), itEnd( front.end() ) ; it != itEnd ; ++it )
    {
        Float2 middle= (a_map[boost::source( *it, a_map )] + a_map[boost::target( *it, a_map )]);
        middle*= 0.5f;
        float vardist2= ref.distance2(middle);
        
        if( vardist2 < dist2 || !selected )
        {
            goal= middle;
            dist2= vardist2;
            selected= true;
        }
    }

    return goal;
};

Float2 VisiMap::goal_random()const
{
    std::list<Graph2::edge_descriptor> front= VisiMap::getEdges( Node2::type_frontier, Node2::type_frontier );
    std::list<Graph2::edge_descriptor>::const_iterator it( front.begin() );
    
    if( front.empty() ) return Float2();
    
    int r= randomInt( front.size() );
    for( ; r != 0 ; --r )
        ++it;

    return (a_map[boost::source( *it, a_map )] + a_map[boost::target( *it, a_map )]) * 0.5f;
}

Float2 VisiMap::goal_largest()const
{
    std::list<Graph2::edge_descriptor> front= VisiMap::getEdges( Node2::type_frontier, Node2::type_frontier );
    typedef std::list<Graph2::edge_descriptor>::const_iterator edIt;
 
    if( front.empty() ) return Float2();
    
    Float2 goal;
    float dist2;
    bool selected(false);

    for( edIt it( front.begin() ), itEnd( front.end() ) ; it != itEnd ; ++it )
    {
        Float2 src( a_map[boost::source( *it, a_map )] ), trg( a_map[boost::target( *it, a_map )] );
        float vardist2= src.distance2(trg);
        
        if( vardist2 > dist2 || !selected )
        {
            goal= (src+trg)*0.5f;
            dist2= vardist2;
            selected= true;
        }
    }

    return goal;
};

Float2 VisiMap::goal_largest_random( int range )const
{
    list<valued<Float2>> vfront= list_goal();
 
    list<valued<Float2>>::reverse_iterator it( vfront.rbegin() );
    int r= randomInt( min( (int)(vfront.size()), range ) );
    for( ; r != 0 ; --r )
        ++it;

    return it->item;
};

// All goals:
// std::list<std::pair<Float2,Float2>> VisiMap::list_frontier()const
// {
//     int numv( boost::num_vertices(a_map) );
//     int group[numv];
//     
//     // initialize empty group:
//     for( int i(0) ; i < numv ; ++i )
//         group[i]= 0;
// 
//     // Build group :
//     int grp(1);
//     for( int iRef(0) ; iRef < numv ; ++iRef )
//         if( group[iRef] == 0 && a_map[iRef].type == Node2::type_frontier )
//     {
//         
//         list<int> grpVertices;
//         grpVertices.push_back( iRef );
//         
//         while( !grpVertices.empty() )
//         {
//             int ref= grpVertices.
//             group[iRef]= grp;
//             boost::out_edges<>()
//         }
//         
//         ++grp;
//     }
// }

std::list<valued<Float2>> VisiMap::list_goal()const
{
    std::list<Graph2::edge_descriptor> front= VisiMap::getEdges( Node2::type_frontier, Node2::type_frontier );
    list<valued<Float2>> vfront;
    
    for( std::list<Graph2::edge_descriptor>::const_iterator it( front.begin() ), itEnd( front.end() ) ; it != itEnd ; ++it )
    {
        Float2 src( a_map[ boost::source(*it, a_map) ] );
        Float2 trg( a_map[ boost::target(*it, a_map) ] );
        vfront.push_back( valued<Float2>( (src+trg) * 0.5f, src.distance(trg) ) );
    }
    vfront.sort();
    
    return vfront;
}

// DataIt:

// Initialization:
void VisiMap::generation_from_grid( const OGMap & ogrid, int tref, int ttrg, Node2::EType nodeType, float gridEps )
{
//     std::cout << "generation_from_grid :" << std::endl;

    //Filtering:
    unsigned int width( ogrid.width() ), height( ogrid.height() );
    
    int ** limit= new int * [width]; //[height];
    for( unsigned int i(0) ; i < width ; ++i )
        limit[i]= new int[height];
    
    for( unsigned int i(0); i < width ; ++i )
    {
        for( unsigned int j(0); j < height ; ++j )
        {
            if( ogrid[i][j] == ttrg && ogrid.is_neighbor_type(i, j, tref) )
            {
                limit[i][j]= -2; // mark as cell to treat (-2: interesting -cell obstacle, frontier...- but untreated yet)
            }
            else limit[i][j]= -1;
        }
    }
    
    //  group the obstacle cell sharing a same segment regression at a_epsilon treshold :
    int iObs= 0;
    for( unsigned int i1(0); i1 < width ; ++i1 )
        for( unsigned int j1(0); j1 < height ; ++j1 )
    {
        if( limit[i1][j1] == -2 )
        {
// 	    std::cout << "\tNew group "<< iObs <<" starting on ("  << i1 << ", " << j1 << ")" << std::endl;
// 	    ++iObs;

            // Initialize group structure :
            list<Int2> group;
            list<Float2> group_position;
            int group_size(1);
            
            limit[i1][j1]= -3; // mark the cell
            group.push_back( Int2(i1, j1) );
            group_position.push_back( Float2(i1, j1) );
            
            for( list<Int2>::iterator it( group.begin()); it != group.end() ; ++it ) // For each cell in the group :
            {
                for( int ii( max(it->x-1, 0) ), iiEnd( min( it->x+2, (int)width ) ) ; ii < iiEnd ; ++ii )
                    for( int jj( max(it->y-1, 0) ), jjEnd( min(it->y+2, (int)height) ) ; jj < jjEnd ; ++jj )
                {
                    if( limit[ii][jj] == -2 )// Search for new available cell to add :
                    {
                        group_position.push_back( Float2(ii, jj) ); // added for test
                        
                        if( Float2::possibleSegmentRegression( group_position, gridEps ) )
                        {  // New cell is added if and only if it does not destroy segement regression constraint :
                            limit[ii][jj]= -3; // mark the cell
                            group.push_back( Int2(ii, jj) );
                            ++group_size;
                        }
                        else
                        {
                            group_position.pop_back(); // removed if not valid cell
                        }
                    }
                }
            }
            
            // Build the group:
            if( group_size > 1 )
            {
                array<Float2, 2> segment= Float2::projectionSegment( group_position, Float2::simpleLinearRegression( group_position ) );

                // generate node and edge according to segment from the regression.
                Graph2::vertex_descriptor vd1, vd2;
                vd1= boost::add_vertex(a_map);
                vd2= boost::add_vertex(a_map);
                a_map[vd1]= Node2( ogrid.gridToWorld( segment[0] ), nodeType );
                a_map[vd2]= Node2( ogrid.gridToWorld( segment[1] ), nodeType );
		
// 	        std::cout << "\t\tConnect ("  << vd1 << ", " << vd2 << ")" << std::endl;
                Graph2::edge_descriptor ed= boost::add_edge(vd1, vd2, a_map).first;
                a_map[ed]= Node2::type_free;
                
                assert( (int)(vd1) > 0 && (int)(vd2) > 0 );
                
                // associate a node id to each cell of the group.
                for( list<Float2>::const_iterator ipos(group_position.begin()), iposend(group_position.end()) ; ipos != iposend ; ++ipos )
                {
                    if( ipos->distance2(segment[0]) < ipos->distance2(segment[1]) )
                        limit[ (int)(ipos->x) ][ (int)(ipos->y) ]= (int)(vd1);
                    else
                        limit[ (int)(ipos->x) ][ (int)(ipos->y) ]= (int)(vd2);
                }
            }
            else // Simple case the group is composed by one cell 
            {
                list<Int2>::iterator it(group.begin());
                
                Graph2::vertex_descriptor vd1= boost::add_vertex(a_map);
                a_map[vd1]= Node2( ogrid.gridToWorld( Float2(it->x, it->y) ), nodeType );
                limit[it->x][it->y]= (int)(vd1);
                
// 		std::cout << "\t\t\Add node " << vd1 << std::endl;
            }
        }
    }
    
    //  Connect pieces together...
    for( int i1(1), iEnd(width-1); i1 < iEnd ; ++i1 )
        for( int j1(1), jEnd(height-1); j1 < jEnd ; ++j1 )
            if( limit[i1][j1] > -1 )
    {
        for( int ni(i1-1); ni < i1+2 ; ++ni )
            for( int nj(j1-1); nj < j1+2 ; ++nj )
                if( limit[ni][nj] > -1 && limit[ni][nj] != limit[i1][j1]
		    && !boost::edge( limit[ni][nj], limit[i1][j1], a_map ).second )
        {
// 	    std::cout << "\tConnect "  << limit[ni][nj] << " with " << limit[i1][j1] << " " << std::endl;
            Graph2::edge_descriptor ed= boost::add_edge(limit[ni][nj], limit[i1][j1], a_map).first;
            a_map[ed]= nodeType;
        }
    }

    for( unsigned int i(0) ; i < width ; ++i )
        delete [] limit[i];
    delete [] limit;
}

void VisiMap::initialize( const OGMap & grid )
{
//    cout << "VisiMap process..." << endl;

    // Initialize the graph with only the zero position:
    Node2 freePos= a_map[0];
    a_map.clear();
    Graph2::vertex_descriptor vd= boost::add_vertex(a_map);
    a_map[vd]= freePos;
    assert( (int)(vd) == 0 );

    // Initialize variable:
    float gridEps= 0.708f;
    a_epsilon= gridEps * grid.resolution();
    a_origin= grid.origin();
    a_size= Float2( (float)(grid.width())*grid.resolution(), (float)(grid.height())*grid.resolution() );
    
    generation_from_grid( grid, 0, 100, Node2::type_obstacle, gridEps );
    generation_from_grid( grid, 0, -1, Node2::type_frontier, gridEps );
    frontier_regression();
}

void VisiMap::initialize( const OGMap & grid, Float2 refposition, float epsilon )
{
    initialize(grid);
    setRefPosition(refposition);
    setEpsilon( epsilon );
    obstacle_regression();
}

void VisiMap::initialize( const std::list<Float2> & scan, Float2 refposition, float epsilon )
{
//  cout << "coucou" << endl;
  
  // Initialize the map:
  a_map.clear();
  Graph2::vertex_descriptor vd= boost::add_vertex(a_map);
  a_map[vd]= refposition;
  setEpsilon( epsilon );

  // TODO : filtrer et corriger le scan à epsilon prés.
  
  // read scan :
  const int size( scan.size() );
  vector<Float2> obstacles;
  obstacles.resize(size);
  vector<float> angle;
  angle.resize(size);
  vector<int> node;
  node.resize(size);

  int i(0);
  for( list<Float2>::const_iterator it= scan.begin(); it != scan.end(); ++it )
  {
    obstacles[i]= *it;
    angle[i]= obstacles[i].angle();
    ++i;
  }

  // sort : // TODO trie à bulle :
  for(unsigned int i1= 0; i1 < obstacles.size(); ++i1 )
  {
  int iMin= i1;

    for(unsigned int i2= i1; i2 < obstacles.size(); ++i2 )
      if( angle[iMin] > angle[i2] )
	iMin= i2;

    Float2 var= obstacles[i1];
    obstacles[i1]= obstacles[iMin];
    obstacles[iMin]= var;
    float varA= angle[iMin];
    angle[iMin]= angle[i1];
    angle[i1]= varA;
  }

  if( size == 0 )
    return;

  int last= 0;
  node[last]= boost::add_vertex( a_map );
  a_map[ node[last] ]= Node2( obstacles[last], Node2::type_obstacle );
  
  // build opening :
  float sumRadius=  2*a_epsilon;
  Graph2::vertex_descriptor ref;
  int count= 0;
  
  for( int i=1; i < size; ++i )
  {
    float distAB= obstacles[i-1].distance(obstacles[i]);
    
    if( distAB > sumRadius )
    {
      if( last != i-1 ){
	node[i-1]= boost::add_vertex( a_map );
	a_map[ node[i-1] ]= Node2( obstacles[i-1], Node2::type_obstacle );
	boost::add_edge( node[last], node[i-1], a_map );
      }
      
      node[i]= boost::add_vertex( a_map );
      a_map[ node[i] ]= Node2( obstacles[i], Node2::type_obstacle );

      ref= boost::add_vertex( a_map );
      float diffangle= reduceRadian( angle[i] - angle[i-1] );
      diffangle= diffangle>0.f?diffangle:diffangle+_2PI; // force positive rotation
      Float2 position;
      position.setDirection( reduceRadian( angle[i-1] + diffangle*0.5f ) );
      position*= (obstacles[i-1].length()+obstacles[i].length())*0.5;
      
      a_map[ref]= Node2( position, Node2::type_frontier );

//       cout << "\t" << angle[i] << ", " << angle[i-1] << ": " << diffangle << ": " << reduceRadian( angle[i-1] + diffangle*0.5f );
//       cout << "-> " << ref << ": " << a_map[ref] << endl;

      boost::add_edge(ref, node[i], a_map);
      boost::add_edge(ref, node[i-1], a_map);
      
      ++count;
    
      last= i;
    }
    else if( last != i && obstacles[last].distance( obstacles[i] ) > sumRadius ){
      node[i]= boost::add_vertex( a_map );
      a_map[ node[i] ]= Node2( obstacles[i], Node2::type_obstacle );

      boost::add_edge(node[last], node[i], a_map);

      last= i;      
    }
  }

  // closse the loop:
  float distAB= obstacles[size-1].distance( obstacles[0] );

  if( distAB > sumRadius )
  {
    if( last != size-1 ){
      node[size-1]= boost::add_vertex( a_map );
      a_map[ node[size-1] ]= Node2( obstacles[size-1], Node2::type_obstacle );
      boost::add_edge( node[last], node[size-1], a_map );
    }

    ref= boost::add_vertex( a_map );
    float diffangle= reduceRadian( angle[0] - angle[size-1] );
    diffangle= diffangle>0.f?diffangle:diffangle+_2PI; // force positive rotation
    Float2 position;
    position.setDirection( reduceRadian( angle[size-1] + diffangle*0.5f ) );
    position*= (obstacles[0].length()+obstacles[size-1].length())*0.5;
    
    a_map[ref]= Node2( position, Node2::type_frontier );

//     cout << "\t" << angle[size-1] << ", " << angle[0] << ": " << diffangle << ": " << reduceRadian( angle[size-1] + diffangle*0.5f );
//     cout << "-> " << ref << ": " << a_map[ref] << endl;

    boost::add_edge(ref, node[size-1], a_map);
    boost::add_edge(ref, node[0], a_map);
      
    ++count;
  }
  else{
      boost::add_edge( node[last], node[0], a_map );
  }
  if ( count == 0 ){
    Graph2::vertex_descriptor v[3];
    v[0]= boost::add_vertex( a_map );
    a_map[v[0]]= Node2( Float2(1.f,-1.f), Node2::type_frontier );
    v[1]= boost::add_vertex( a_map );
    a_map[v[1]]= Node2( Float2(1.f,1.f), Node2::type_frontier );
    v[2]= boost::add_vertex( a_map );
    a_map[v[2]]= Node2( Float2(-1.f,0.f), Node2::type_frontier );
    
    boost::add_edge(v[0], v[1], a_map);
    boost::add_edge(v[1], v[2], a_map);
    boost::add_edge(v[2], v[0], a_map);
  }

  initialize_frame();
}

void VisiMap::frontier_regression()
{
    std::list< std::list<Float2> > grp;
    
    int size= boost::num_vertices(a_map);
    bool available [size];
    
    for( int i(0) ; i < size ; ++i )
        available[i]= true;

    Graph2::vertex_iterator vRef, vRefEnd;
    for( boost::tie(vRef, vRefEnd)= boost::vertices(a_map) ; vRef != vRefEnd ; ++vRef )
         if( available[*vRef] && a_map[*vRef].type == Node2::type_frontier )
    {
        grp.push_back( std::list<Float2>() );

        std::list< Graph2::vertex_descriptor > grpTrack;
        grpTrack.push_back( *vRef );
        available[ *vRef ]= false;

        Float2 mean( a_map[*vRef]*a_epsilon*0.1f );
        float weight( a_epsilon*0.1f );
        
        while( !grpTrack.empty() )
        {
            // get the new vertice
            Graph2::vertex_descriptor src_vd= grpTrack.front();
            grpTrack.pop_front();

            // add to grp:
            grp.back().push_back( a_map[src_vd] );

            Graph2::out_edge_iterator ed, edEnd;
            
            for( boost::tie(ed, edEnd)=boost::out_edges(src_vd, a_map) ; ed != edEnd ; ++ed )
            {
                Graph2::vertex_descriptor trg_vd= boost::target( *ed, a_map );
                
                if( a_map[trg_vd].type == Node2::type_frontier )
                {
                    float edgeWeight= a_map[src_vd].distance(a_map[trg_vd]);
                    mean+= (Float2)(a_map[src_vd] + a_map[trg_vd])*(0.5f*edgeWeight);
                    weight+= edgeWeight;
                    
                    if( available[trg_vd] )
                    {
                        grpTrack.push_back( trg_vd );
                        available[trg_vd]= false;
                    }
                }
            }
        }
        
        // reccord grp mean:
        grp.back().push_front( mean/weight );
     }
    
    // Destroy old frontier:
    list< Graph2::vertex_descriptor > ltype= getVertices(Node2::type_frontier);
    for(list< Graph2::vertex_descriptor >::reverse_iterator it(ltype.rbegin()), itEnd(ltype.rend()) ; it != itEnd ; ++it )
    {
        boost::clear_vertex(*it, a_map);
        boost::remove_vertex(*it, a_map);
    }

    // Rebuild type
    float closeDist= a_epsilon*3.5f;

    for( std::list< std::list<Float2> >::iterator igrp( grp.begin() ), igrpEnd( grp.end() ) ; igrp != igrpEnd ; ++igrp )
    {
        std::list<Float2>::iterator it( igrp->begin() ), itEnd( igrp->end() );
        
        Graph2::vertex_descriptor ref= boost::add_vertex( a_map );
        a_map[ref]= Node2( it->x, it->y, Node2::type_frontier );
                
        ++it;
        while( it != itEnd )
        {
            list<Graph2::vertex_descriptor> closes= getVertices( *it, closeDist );
            for( list<Graph2::vertex_descriptor>::iterator ic( closes.begin() ), icEnd( closes.end() ) ; ic != icEnd ; ++ic )
                if( a_map[*ic].type == Node2::type_obstacle && !boost::edge( ref, *ic, a_map ).second )
            {
                Graph2::edge_descriptor ed= boost::add_edge( ref, *ic, a_map ).first;
                a_map[ed]= Node2::type_frontier;
            }

            ++it;
        }
    }
}

void VisiMap::clean_frontier(){
    // Remove uncoherent frontier edge and vertices.
    
    // Remove frontier edge connected to un obstacle vertice knowing its 2 obstacle edge.
    {
        Graph2::vertex_iterator iv, ivEnd;
        for( boost::tie(iv, ivEnd)= boost::vertices(a_map); iv != ivEnd ; ++iv )
            if( a_map[*iv].type == Node2::type_obstacle ) 
        {
            int countF= 0;
            // obstacle with 2 frontier ?
            Graph2::out_edge_iterator ie, ieEnd;
            for( boost::tie(ie, ieEnd)= boost::out_edges(*iv, a_map); ie != ieEnd ; ++ie )
                if( a_map[ boost::target(*ie, a_map) ].type == Node2::type_obstacle ) 
                    ++countF;
            
//             cout << "obstacle " << *iv << ": " << countF << " edges " << endl;
            
            if( countF > 1 )// if regular obstacle vertice, it does'nt have frontier :
            {
                boost::tie(ie, ieEnd)= boost::out_edges(*iv, a_map);
                while( ie != ieEnd )
                {
                    if( a_map[ boost::target(*ie, a_map) ].type == Node2::type_frontier )
                    {
//                         cout << "\t remove " << *ie << " to " << boost::target(*ie, a_map) << endl;
                        boost::remove_edge( *ie, a_map );
                        boost::tie(ie, ieEnd)= boost::out_edges(*iv, a_map);
                    }
                    ++ie;
                }
            }
        }
    }

    // Remove frontier vertice if their connect less than 2 other vertices.
    {
        Graph2::vertex_iterator iv, ivEnd;
        boost::tie(iv, ivEnd)= boost::vertices(a_map);
        while( iv != ivEnd)
        {
            if( a_map[*iv].type == Node2::type_frontier && boost::out_degree(*iv, a_map) < 2 ) 
            {
                boost::clear_vertex(*iv, a_map);
                boost::remove_vertex(*iv, a_map);
                boost::tie(iv, ivEnd)= boost::vertices(a_map);
            }
            else ++iv;
        }
    }
}

int VisiMap::obstacle_edge_regression()
{
    int removed= 0;
    
    Graph2::vertex_iterator vi, viEnd;
    Graph2::edge_iterator ei, eiEnd;
    Float2 colPoint; 

    // If an edge pass at epsilon distance to a node :
    for( boost::tie(vi, viEnd)= boost::vertices(a_map) ; vi != viEnd ; ++vi )
        if( a_map[*vi].type == Node2::type_obstacle )
    {
        boost::tie(ei, eiEnd)= boost::edges(a_map);
        while( ei != eiEnd )
        {
            Graph2::vertex_descriptor src( boost::source(*ei, a_map) ), trg( boost::target(*ei, a_map) );
            if( src != *vi && trg != *vi && a_map[src].type == Node2::type_obstacle 
		&& impact(Segment2(a_map[src], a_map[trg]), Circle2(a_map[*vi], a_epsilon), colPoint) )
            {   
                boost::remove_edge( *ei, a_map );
                ++removed;
                
                if( ! boost::edge(src, *vi, a_map).second )
                {
                    boost::add_edge(src, *vi, a_map);
                    --removed;
                }
                
                if( ! boost::edge(*vi, trg, a_map).second )
                {
                    boost::add_edge(*vi, trg, a_map);
                    --removed;
                }
                
                boost::tie(ei, eiEnd)= boost::edges(a_map);
            }
            else ++ei;
        }
    }

    // If a node has more than 2 obstacle edges with short edges :
    for( boost::tie(vi, viEnd)= boost::vertices(a_map) ; vi != viEnd ; ++vi )
        if( a_map[*vi].type == Node2::type_obstacle )
    {

        list< valued<int> > neibourg;
        Graph2::out_edge_iterator oei, oeiEnd;
        int size= 0;
        
        for( tie(oei, oeiEnd)= boost::out_edges(*vi, a_map) ; oei != oeiEnd; ++oei )
        {
            int trg= boost::target(*oei, a_map);
            if( a_map[ trg ].type == Node2::type_obstacle )
            {
                neibourg.push_back( valued<int>( trg, 0.f ) );
                ++size;
            }
        }
        
        if( size > 2 )
        {
            float eps22= 2.f*a_epsilon;
            eps22= eps22*eps22;
            
            for( list< valued<int> >::iterator it(neibourg.begin()), itEnd(neibourg.end()) ; it != itEnd ; ++it )
                it->value= a_map[*vi].distance2(a_map[it->item] );
            
            neibourg.sort();
            while( neibourg.front().value < eps22 && size > 2 )
            {   
                boost::remove_edge( *vi, neibourg.front().item, a_map );
                removed++;
                neibourg.pop_front();
                --size;
            }
        }
    }
    
//    cout << "\t" << removed << " edges removed" << endl;
    return removed;
}

int VisiMap::obstacle_node_regression()
{
    int removed= 0;
    Float2 rescol;
    
    // Search for plate triangles
    Graph2::vertex_iterator vi, viEnd;
    for( tie(vi, viEnd)= boost::vertices(a_map) ; vi != viEnd ; ++vi )
    {
        bool next= false;
        while( !next )
        {
            next= true;
            Graph2::vertex_iterator vi2= vi;
            ++vi2;
            if( vi2 != viEnd && a_map[*vi2].type == Node2::type_obstacle )
            {
                int neibourg[2];
                int size= 0;
                Graph2::out_edge_iterator oei, oeiEnd;
        
                for( tie(oei, oeiEnd)= boost::out_edges(*vi2, a_map) ; size < 2 && oei != oeiEnd; ++oei )
                {
                    neibourg[size]= boost::target(*oei, a_map);
                    if( a_map[ neibourg[size] ].type == Node2::type_obstacle )
                        ++size;
                }
                
                if( size == 2 && impact(Segment2(a_map[ neibourg[0] ], a_map[ neibourg[1] ]), Circle2(a_map[*vi2], a_epsilon), rescol) )
                {
                    if( ! boost::edge( neibourg[0], neibourg[1], a_map ).second )
                        boost::add_edge( neibourg[0], neibourg[1], a_map );
                    
                    //cout << "\tremove node: "<< *vi2 << endl;
                    boost::clear_vertex( *vi2, a_map );
                    boost::remove_vertex( *vi2, a_map );
                    
                    next= false;
                    ++removed;
                }
            }
        }
    }

//    cout << "\t" << removed << " nodes removed" << endl;
    return removed; 
}

bool VisiMap::replace_if_posible( int test, int pivot )
{
    Graph2::out_edge_iterator it, itEnd;
    boost::tie(it, itEnd)= boost::out_edges(test, a_map);
    
    if( it != itEnd )
    {
        int ref= boost::target(*it, a_map);
        ++it;

        Float2 res;
        
        if( it==itEnd && impact(Segment2(a_map[pivot], a_map[ref]), Circle2(a_map[test], a_epsilon), res) 
	  
	)
        {
            boost::add_edge(pivot, ref, a_map);

            boost::clear_vertex(test, a_map);
            boost::remove_vertex(test, a_map);
            return true;
        }
    }
    return false;
}

// std::list<std::pair<Float2,Float2>> VisiMap::list_frontier()const
// {
//     int numv( boost::num_vertices(a_map) );
//     int group[numv];
//     
//     // initialize empty group:
//     for( int i(0) ; i < numv ; ++i )
//         group[i]= 0;
// 
//     // Build group :
//     int grp(1);
//     for( int iRef(0) ; iRef < numv ; ++iRef )
//         if( group[iRef] == 0 && a_map[iRef].type == Node2::type_frontier )
//     {
//         
//         list<int> grpVertices;
//         grpVertices.push_back( iRef );
//         
//         while( !grpVertices.empty() )
//         {
//             int ref= grpVertices.
//             group[iRef]= grp;
//             boost::out_edges<>()
//         }
//         
//         ++grp;
//     }
// }

// Fusion:
void VisiMap::add( const VisiMap & vm, const Transform & tr21 )
{
    int numv2= boost::num_vertices( vm.a_map );
    int match[ numv2 ];
    Graph2::vertex_descriptor node21[ numv2 ];
    float eps2= a_epsilon*a_epsilon;
    Float2 collision;
    
    // No match :
    for( int i(0) ; i < numv2 ; ++i )
        match[i]= 0; // no match

    // Search for match between nodes :
    {
        Graph2::vertex_iterator vi1, vi1End, vi2, vi2End;
        for( boost::tie(vi2, vi2End)= boost::vertices(vm.a_map) ; vi2 != vi2End ; ++vi2 )
            for( boost::tie(vi1, vi1End)= boost::vertices(a_map) ; match[ *vi2 ]== 0 && vi1 != vi1End ; ++vi1 )
        {
            if( a_map[*vi1].type == vm.a_map[*vi2].type && a_map[*vi1].distance2(tr21(vm.a_map[*vi2]) ) < eps2 )
            {
                match[ *vi2 ]= 1;
                node21[ *vi2 ]= *vi1;
            }
        }
    }
    
    // Generate nodes :
    cout << "match v_2xv_1 : [ ";
    for( int i(0) ; i < numv2 ; ++i )
    {
        if( match[i] == 0 )
        {
            Graph2::vertex_descriptor v= boost::add_vertex( a_map);
            a_map[v]= tr21(vm.a_map[i]);
            a_map[v].type= vm.a_map[i].type;
            
            match[i]= 2;
            node21[i]= v;
        }
        else
            cout << '(' << i << ", " << node21[i] << ") ";
        
    }
    cout << "]" << endl;
    
    // Generate edges :
    {
        Graph2::edge_iterator ei2, ei2End;
        for( boost::tie(ei2, ei2End)= boost::edges(vm.a_map) ; ei2 != ei2End ; ++ei2 )
        {
            int src2( boost::source(*ei2, vm.a_map) ), trg2( boost::target(*ei2, vm.a_map) );
            if( ! boost::edge( node21[src2], node21[trg2], a_map ).second )
                boost::add_edge( node21[src2], node21[trg2], a_map);
        }
    }

    // Clean :
    do
    {
        obstacle_edge_regression();
    }while( obstacle_node_regression() );
    
    // Actualize Meta : //TODO do it more properly...
    a_origin.x= min(a_origin.x, vm.a_origin.x);
    a_origin.y= min(a_origin.y, vm.a_origin.y);
    a_size.x= max(a_size.x, vm.a_size.x);
    a_size.y= max(a_size.y, vm.a_size.y);
    
};

// File interaction:
void VisiMap::load( const std::string & file )
{
    a_map.clear();

    cout << "read map from: " << file;
    
    std::ifstream is;
    is.open( file.c_str(), std::ios::in );
    
    unsigned int nbNode, nbEdge, n1, n2;
    Float2 position;
    int type;
    
    is >> a_origin.x >> a_origin.y >> a_size.x >> a_size.y;
    is >> nbNode >> nbEdge >> a_epsilon;

    cout << "\n\torigin: " << a_origin << ", size: " << a_size;
    cout << "\n\tgraph: " << nbNode << ", " << nbEdge << endl;
    
    for( unsigned int i(0) ; i < nbNode ; ++i )
    {
        is >> position.x >> position.y >> type;
        assert( i == boost::add_vertex( a_map) );
        a_map[i]= position;
        a_map[i].type= (Node2::EType)type;
    }

    for( unsigned int i(0) ; i < nbEdge ; ++i )
    {
        is >> n1 >> n2;
        boost::add_edge( n1, n2, a_map );
    }
    
    is.close();
}

void VisiMap::save( const std::string & file )const
{
    ofstream os;
    os.open( file, std::ofstream::out );

    int vSize( boost::num_vertices(a_map) );
    int eSize( boost::num_edges(a_map) );
    
    os << a_origin.x << " " << a_origin.y << " " << a_size.x << " " << a_size.y << "\n";
    os << vSize << " " << eSize << " " << a_epsilon << "\n";
    
    if( vSize > 0 )
    {
        Graph2::vertex_iterator it, itEnd;
        boost::tie(it, itEnd)= boost::vertices(a_map);
        
        os << a_map[*it].x << " " << a_map[*it].y << " " << a_map[*it].type;

        int i(0);
        for( ++it; it!=itEnd ; ++it )
        {
            ++i;
            assert( i == (int)(*it) );
            os << " " << a_map[*it].x << " " << a_map[*it].y << " " << a_map[*it].type;
        }
        os << "\n";
    }
    
    if( eSize > 0 )
    {
        Graph2::edge_iterator it, itEnd;
        boost::tie(it, itEnd)= boost::edges(a_map);

        os << boost::source(*it, a_map) << " " << boost::target(*it, a_map);

        for( ++it; it!=itEnd ; ++it )
        {
            os << " " << boost::source(*it, a_map) << " " << boost::target(*it, a_map);
        }
        os << "\n";
    }

    os.close();
}

// Data interaction:
void VisiMap::load( const mia::Data &  d ){
    a_map.clear();

    cout << "Load map from data: ";
    
    a_origin.x= d.value(0);
    a_origin.y= d.value(1);
    a_size.x= d.value(2);
    a_size.y= d.value(3);
    a_epsilon= d.value(4);
    unsigned int vSize= (d.value_size()-5)/2;
    unsigned int eSize= (d.flag_size()-vSize)/2;

    cout << " graph: " << vSize << ", " << eSize << endl;
    
    //Vertices :
    for( unsigned int iv(0) ; iv < vSize ; ++iv )
    {
        assert( iv == boost::add_vertex( a_map) );
      
        a_map[iv]= Float2( d.value(5+iv*2),  d.value(6+iv*2) );
        a_map[iv].type= (Node2::EType) d.flag(iv);
    }

    for( unsigned int ie(0) ; ie < eSize ; ++ie )
    {
        boost::add_edge( d.flag( vSize+ie*2 ), d.flag( vSize+1+ie*2 ), a_map );
    }
}

mia::Data VisiMap::save()const{
    int vSize( boost::num_vertices(a_map) );
    int eSize( boost::num_edges(a_map) );
    
    mia::Data d( "visimap", vSize+(eSize*2), 5+vSize*2 );
    
    d.value(0, a_origin.x);
    d.value(1, a_origin.y);
    d.value(2, a_size.x);
    d.value(3, a_size.y);
    d.value(4, a_epsilon);
    
    //Vertices :
    int iv(0);
    {
        Graph2::vertex_iterator it, itEnd;
        for( boost::tie(it, itEnd)= boost::vertices(a_map); it!=itEnd ; ++it ){
            d.value(5+iv*2, a_map[*it].x);
            d.value(6+iv*2, a_map[*it].y);
            d.flag(iv, a_map[*it].type);
            ++iv;
        }
    }
    
    //Edges :
    int ie(0);
    {
        Graph2::edge_iterator it, itEnd;
        for( boost::tie(it, itEnd)= boost::edges(a_map); it!=itEnd ; ++it )
        {
            d.flag(iv+ie*2, boost::source(*it, a_map) );
            d.flag(iv+ie*2+1, boost::target(*it, a_map) );
            ++ie;
        }
    }
    
    return d;
}

mia::Data VisiMap::save2()const{
    int vSize( boost::num_vertices(a_map) );
    int eSize( boost::num_edges(a_map) );
    
    mia::Data d( "vmap", Node2::type_size+(eSize*2), 5+vSize*2 );
    
    d.a_value[0]= a_origin.x;
    d.a_value[1]= a_origin.y;
    d.a_value[2]= a_size.x;
    d.a_value[3]= a_size.y;
    d.a_value[4]= a_epsilon;
    
    //Vertices :
    int* vid01= new int[vSize];
    int* vid10= new int[vSize];
    int* typeSize= new int[Node2::type_size];
    
    sortVerticesByType(vid01, vid10, typeSize);

    for( int it(0); it < Node2::type_size ; ++it )
        d.flag( it, typeSize[it] );

    for( int iv(0); iv < vSize ; ++iv )
    {
        d.a_value[5+iv*2]= a_map[vid10[iv]].x;
        d.a_value[6+iv*2]= a_map[vid10[iv]].y;
    }

    //Edges :
    {
        int ie(0);
        Graph2::edge_iterator it, itEnd;
        for( boost::tie(it, itEnd)= boost::edges(a_map); it!=itEnd ; ++it )
        {
            d.a_flag[Node2::type_size+ie*2]= vid01[ boost::source(*it, a_map) ];
            d.a_flag[Node2::type_size+ie*2+1]=  vid01[ boost::target(*it, a_map) ];
            ++ie;
        }
    }
    
    delete vid01;
    delete vid10;
    delete typeSize;
    
    return d;
}

void VisiMap::sortVerticesByType(int* v01, int* v10, int * tSize)const{
    int vSize( boost::num_vertices(a_map) );
    int iv= 0;
    for( int iType= 0; iType < Node2::type_size ; ++iType )
    {
        tSize[iType]= 0;
        for( int jv= 0; jv < vSize; ++jv )
            if( a_map[jv].type == iType )
            {
                v01[jv]= iv;
                v10[iv]= jv;
                ++iv;
                ++tSize[iType];
            }
    }
}

// Data interaction:
void VisiMap::load2( const mia::Data &  d ){
    a_map.clear();

    cout << "Load map from data: ";
    
    a_origin.x= d.value(0);
    a_origin.y= d.value(1);
    a_size.x= d.value(2);
    a_size.y= d.value(3);
    a_epsilon= d.value(4);
    
    unsigned int vSize= (d.value_size()-5)/2;
    unsigned int eSize= (d.flag_size()-Node2::type_size)/2;

    cout << " graph: " << vSize << ", " << eSize << endl;
    
    //Vertices :
    int type= 0;
    unsigned int typeTold= d.flag(type);
    for( unsigned int iv(0) ; iv < vSize ; ++iv )
    {
        assert( iv == boost::add_vertex( a_map) );
        
        a_map[iv]= Float2( d.value(5+iv*2),  d.value(6+iv*2) );
        
        if( iv == typeTold )
        {
            ++type;
            typeTold+= d.flag(type);
        }
        a_map[iv].type= (Node2::EType)type;
    }

    for( unsigned int ie(0) ; ie < eSize ; ++ie )
    {
        boost::add_edge( d.flag( Node2::type_size+ie*2 ), d.flag( Node2::type_size+1+ie*2 ), a_map );
    }
}

void VisiMap::toSVG(ostream& os)const
{
    os << "<svg width=\"" << a_size.x << "\" height=\"" << a_size.y << "\">\n";
    
    //Obstacle edges:
    os << "<g  stroke=\"red\" stroke-width=\"" << a_epsilon << "\" fill=\"none\" >\n";

    {
        Graph2::edge_iterator it, itEnd;
        for( boost::tie(it, itEnd)= boost::edges(a_map); it!=itEnd ; ++it )
        {
            Node2 src= a_map[ boost::source(*it, a_map) ];
            Node2 trg= a_map[ boost::target(*it, a_map) ];
	    if( src.type == Node2::Node2::type_obstacle && trg.type == Node2::Node2::type_obstacle )
	      os << "\t\t<path  d=\"M" << src.x - a_origin.x  << " " << a_size.y - (src.y - a_origin.y)
                << " L" << trg.x - a_origin.x << " " << a_size.y - (trg.y - a_origin.y) << "\" />\n";
        }
    }
    os << "\t</g>\n";
    
    //Frontier edges:
    os << "<g  stroke=\"green\" stroke-width=\"" << a_epsilon << "\" fill=\"none\" >\n";
    {
        Graph2::edge_iterator it, itEnd;
        for( boost::tie(it, itEnd)= boost::edges(a_map); it!=itEnd ; ++it )
        {
            Node2 src= a_map[ boost::source(*it, a_map) ];
            Node2 trg= a_map[ boost::target(*it, a_map) ];
	    if( src.type == Node2::Node2::type_frontier || trg.type == Node2::Node2::type_frontier )
	      os << "\t\t<path  d=\"M" << src.x - a_origin.x  << " " << a_size.y - (src.y - a_origin.y)
                << " L" << trg.x - a_origin.x << " " << a_size.y - (trg.y - a_origin.y) << "\" />\n";
        }
    }
    os << "\t</g>\n";
    os << "</svg>" << endl;
}
