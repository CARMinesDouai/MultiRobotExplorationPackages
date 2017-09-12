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

#ifndef VISIMAP_H
#define VISIMAP_H

//mia: 
#include "tools.h"
//#include "link.h"
#include "ogmap.h"

// Boost:
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

namespace mia{

class Node2 : public Float2
{
public:
    enum EType{
        type_free= 0,
        type_obstacle,
        type_frontier,
        type_size
    };

    EType type;
    
    Node2():Float2(),type(type_free){}
    Node2(float x, float y, EType t= type_free):Float2(x, y),type(t){}
    Node2(Float2 f2, EType t= type_free):Float2(f2),type(t){}
    
    Node2 & operator=(const Node2 & n)
    {
        x= n.x;
        y= n.y;
        type= n.type;
        return *this;
    }
    
    Node2 & operator=(const Float2 & n)
    {
        x= n.x;
        y= n.y;
        return *this;
    }
};

typedef boost::adjacency_list<  // adjacency_list est un template qui dépend de :
    boost::listS,               //  le conteneur utilisé pour les arcs partant d'un sommet. Ici, std::list.
    boost::vecS,                //  le conteneur utilisé pour contenir tous les sommets du graphe. Ici, std::vector.
    boost::undirectedS,         //  le type des arcs. Pourrait être boost::directedS ou boost::undirectedS.
    Node2,                      //  le type qui décrit un sommet.
    Node2::EType                //  le type qui décrit un arc.
> Graph2;

class VisiMap
{
public:
    // topology:
    float a_epsilon;
    Graph2 a_map;

    // meta:
    Float2 a_origin, a_size;

public:
    // Constructor/Destructor:
    VisiMap();
    VisiMap(const VisiMap &vm);
    virtual ~VisiMap();

    // Agent Function:

    //getter:
    virtual float getEpsilon()const{ return a_epsilon; }
    virtual Float2 getRefPosition()const{ return Float2( a_map[0].x, a_map[0].y ); }
    virtual std::list<Graph2::vertex_descriptor> getVertices( Node2::EType type )const;
    virtual std::list<Graph2::vertex_descriptor> getVertices( Float2 pos, float dist )const;

    virtual std::list<Graph2::edge_descriptor> getEdges( Node2::EType srcT, Node2::EType trgT )const;
    
    virtual int verticesSize(){ return boost::num_vertices(a_map); };
    virtual int edgesSize(){ return boost::num_edges(a_map); };
    
    //setter:
    virtual void setRefPosition(const Float2 & pos){ a_map[0]= Node2(pos, Node2::type_free); }
    virtual void setEpsilon(float esp){ a_epsilon= esp; }
    
    virtual void initialize_frame();// Generate origine and size frame coordinates.

    // Node managment :
    virtual void remove_node(Node2::EType type);
    
    // 2D transformation :
    virtual void transform( const Transform & t );
    
    // Simple goal detection :
    virtual Float2 goal_closest( const Float2 &ref )const;
    virtual inline Float2 goal_closest()const{ return goal_closest( getRefPosition() ); }
    virtual Float2 goal_random()const;
    virtual Float2 goal_largest()const;
    virtual Float2 goal_largest_random( int range )const; 

    // All goals:
//    virtual std::list<std::pair<Float2,Float2>> list_frontier()const;
    virtual std::list<valued<Float2>> list_goal()const;
    
    // Initialization:
    virtual void initialize( const OGMap & grid );
    virtual void initialize( const OGMap & grid, Float2 refposition, float epsilon );
    virtual void initialize( const std::list<Float2> & scan, Float2 refposition, float epsilon );
    virtual void generation_from_grid( const OGMap & grid, int type_ref, int type_target, Node2::EType nodeType, float gridEps= 0.70f );
    virtual void frontier_regression();
    virtual void clean_frontier();// Remove uncoherent frontier edge and vertices.
    virtual int obstacle_edge_regression();
    virtual int obstacle_node_regression();
    virtual void obstacle_regression(){ do{ obstacle_edge_regression(); }while( obstacle_node_regression() ); }
    
    // Initialization Tools:
    virtual bool replace_if_posible( int test, int pivot );
    
    // Fusion:
    virtual void add( const VisiMap & vm, const Transform & t );
    
    // File interaction:
    virtual void load( const std::string &  file );
    virtual void save( const std::string &  file )const;
    
    // Data interaction:
    virtual void load( const mia::Data &  d );
    virtual mia::Data save()const;
    virtual void load2( const mia::Data &  d );
    virtual mia::Data save2()const;
    virtual void toSVG( std::ostream & os )const;
    
    // Vertices transformation:
    virtual void sortVerticesByType(int*, int*, int*)const;
};
};

#endif // VISIMAP_H
