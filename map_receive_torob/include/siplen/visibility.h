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

#ifndef MIA_VISIBILITY_H
#define MIA_VISIBILITY_H

#include "graph.h"
#include "localmodel.h"
#include "particle.h"
#include "link.h"

namespace mia {

typedef std::pair< BoostTopo::vertex_descriptor, int > pairOFvertices;

class Visibility
{
public:
    
    class Node {
    public :
        Float2 position;
        int weight;
    };

    class Edge {
    public :
        Edge(int f= 0, int w= 0):flag(f),weight(w){}
        Edge(const Edge & e):flag(e.flag),weight(e.weight){}
        virtual ~Edge(){}
        
        int flag;
        int weight;
    };

    const LocalModel * a_ptrlocal;
    undirected_graph<Node, Edge> a_visibility;

    std::list< pairOFvertices > a_loc2vis;

    int a_learnTampon;
    float a_epsilon;
    const float a_maximun, a_2maximun, a_maximun2, a_2maximun2;

    enum EEdgeFlag
    {
            edge_undef= 0,
            edge_obstacle,
            edge_viewline,
            
            edge_number
    };

public:
    Visibility(float eps, float max);
    
    // Getter and setter :
    //--------------------
    virtual float getEpsilon()const{ return a_epsilon; }
    virtual float setEpsilon(float eps)
    {
        a_epsilon= eps;
        clean();
    }
    virtual float getMaximun()const{ return a_maximun; }
    
    virtual std::pair<bool, int> local2visibility( BoostTopo::vertex_descriptor lv ) const;
    virtual std::pair<bool, BoostTopo::vertex_descriptor> visibility2local( int vv ) const;

    virtual std::list<int> get_obstacle()const;
    virtual std::list<int> get_obstacle( const Float2 & pos, float radius )const;
    virtual std::list<int> get_obstacle(int src, int trg)const;

    virtual int add_limit(const Float2 & position);
    virtual int add_corner(const Float2 & position);

    virtual int add_edge(int src, int trg, int flag);
    virtual std::list<int> add_obstacle(int src, int trg);
    virtual int add_viewline(int src, int trg);

//    virtual void eraseUnreachable();

    virtual void clear();
    virtual void clean();
    virtual void update_around(const Float2 & position);
    virtual void actualize( const LocalModel * ptrlocal, const Float2 & position );

    virtual int actualizeVertex( const std::list<int> & visibilityNodes, const BoostTopo::vertex_descriptor & localVertex );
    virtual void replaceVisInLoc2Vis( int oldIndex, int newIndex );
    
    virtual void ostreamLoc2Vis ( std::ostream & os );
    virtual void print( std::ostream & os )const;
    
    virtual void load( const std::string &  file );
//    virtual void save( const std::string &  file ){};
};
    std::ostream & operator<<( std::ostream & os, const mia::Visibility & );
    
};


#endif // MIA_VISIBILITY_H
