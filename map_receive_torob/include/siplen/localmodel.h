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

#ifndef MIA_LOCALMODEL_H
#define MIA_LOCALMODEL_H

// Boost :
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include "particle.h"
#include "link.h"


namespace mia {

typedef boost::adjacency_list<  // adjacency_list est un template qui dépend de :
    boost::listS,               //  le conteneur utilisé pour les arcs partant d'un sommet. Ici, std::list.
    boost::vecS,                //  le conteneur utilisé pour contenir tous les sommets du graphe. Ici, std::vector.
    boost::undirectedS,         //  le type des arcs. Pourrait être boost::directedS ou boost::undirectedS.
    Particle,                   //  le type qui décrit un sommet.
    Link                        //  le type qui décrit un arc.
> BoostTopo;

class Visibility;

class LocalModel
{
    friend Visibility;

protected :
    Particle a_body;
    float a_epsilon, a_safeDistance;
    BoostTopo::vertex_descriptor a_agentVD;

public:
    BoostTopo a_local;

    enum EParticleFlag
    {
            particle_undef= 0,
            particle_obstacle,
            particle_hide_obstacle,
            particle_safe,
            particle_exit,
            particle_waypoint,
            
            particle_number
    };

    enum ELinkFlag
    {
            link_undef= 0,
            link_obstacle,
            link_hide_obstacle,
            link_safe,
            link_exit,
            link_path,
            
            link_number
    };

    LocalModel();
    ~LocalModel();
   
    // Getter and setter :
    //--------------------
    virtual std::pair<Particle, bool> getClosest( const Float2 & position )const;
    virtual std::pair<Particle, bool> getClosestSafe( const Float2 & position )const;
    virtual std::pair<Particle, bool> getDirection( const Float2 & position, EParticleFlag flag )const;
    virtual std::pair<Particle, bool> getDirectionSafe( const Float2 & position )const;

    virtual void initialize( const Particle & body, const std::vector<Particle> &obstacle, const std::list<int> &edgeList, float epsilon, float safeDistance );
    virtual void actualize();

    virtual BoostTopo::vertex_descriptor addLinkedParticle(const Float2 & position, EParticleFlag pflag, BoostTopo::vertex_descriptor ref, ELinkFlag lflg );
    virtual std::pair<BoostTopo::vertex_descriptor, bool> addLocalSafeParticle(const Float2 & position, BoostTopo::vertex_descriptor ref, bool exit );

protected :

    virtual void erraseUnreachableExit();
    virtual void buildVisibility();

};

};
#endif // MIA_LOCALMODEL_H
