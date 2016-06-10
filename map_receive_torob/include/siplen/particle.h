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

#ifndef MIA_PARTICLE_H
#define MIA_PARTICLE_H

#include "float2.h"
#include "data.h"

#include <vector>
#include <boost/concept_check.hpp>

namespace mia
{
    
class Particle
{
public:
    // Definition
    Float2 position;
    float theta;
    float radius;

    // Attributs :
    int flag;
    
    // Dynamic :
    Particle * ptrDynamic;

    /// Contructor / Destructor :
    ///--------------------------
    Particle ( const Float2 & p= 0.f, float t= 0.f, float r= 0.f, int f= 0,
               const Particle * ptS= nullptr );

    Particle ( const Float2 & p, float t, float r, int f, bool mobile );
    
    Particle ( const Particle & p );
    Particle ( const Data & p );
    
    virtual ~Particle();

    /// Geter / seter :
    ///----------------
    virtual bool isMobile()const{ return (ptrDynamic != nullptr); }
    
    /// Transformation :
    ///-----------------
    virtual Particle fromBasis( const Particle & basis )const;
    virtual Particle toBasis( const Particle & basis )const;
    
    // Assignement :
    //--------------
    virtual const Particle &operator=( const Particle & p );
    virtual const Particle &operator=( const Data & d ) {
         return operator=( Particle(d) );
    }

    // Data manadgment :
    //------------------
    Data toData()const;

    // Comparison      :
    //------------------
    virtual bool operator==( const Particle & p )const;

    /// Process Dynamiq :
    ///------------------
    virtual void move(float time);
    
    static void sortAngle( std::vector<Particle> &tab ); 
    static std::list<Float2> polarFiltering( const std::vector<Particle> &tab, float eps );
};

    //const Data & operator=( Data & d, const Particle &p );
    std::ostream & operator<< ( std::ostream & os, const mia::Particle & p );
    bool collision( const Particle &p1, const Particle &p2 );
    bool collisionParticleLine( const Particle &p, const Float2 &A, const Float2 &B );
};


#endif // PARTICLE_H
