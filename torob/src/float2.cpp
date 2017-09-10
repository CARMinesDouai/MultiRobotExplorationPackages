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

#include "float2.h"
#include "tools.h"
#include <boost/concept_check.hpp>

using namespace mia;
using namespace std;

Float2 Float2 :: middle( const Float2 & A, const Float2 & B )
{
  return (A+B)*0.5f;
}

Float2 Float2 :: mean( const std::list<Float2> & lFloat2 )
{
    Float2 mean;
    int size(0);
        
    for( std::list<Float2>::const_iterator it= lFloat2.begin() ; it != lFloat2.end(); ++it ){
        mean+= *it;
        ++size;
    }
    return mean / (float)size;
}

std::array<Float2, 2> Float2::simpleLinearRegression( const std::list<Float2> & point )
{
    int size(0);
    Float2 mean(0.f, 0.f), origin(0.f, 0.f);
    
    for( std::list<Float2>::const_iterator it(point.begin()), itEnd(point.end()) ; it != itEnd ; ++it )
    {
        mean+= Float2(it->x, it->y);
        ++size;
    }
    mean/= (float)size;

    float slopeNum(0.f), yxSlopeDenum(0.f), xySlopeDenum(0.f);
    for( std::list<Float2>::const_iterator it(point.begin()), itEnd(point.end()) ; it != itEnd ; ++it )
    {
        float dx= (it->x-mean.x);
        float dy= (it->y-mean.y);
        
        slopeNum+= dx*dy;
        
        yxSlopeDenum+= dx*dx;
        xySlopeDenum+= dy*dy;
        
        ++size;
    }

    if( yxSlopeDenum == 0 && xySlopeDenum == 0 )
    {
        origin.x= mean.x;
        origin.y= 0.f;
    }
    else if( yxSlopeDenum*yxSlopeDenum > xySlopeDenum*xySlopeDenum )
    {
        float slope= slopeNum / yxSlopeDenum;
        origin.x= 0;
        origin.y= mean.y - slope * mean.x;
    }
    else
    {
        float slope= slopeNum / xySlopeDenum;
        origin.y= 0;
        origin.x= mean.x - slope * mean.y;
    }
    
    std::array<Float2, 2> descriptor= {mean, Float2(origin, mean)};
    descriptor[1].normalize();
    
    return descriptor;
}


std::array<Float2, 2> Float2::projectionSegment( const std::list<Float2> & point, const Float2 &mean, const Float2 &normDir )
{
    list<float> projection;
    for( list<Float2>::const_iterator it(point.begin()), itEnd(point.end()) ; it!=itEnd ; ++it )
        projection.push_back( dotProduct( (*it) - mean, normDir ) );
    projection.sort();

    array<Float2, 2> segment= { normDir * *(projection.begin()) + mean, normDir * *(projection.rbegin()) + mean };
    return segment;
}

bool Float2::validSegmentRegression( const std::list<Float2> & lFloat2, const std::array<Float2, 2> & normSegment, float treshold )
{
    Float2 normDir= normSegment[1].orthogonal();
    bool valid(true);
    
    for( list<Float2>::const_iterator it(lFloat2.begin()), itEnd(lFloat2.end()) ; valid && it!=itEnd ; ++it )
        valid= dotProduct( (*it) - normSegment[0], normDir ) < treshold;
    
    return valid;
}

list<Float2> Float2 :: polarSort(const list<Float2> & lFloat2)
{
    list< valued<Float2> > toSort;    
    for(list<Float2>::const_iterator it = lFloat2.begin(); it != lFloat2.end() ; ++it )
        toSort.push_back( valued<Float2>( *it, it->angle() ) ); 
    toSort.sort();
    
    list<Float2> ret;
    for(list<valued<Float2>>::const_iterator it = toSort.begin(); it != toSort.end() ; ++it )
        ret.push_back( it->item );
    
    return ret;
}


Transform Transform::from_match ( std::list<std::pair<Float2, Float2>>::const_iterator itBegin,
                                     std::list<std::pair<Float2, Float2>>::const_iterator itEnd )
{ // Which translation / rotation to transfom second to first ?
    Transform t;
    t.translation= Float2(0.f, 0.f);
    t.center= Float2(0.f, 0.f);
    t.rotation= 0.f;
    int size(0);

    for( std::list<std::pair<Float2, Float2>>::const_iterator it= itBegin ; it != itEnd ; ++it )
    {// Get center :
        t.center+= it->first;
        t.translation+= it->second;
        ++size;
    }
    t.center/= (float)size; // Center first
    t.translation/= (float)size; // Center second
    t.translation= t.center - t.translation; // translation from second to first.
    
    // Rotation :
    for( std::list<std::pair<Float2, Float2>>::const_iterator it= itBegin ; it != itEnd ; ++it )
    {
        t.rotation+= angle( it->second + t.translation, t.center, it->first );
    }
    t.rotation= reduceRadian( t.rotation/(float)size );
    
    return t;
}
