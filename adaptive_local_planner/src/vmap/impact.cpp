/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
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

#include "impact.h"

#include <boost/geometry/algorithms/intersection.hpp> 
#include <list>

using namespace mia;

bool mia::impact(const Segment2 & s1, const Segment2 & s2)
{
  std::list<Float2> X;
  return ( boost::geometry::intersection(s1, s2, X) && !X.empty() );
}

bool mia::impact(const Segment2 & s1, const Segment2 & s2, Float2 & point)
{
  std::list<Float2> X;
  if( boost::geometry::intersection(s1, s2, X) && !X.empty() )
  {
    point= *(X.begin());
    return true;
  }
  return false;
}

bool mia::impact(const Circle2 & c1, const Circle2 & c2)
{
    Float2 between= c2.center - c1.center;
    float distMin= c1.radius + c2.radius;

    return between.length2() < (distMin*distMin);
}

bool mia::impact(const Circle2 & c1, const Circle2 & c2, Float2 & point)
{
    Float2 between= c2.center - c1.center;
    float distMin= c1.radius + c2.radius;
    bool out(false);
    
    if( between.length2() < (distMin*distMin) )
    {
      point= c2.center + (between*0.5f);
      out= true;
    }
    return out;
}

bool mia::impact(const Float2 & v, const Circle2 & c){
  Float2 d(v - c.center);
  return d.length2() < (c.radius*c.radius);
}

bool mia::impact(const Segment2 & s, const Circle2 & c){
  Float2 out;
  return impact(s, c, out);
}

bool mia::impact(const Segment2 & s, const Circle2 & c, Float2 & point){
    
    Float2 radius= s.b - s.a;
    radius.orthonormalize(c.radius);
    Segment2 diametre( c.center - radius, c.center + radius );
    
    if ( impact( s, diametre, point ) )
    {
        return true;
    }

    if( impact( s.a, c ) )
    {
        point= s.a;
        return true;
    }

    if( impact( s.b, c ) )
    {
        point= s.b;
        return true;
    }

    return false;
}

bool mia::impact(const Polygon2 & p1, const Polygon2 & p2)
{
  std::list<Polygon2> output;
  return (boost::geometry::intersection(p1, p2, output) && !output.empty() );
}

bool mia::impact(const Polygon2 & p1, const Polygon2 & p2, std::list<Polygon2> & output)
{
  return (boost::geometry::intersection(p1, p2, output) && !output.empty() );
}

// bool mia::impact(const Polygon2 & p1, Polygon2 & p2, Float2 & point)
// {
//   std::list<Polygon2> output;
//   boost::geometry::intersection(p1, p2, output);
//   compute mean point of output;
//   return output.empty();
// }
