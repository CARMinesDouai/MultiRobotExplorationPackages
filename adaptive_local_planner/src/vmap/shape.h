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

#ifndef MIA_SHAPE_H
#define MIA_SHAPE_H

#include <boost/geometry/geometries/register/segment.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include "float2.h"

namespace mia{

class Segment2{
public:
  Float2 a, b;

  // constructor :
  //--------------
  Segment2():a(Float2()), b(Float2()) {}
  Segment2(const Float2 &aa, const Float2 &bb):a(aa),b(bb) {}
  Segment2(const Segment2 &s):a(s.a),b(s.b) {}
};

class Circle2{
public:
  Float2 center;
  float radius;

  // constructor :
  //--------------
  Circle2():center(Float2()), radius(0.f) {}
  Circle2(const Float2 &cc, float rr):center(cc),radius(rr) {}
  Circle2(const Circle2 &c):center(c.center),radius(c.radius) {}
};

typedef std::vector<Float2> Polygon2;
typedef boost::geometry::model::polygon<Float2> BoostPolygon2;

};

BOOST_GEOMETRY_REGISTER_SEGMENT( mia::Segment2, mia::Float2, a, b )
BOOST_GEOMETRY_REGISTER_RING( mia::Polygon2 )

#endif // SHAPE_H
