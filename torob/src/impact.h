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

#ifndef IMPACT_H
#define IMPACT_H

#include "float2.h"
#include "shape.h"

namespace mia{
  
  bool impact(const Segment2 & s1, const Segment2 & s2);
  bool impact(const Segment2 & s1, const Segment2 & s2, Float2 & point);
  
  bool impact(const Circle2 & c1, const Circle2 & c2);
  bool impact(const Circle2 & c1, const Circle2 & c2, Float2 & point);

  bool impact(const Float2 & v, const Circle2 & c);
  inline bool impact(const Circle2 & c, const Float2 & v){ return impact(v, c); }
  
  bool impact(const Segment2 & s, const Circle2 & c);
  bool impact(const Segment2 & s, const Circle2 & c, Float2 & point);
  inline bool impact(const Circle2 & c, const Segment2 & s ){ return impact(s, c); }
  inline bool impact(const Circle2 & c, const Segment2 & s, Float2 & point){ return impact(s, c, point); }

  bool impact(const Polygon2 & p1, const Polygon2 & p2);
  bool impact(const Polygon2 & p1, const Polygon2 & p2, std::list<Polygon2> & output);
//  bool impact(const Polygon2 & p1, Polygon2 & p2, Float2 & point);

};

#endif // IMPACT_H
