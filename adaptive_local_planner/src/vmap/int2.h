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

#include "tools.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#ifndef INT2_H
#define INT2_H


namespace mia{

struct Int2 {
    int x, y;

    // constructor :
    //--------------
    inline Int2():x(0.0f),y(0.0f) {}
    inline Int2(const int &xx, const int &yy):x(xx),y(yy) {}
    inline Int2(const Int2 &v):x(v.x),y(v.y) {}
    inline Int2(const Int2 &v1, const Int2 &v2):x(v2.x - v1.x),y(v2.y - v1.y) {}
//    inline Int2(const Data & d):x(d.flag[0]),y(d.flag[1]) {}

  // transform operator :
  //---------------------
  inline const Int2 & operator+=(const Int2 &v2){
    x+= v2.x;
    y+= v2.y;
    return *this;
  }

  inline const Int2 & operator-=(const Int2 &v2){
    x-= v2.x;
    y-= v2.y;
    return *this;
  }
  
  inline const Int2 & operator*=(int f){
    x*= f;
    y*= f;
    return *this;
  }
  
  inline const Int2 & operator/=(int f){
    x/= f;
    y/= f;
    return *this;
  }
   
  // aritmethic operator :
  //----------------------
  inline Int2 operator+(const Int2 &v2){ return Int2(x+v2.x, y+v2.y); }
  inline Int2 operator-(const Int2 &v2){ return Int2(x-v2.x, y-v2.y); }
  inline Int2 operator*(int f){ return Int2(x*f, y*f); }
  inline Int2 operator/(int f){ return Int2(x/f, y/f); }

  // Equality :
  //-----------
  inline bool operator==(const Int2 &i){ return (x==i.x && y==i.y); }
  inline bool operator!=(const Int2 &i){ return (x!=i.x || y!=i.y); }
};
  // stream :
  //---------
  inline std::ostream & operator<< ( std::ostream & os, const Int2 & v )
  {
      os << "[" << v.x << " " << v.y << "]";
      return os;
  }
};

#endif // INT2_H
