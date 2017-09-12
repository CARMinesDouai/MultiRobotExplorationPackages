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

// Boost :
#include "tools.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#ifndef MIA_FLOAT2_H
#define MIA_FLOAT2_H

namespace mia{

class Float2 {
public:
  float x, y;

  // constructor :
  //--------------
  inline Float2():x(0.0f),y(0.0f) {}
  inline Float2(const float &xx, const float &yy):x(xx),y(yy) {}
  inline Float2(const Float2 &v):x(v.x),y(v.y) {}
  inline Float2(const Float2 &v1, const Float2 &v2):x(v2.x - v1.x),y(v2.y - v1.y) {}
//    inline Float2(const Data & d):x(d.value[0]),y(d.value[1]) {}


  // sum operator :
  //---------------

  inline const Float2 & operator+=(const Float2 &v2){
    x+= v2.x;
    y+= v2.y;
    return *this;
  }

  inline const Float2 & operator-=(const Float2 &v2){
    x-= v2.x;
    y-= v2.y;
    return *this;
  }
  
  inline Float2 operator+(const Float2 & f)const{ return Float2(x+f.x, y+f.y); }
  inline Float2 operator-(const Float2 & f)const{ return Float2(x-f.x, y-f.y); }

  // scalar multiplication :
  //------------------------
  
  inline const Float2 & operator*=(float f){
    x*= f;
    y*= f;
    return *this;
  }
  
  inline const Float2 & operator/=(float f){
    f= 1.0f/f;
    x*= f;
    y*= f;
    return *this;
  }

  inline Float2 operator*(float f)const{ return Float2(x*f, y*f); }
  inline Float2 operator/(float f)const{
    f= 1.0f/f;
    return Float2(x*f, y*f); 
  }

  // length :
  //---------
  inline float length2()const {
    return (x*x + y*y);
  }
  inline float length()const {
    return sqrtf( length2() );
  }

  // length :
  //---------
  inline float distance2(const Float2 & to)const {
    return (to - *this).length2();
  }
  inline float distance(const Float2 & to)const {
    return sqrtf( distance2(to) );
  }

  // normalization :
  //----------------
  inline float normalize( float unit= 1.0f )
  {
      float l = length();
      if(l != 0)
      {
	  float n = unit/l;
	  x*= n;
	  y*= n;
      }
      return l;
  }

  inline Float2 normal( float unit= 1.0f )const
  {
      float l= length();
      if(l != 0.f)
      {
	  float n = unit/l;
	  return Float2(x*n, y*n);
      }
      return Float2(0.0f, 0.0f);
  }

  inline float orthonormalize( float unit )
  {
      float xx= x;
      x= -y;
      y= xx;
      return normalize( unit );
  }
  
  inline Float2 orthogonal()const{ return Float2(-y, x); }
  inline Float2 orthonormal(float unit= 1.f)const{ return orthogonal().normal( unit ); }
  
  // Angle :
  //-------------
  inline float angle()const
  {
    if ( y < 0.0f )
	return ( -acosf( normal().x ) );
    return acosf( normal().x );
  }

  inline float angleOfNormalized()const
  {
    if ( y < 0.0f )
	return ( -acosf(x) );
    return acosf(x);
  }

  // Rotation
  inline const Float2 & rotate(float delta)
  {
    Float2 X( cos( delta ), sin( delta ) );
    Float2 Y( -X.y, X.x );
    (*this)= X * x + Y * y;

    return *this;
  }
  
  /// return the vector in the direction delta.
  static Float2 direction( float delta ){ 
    /// \param delta in radian
    return Float2(cos( delta ), sin( delta ));
  }
  
  inline void setDirection( float delta )
  {
    /// \param delta in radian
    x= cos( delta );
    y= sin( delta );
  }
  
  inline void setAngle( float delta )
  {
    /// \param delta in radian
    float l= length();
    x= cos( delta )*l;
    y= sin( delta )*l;
  }
  
  // tools :
  //--------
  // String toString()const;
  static Float2 middle( const Float2 & A, const Float2 & B );
  static Float2 mean( const std::list<Float2> & lFloat2 );
  static std::array<Float2, 2> simpleLinearRegression( const std::list<Float2> & lFloat2 );
  static std::array<Float2, 2> projectionSegment( const std::list<Float2> & lFloat2, const Float2 &mean, const Float2 &normDir );
  inline static std::array<Float2, 2> projectionSegment( const std::list<Float2> & lFloat2, const std::array<Float2, 2> & normSegment)
  {
    return projectionSegment( lFloat2, normSegment[0], normSegment[1] );
  }
  inline static bool possibleSegmentRegression( const std::list<Float2> & lFloat2, float threshold )
  {
    return validSegmentRegression( lFloat2, simpleLinearRegression( lFloat2 ), threshold );
  }
  static bool validSegmentRegression( const std::list<Float2> & lFloat2, const std::array<Float2, 2> & normSegment, float treshold );

  static std::list<Float2> polarSort(const std::list<Float2> & lFloat2);
};

  // stream :
  //---------
  inline std::ostream & operator<< ( std::ostream & os, const Float2 & v )
  {
      os << "[" << v.x << " " << v.y << "]";
      return os;
  }
  
  // angle calculation :
  //--------------------
  inline float angle(const Float2 & v1, const Float2 & v2)
  {
      return reduceRadian( v2.angle() - v1.angle() );
  }

  inline float angleOfNormalized(const Float2 & v1, const Float2 & v2)
  {
      return reduceRadian( v2.angleOfNormalized() - v1.angleOfNormalized() );
  }

  inline float angle(const Float2 & p1, const Float2 & p2, const Float2 & p3 )
  {
      return reduceRadian( Float2(p2, p1).angle() - Float2(p2, p3).angle() );
  }
  
  // Dot-Product (projection):
  //-------------
  inline float dotProduct(const Float2 &v, const Float2 &w)
  {
    return v.x*w.x + v.y*w.y;
  }

  // Cross product (in Z) :
  //-------------
  inline float crossProduct(const Float2 &v,const Float2 &w)
  {
    return (v.x * w.y - v.y * w.x);
  }

  inline float crossProductNormalized(Float2 &v, Float2 &w)
  {
    v.normalize();
    w.normalize();
    return (v.x * w.y - v.y * w.x);
  }
  
  // Transform obj :
  //----------------
  class Transform{
  public:
    Float2 translation;
    Float2 center;
    float rotation;
    
    Transform( const Float2 & trans= Float2(), const Float2 & rotCenter= Float2(), float rot= 0.f ):
        translation(trans),
        center(rotCenter),
        rotation(rot)
    {
    }

    static Transform from_match ( std::list<std::pair<Float2, Float2>>::const_iterator it,
                                     std::list<std::pair<Float2, Float2>>::const_iterator itEnd );
    
    inline Float2 operator()( const Float2 & f2 )const
    {
        return center + ( (f2+translation)-center ).rotate( rotation );
    }
  };
};

BOOST_GEOMETRY_REGISTER_POINT_2D( mia::Float2, float, cs::cartesian, x, y )

#endif // FLOAT2_H
