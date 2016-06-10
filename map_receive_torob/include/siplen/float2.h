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

#ifndef MIA_FLOAT2_H
#define MIA_FLOAT2_H

//#include "Math/Math.h"
//#include "Structure/Data.h"
#include "int2.h"
#include "tools.h"

#include <iostream>
#include <array>
#include <list>
#include <assert.h>
namespace mia
{

/// 2 dimention floating number.
class Float2
{
public :

    // Attributes :
    //-------------
    float x, y; /// y coordinate.

    // constructor :
    //--------------
    inline Float2():x(0.0f),y(0.0f) {}
    inline Float2(const float & f):x(f),y(f) {}
    inline Float2(const float &xx, const float &yy):x(xx),y(yy) {}
    inline Float2(const Float2 &v):x(v.x),y(v.y) {}
    inline Float2(const Float2 &v1, const Float2 &v2):x(v2.x - v1.x),y(v2.y - v1.y) {}
//    inline Float2(const Data & d):x(d.value[0]),y(d.value[1]) {}

    // accessor :
    //-----------
    inline float &operator[](int i){ return *(&x+i); } 
    static unsigned size(){ return 2; }

    // Assignment and cast :
    //---------------------
    inline Float2 &operator=(const Float2 &v)
    {
        x= v.x;
        y= v.y;
        return (*this);
    }
    inline Float2 &operator=(const float &f)
    {
        x= f;
        y= f;
        return *this;
    }

    // Convertion
    inline operator Int2() {
        return Int2( (int)x, (int)y );
    }

    // Equality :
    //-------------
    inline bool operator==(const Float2 &v) const
    {
        return ( x == v.x && y == v.y );
    }
    inline bool operator!=(const Float2 &v) const
    {
        return ( x != v.x || y != v.y );
    }

    // Scalar Addition :
    //------------------
    inline Float2 operator+(const float f)const
    {
        return Float2( x+f, y+f );
    }
    inline Float2 &operator+=(const float f)
    {
        x+= f;
        y+= f;
        return *this;
    }
    inline Float2 operator-(const float f)const
    {
        return Float2( x-f, y-f );
    }

    inline Float2 &operator-=(const float f) {
        x-= f;
        y-= f;
        return *this;
    }

    // Addition :
    //-----------
    inline Float2 operator+(const Float2 &v) const
    {
        return Float2( x+v.x, y+v.y );
    }
    inline Float2 &operator+=(const Float2 &v)
    {
        x+= v.x;
        y+= v.y;
        return *this;
    }
    inline Float2 operator-(const Float2 &v) const
    {
        return Float2( x-v.x, y-v.y );
    }
    inline Float2 &operator-=(const Float2 &v)
    {
        x-= v.x;
        y-= v.y;
        return *this;
    }
    inline Float2 operator-()const {
        return Float2(-x, -y);
    }

    // Scalar Multiplication :
    //------------------------
    inline Float2 operator*(const float a) const
    {
        return Float2( x*a, y*a );
    }
    inline Float2 &operator*=(const float a)
    {
        x*= a;
        y*= a;
        return *this;
    }

    // Scalar Division :
    //-------------
    inline Float2 operator/(const float a)const
    {
        assert(a!=0);
        float ia = 1.0f/a;
        return Float2( x*ia, y*ia );
    }
    inline Float2 &operator/=(const float a)
    {
        assert(a!=0);
        float ia = 1.0f/a;
        x*= ia;
        y*= ia;
        return *this;
    }

    // Colinear vector :
    //------------------
    Float2 getColinearFromX( float xx )const {
        return Float2( xx, y*(xx/x) );
    }
    Float2 getColinearFromY( float yy )const {
        return Float2( x*(yy/y), yy );
    }

    // reverse :
    //-------------
    /*
            inline Float2 &reverse()
            {
                            x=1/x; y=1/y;
                            return *this;
            }
    */

    // Dot / cross Product :
    //---------------------------------------
    inline float operator*(const Float2 &v)const
    {
        return (x*v.x + y*v.y );
    }

    // Cross-Product :
    //----------------
    inline float crossProduct(const Float2 &w)const
    {
        return (x * w.y - y * w.x);
    }

    // Cross-Product :
    //----------------

    // Normalization :
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

    inline Float2 normalVector( float unit= 1.0f )const
    {
        Float2 v(0.0f);
        float l= length();
        if(l != 0.f)
        {
            float n = unit/l;
            v.x= x*n;
            v.y= y*n;
        }
        return v;
    }

    inline float bound(float told)
    {
        float l = length();
        if(l > told)
        {
            float n = told/l;
            x*= n;
            y*= n;
        }
        return l;
    }

    inline float length2()const {
        return (x*x + y*y);
    }
    inline float length()const {
        return sqrtf( length2() );
    }

    inline Float2 orthogonal()const
    {
        return Float2(-y, x);
    };
    inline Float2 orthonormal(float unit= 1.f)const
    {
        Float2 v= Float2(-y, x);
        v.normalize(unit);
        return v;
    }
    inline float orthonormalize()
    {
        float xx= x;
        x= -y;
        y= xx;

        return normalize();
    }
    inline float orthonormalize( float unit )
    {
        float xx= x;
        x= -y;
        y= xx;
        return normalize( unit );
    }

    // Order theory :
    //---------------
    // sur les angles dans le sens trigonométrique

    inline bool operator<(const Float2 & p) const
    {
        float cp= crossProduct(p);
        if( y < 0.f )
        {
            return ( p.y >= 0.f || cp > 0.f );
        }
        return ( p.y > 0.f && cp > 0.f );
    }

    inline bool operator>(const Float2 & p) const
    {
        float cp= crossProduct(p);
        if( y <= 0.f )
        {
            return ( p.y < 0.f && cp < 0.f );
        }
        return ( p.y <= 0.f || cp < 0.f );
    }

    // Angle :
    //-------------
    inline float angle()const
    {
        if ( y < 0.0f )
            return ( -acosf( normalVector().x ) );
        return acosf( normalVector().x );
    }
    
    inline float angleOfNorm()const
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
    static inline Float2 direction( float delta )
    {
        /// \param delta in radian
        return Float2( cos( delta ), sin( delta ) );
    }

    //inline bool isColinear(const Float2 &v)const{ return ( crossProduct(v) == 0.0f ); }
    // to do suprime that 4 functions:

//      inline bool isTrigo(const Float2 &v1, const Float2 &v2){ return ( crossProduct(v1, v2) > 0.0f ); }
//      inline bool isnTrigo(const Float2 &v1, const Float2 &v2){ return ( crossProduct(v1, v2) < 0.0f ); }
//      inline bool isRight(const Float2 &v)const{ return ( crossProduct(v) < 0.0f ); }
//      inline bool isLeft(const Float2 &v)const{ return ( crossProduct(v) > 0.0f ); }
//      bool isBetweenRight(Float2 v1, Float2 v2)const;
//      inline bool isBetweenLeft(Float2 v1, Float2 v2)const{ return !isBetweenRight(v1, v2); }
    /*
            inline bool isBetween(Float2 v1, Float2 v2)const;
            {
    / *
                    bool posV1= crossProduct(v1) < 0.0f;
                    bool posV2= crossProduct(v2) < 0.0f;
                    bool v2posV1= v2.crossProduct(v1) < 0.0f;
                    return ( posV1 && ( !posV2 || !v2posV1 ) )
                                    || !( posV1 || v2posV1 || posV2 );
    * /
                    return crossProduct(v1) < 0.0f && crossProduct(v2) > 0.0f;
            }
    */
    // Basis change :
    //-------------
    inline Float2 fromBasis( float angle, const Float2 & refPoint= 0.0f )const
    {
        Float2 X( cos( angle ), sin( angle ) );
        Float2 Y( -X.y, X.x );

        return (X*x) + (Y*y) + refPoint;
    }

    inline void moveFromBasis( float angle, const Float2 & refPoint= 0.0f )
    {
        (*this)= fromBasis(angle, refPoint);
    }

    inline Float2 toBasis( float angle, const Float2 & refPoint= 0.0f )const
    {
        Float2 X( cos( -angle ), sin( -angle ) );
        Float2 Y( -X.y, X.x );
        return X * (x - refPoint.x) + Y * (y - refPoint.y);
    }

    inline Float2 moveToBasis( float angle, const Float2 & refPoint= 0.0f )
    {
        (*this)= toBasis(angle, refPoint);
    }
    
    // tools :
    //--------
    // String toString()const;
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

// Distance betwen 2 points : :
//-------------
inline float distance(Float2 point1, const Float2 &point2)
{
    point1-= point2;
    return point1.length();
}

inline float distance2(Float2 point1, const Float2 &point2)
{
    point1-= point2;
    return point1.length2();
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

// reverse :
//-------------
//inline Float2 reverse(const Float2 &v){ return Float2( 1/v.x, 1/v.y); }

// Angle :
//-------------
inline Float2 normalDirection(float theta)
{
    return Float2( cos(theta), sin(theta) );
}

inline float angle(const Float2 & v1, const Float2 & v2)
{
    return reduceRadian( v2.angle() - v1.angle() );
}

inline float angleOfNorm(const Float2 & v1, const Float2 & v2)
{
    return reduceRadian( v2.angleOfNorm() - v1.angleOfNorm() );
}

inline float angle(const Float2 & p1, const Float2 & p2, const Float2 & p3 )
{
    return reduceRadian( Float2(p2, p1).angle() - Float2(p2, p3).angle() );
}

// Trigo test :
//-------------
inline bool isTrigo(const Float2 &v1, const Float2 &v2) {
    return ( crossProduct(v1, v2) > 0.0f );
}

inline bool isnTrigo(const Float2 &v1, const Float2 &v2) {
    return ( crossProduct(v1, v2) < 0.0f );
}

inline bool isTrigo(const Float2 &v1, const Float2 &v2, const Float2 &v3)
{
    bool t12(isTrigo(v1, v2) ), t23(isTrigo(v2, v3) ), t13(isTrigo(v1, v3) );
    return (t12 && t23) || (!t13 && ( t12 || t23 ));
}
inline bool isnTrigo(const Float2 &v1, const Float2 &v2, const Float2 &v3)
{
    bool t12(isnTrigo(v1, v2) ), t23(isnTrigo(v2, v3) ), t13(isnTrigo(v1, v3) );
    return (t12 && t23) || (!t13 && ( t12 || t23 ));
}

// refered order relation (by trigo) :
//------------------------------------
/*
 inline bool isGreater(const Float2 &v1, const Float2 &v2 const Float2 &ref)
 {
  return ( crossProduct(v1, v2) > 0.0f );
 }
*/

// tools :
//--------
//std::array<Float2, 2> simpleLinearRegression( const std::list<Float2> & lf2);
//std::array<Float2, 2> simpleLinearRegression( const std::list<Float2> & lf2);


// stream :
//-------------
inline std::ostream & operator<< ( std::ostream & os, const Float2 & v )
{
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}

class Transform{
public:
    Float2 translation;
    Float2 center;
    float rotation;
    
    static Transform from_match ( std::list<std::pair<Float2, Float2>>::const_iterator it,
                                     std::list<std::pair<Float2, Float2>>::const_iterator itEnd );
    
    inline Float2 operator()( const Float2 & f2 )const
    {
        return center + ( (f2+translation)-center ).rotate( rotation );
    }
};


/*
bool isTrigoRotation(Float2 v1, Float2 v2)
{
        v1.normalize();
        v2.normalize();

        if( )
}
*/

};

#endif // FLOAT2_H
