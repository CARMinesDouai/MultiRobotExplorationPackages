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

#ifndef MIA_SHAPE_H
#define MIA_SHAPE_H

#include "float2.h"

namespace mia
{

class Shape
{
public :

    enum EType
    {
        e_Type_None= 0,
        e_Type_Line,
        e_Type_Circle,
//              e_Type_Polygone,
//              e_Type_Convex,

        e_NbType
    };

    Float2 a_position;

private:
    EType a_type;

protected :

    // Contructor / Destructor :
    //--------------------------
    Shape(EType type= e_Type_None):a_type(type) {};
    Shape(EType type, Float2 position):a_type(type) {
        a_position= position;
    };

    virtual ~Shape() {};

    // Geter / seter :
    //----------------
    virtual int getNbType() {
        return (int)e_NbType;
    }
    virtual EType getType() {
        return a_type;
    }
    virtual const Float2 & position()const {
        return a_position;
    }
    
    // Assignement :
    //--------------
    virtual const Shape &operator=(const Shape &s);

    // Type :
    //-------
    virtual bool is(EType type) {
        return (a_type == type);
    }

    // Size operation :
    //-----------------
    virtual void increaseSize(float) {}

    // Move Operator :
    //----------------
    inline void operator+= ( const Float2 &v ) {
        a_position+= v;
    }
    inline void operator-= ( const Float2 &v ) {
        a_position-= v;
    }
};

/// Shape : Line :
///---------------
class Line : public Shape
{
public :
    Float2 a_vector;

    /// Contructor / Destructor :
    ///--------------------------
    Line():Shape(e_Type_Line) {};
    Line(const Float2 &A, const Float2 &B):Shape(e_Type_Line) {
        a_position= A ;
        a_vector= B - A;
    };
    virtual ~Line() {};

    /// Getter && setter :
    ///-------------------
    virtual Float2 position2()const {
        return (a_position + a_vector);
    }
    virtual float getSlope()const;
    virtual float getYOrigin()const {
        return a_position.y - (getSlope() * a_position.x);
    }

    /// Assignement :
    ///--------------
    virtual const Line & operator=(const Line &s);

    /// Size operator :
    ///----------------
    //virtual void increaseSize(float){}
};

class Circle : public Shape
{
public :
    float a_radius;

    /// Contructor / Destructor :
    ///--------------------------
    Circle():Shape(e_Type_Circle) {};
    Circle(Float2 position, float radius):Shape(e_Type_Circle, position) {
        a_radius= radius;
    };
    virtual ~Circle() {};

    /// Assignement :
    ///--------------
    virtual const Circle & operator=(const Circle &c);

    /// Size operator :
    ///----------------
    virtual void increaseSize(float f) {
        a_radius+= f;
    }
};


// Point x Circle :
bool collision(const Float2 &p1, const Float2 &p2 );

// Point x Circle :
bool collision(const Float2 & , const Circle & );

// Point x line :
bool collision(const Float2 & , const Line & );

// Point x Convex
//      bool collision(const Float2 & , const Convex & );

// Line x line :
bool collision(Float2 & , const Line & , const Line &);
bool collision(const Line & , const Line & );
bool collisionLineLine(Float2 & res, const Float2 & A1, const Float2 & A2, const Float2 & B1, const Float2 & B2);

// Circle X Circle
bool collision(Float2 & between, float & distMin, const Circle & , const Circle & );
bool collision(Float2 & between, const Circle & , const Circle & );
bool collision(const Circle & c1, const Circle & c2);
bool collisionCircleCircle(const Float2 & c1, float r1, const Float2 & c2, float r2);

// Circle x Line
bool collision(Float2 & position, const Circle & , const Line &);
//      bool collision(Line &, const Circle & , const Line &, bool verbeux= false );
bool collision(const Circle & , const Line &);
bool collisionCircleLine(Float2 & res, const Float2 & cc, float cr, const Float2 & l1, const Float2 & l2);

// Circle x Convex
//      bool collision(const Circle &, const Convex &);

/** Hiding: **/

// Line x line :
bool isHiding(Line &, const Float2 & origin, const Line & ask, const Line & wall );

};

#endif // MIA_SHAPE_H
