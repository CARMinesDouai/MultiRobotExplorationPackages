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

#ifndef MIA_INT2_H
#define MIA_INT2_H

#include <math.h>
#include <iostream>

namespace mia {

class Float2;

class Int2
{
public:
    // Attributes
    int x, y;

    // Constructor
    inline Int2():x(0),y(0) {}
    inline Int2(const int &i):x(i),y(i) {}
    inline Int2(const int &xx, const int &yy):x(xx),y(yy) {}
    inline Int2(const Int2 &op):
        x( op.x ),
        y( op.y )
    {}
    Int2(const Float2 &v);

    // accessor :
    //-----------
    inline int &operator[](int i) {
        return *(&x+i);
    }
    static unsigned size() {
        return 2;
    }

    // Assignment
    inline Int2 &operator=(const Int2 &op)
    {
        x= op.x;
        y= op.y;
        return (*this);
    }

    const Int2 &operator=(const Float2 & v);
    inline Int2 &operator=(const int &i)
    {
        x= i;
        y= i;
        return (*this);
    }

    // Convertion
    operator Float2();

    // Equality & Not
    inline bool operator==(const Int2 &op) const {
        return ( x == op.x && y == op.y );
    }
    inline bool operator!=(const Int2 &op) const {
        return ( x != op.x || y != op.y );
    }

    // Addition/Substraction
    inline Int2 operator +(const Int2 &op) const {
        return Int2( x + op.x, y+op.y );
    }
    inline Int2 operator-(const Int2 &op) const {
        return Int2( x-op.x, y-op.y );
    }
    inline Int2 &operator+=(const Int2 &op)
    {
        x+= op.x;
        y+= op.y;
        return *this;
    }
    inline Int2 &operator-=(const Int2 &op)
    {
        x-= op.x;
        y-= op.y;
        return *this;
    }

    // Scalar mult :
    inline Int2 &operator*=(int n)
    {
        x*= n;
        y*= n;
        return *this;
    }
    inline Int2 &operator/=(int n)
    {
        x/= n;
        y/= n;
        return *this;
    }

    inline Int2 &operator*=(float f)
    {
        x= (int)((float)x*f);
        y= (int)((float)y*f);
        return *this;
    }
    inline Int2 &operator/=(float f)
    {
        float nf= 1.0f/f;
        x= (int)((float)x*nf);
        y= (int)((float)y*nf);
        return *this;
    }

    // Normalization :
    //----------------
    inline float norm2()const {
        return (float)(x*x + y*y);
    }
    inline float length()const {
        return sqrtf( norm2() );
    }

    // To string :
    //------------
    //String toString()const;
};

inline std::ostream & operator<< ( std::ostream & os, const Int2 & v )
{
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}

// Distance betwen 2 points :
//---------------------------
inline float distanceBetween(Int2 op1, const Int2 &op2)
{
    op1-= op2;
    return op1.length();
}

inline float distance2Between(Int2 op1, const Int2 &op2)
{
    op1-= op2;
    return op1.norm2();
}

};

#endif // INT2_H
