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

#ifndef MIA_MATH_H
#define MIA_MATH_H

#include <math.h>
#include <stdlib.h>

/**
 * Some basic mathematics function
 */

namespace mia
{
const float _PIov2= 1.570796327f;
const float _PIov4= 0.785398163f;
const float _PI= 3.14159265f;
const float _2PI= 6.28318531f;

const float _invPI= 0.318309886f;
const float _2ovPI= 0.636619772f;

const float _DEG2RAD= 0.017453293f;
const float _RAD2DEG= 57.295779579f;

/// trigo :
///--------
inline float reduceRadian(float rad)
{
    float recup= rad / _2PI;
    recup-= (int)(recup);

    if( recup < 0.0f )
        recup= 1.0f + recup;

    if( recup > 0.5f )
        return (recup - 1.0f) * _2PI;

    return recup * _2PI;
}

/// Absolute Value
///---------------
inline float absoluteInt( int i ) {
    return abs( i );
}
inline float absoluteFloat( float f ) {
    return fabsf( f );
}

/// Random :
///---------
// main :
inline void randomSeed( int seed=0 ) {
    srandom(seed + 1);
};
inline int randomInt() {
    return (int)random();
};

// interger :
inline int randomInt(int modulo) {
    return randomInt() % modulo;
}
inline int randomInt(int start, int end) {
    return start + ( randomInt() % (end - start) );
}

// Float :
inline float randomFloat() {
    return float(randomInt() % 8388608) / 8388608.0f;
}
inline float randomFloat(float start, float end) {
    return start + ( randomFloat() * (end - start) );
}
inline float randomFloatAround(float center, float radius) {
    return randomFloat(center-radius, center+radius);
}
inline float randomFloatIn(float f1, float f2) {
    return (f1 < f2)?randomFloat(f1, f2):randomFloat(f2, f1);
}
inline float randomAngle() {
    return _PI - (randomFloat() * _2PI);
}

template<class Obj>
class valued{
public :
    Obj item;
    float value;

    valued<Obj>():item(), value(0){};
    valued<Obj>(const Obj & o, float v):item(o), value(v){};

    bool operator<( const valued<Obj> &vo )const{ return (value < vo.value); }
    bool operator<=( const valued<Obj> &vo )const{ return (value <= vo.value); }

    bool operator>( const valued<Obj> &vo )const{ return (value > vo.value); }
    bool operator>=( const valued<Obj> &vo )const{ return (value >= vo.value); }
};

};

#endif // MIA_MATH_H
