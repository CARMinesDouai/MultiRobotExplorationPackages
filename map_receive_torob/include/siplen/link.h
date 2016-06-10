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

#ifndef MIA_LINK_H
#define MIA_LINK_H

#include "particle.h"
#include "data.h"

namespace mia{
    
class Link
{
public:
    // Definition
    float angle;
    float length;

    // Attribute :
    int flag;

    Link(float a= 0.f, float l= 0.f, int f= 0):angle(a),length(l),flag(f){}
    Link(const Link& other):Link(other.angle, other.length, other.flag){}
    ~Link(){}

    Link& operator=(const Link& other);
};

};

#endif // MIA_LINK_H
