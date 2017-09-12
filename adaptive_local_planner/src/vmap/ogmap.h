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

#ifndef OGMAP_H
#define OGMAP_H

#include "int2.h"
#include "float2.h"
#include "data.h"

namespace mia {
class OGMap
{
    int ** a_grid;
    int a_width, a_height;
    Float2 a_origin;
    float a_resolution;
    
public:
    OGMap(int width= 1, int height= 1, const Float2 & origine= Float2(), float resolution= 1.f);
    virtual ~OGMap();
    
    // Getter and setter :
    inline int width()const{ return a_width; }
    inline int height()const{ return a_height; }
    inline Float2 origin()const{ return a_origin; }
    inline float resolution()const{ return a_resolution; }
    
    inline int * operator[](int i){ return a_grid[i]; }
    inline const int * operator[](int i)const{ return a_grid[i]; }
    
    // Test:
    bool is_cell(int i, int j)const;
    bool is_neighbor_type(int i, int j, int type )const;
    
    // Transform :
    virtual Float2 gridToWorld(const Float2 vg)const
    {
        return (vg + Float2(0.5f, 0.5f)) * a_resolution + a_origin;
    }
    virtual Float2 worldToGrid(const Float2 vw)const
    {
        return (vw - a_origin) / a_resolution - Float2(0.5f, 0.5f);
    }
    
    // Construction :
    virtual void initialize(const Data &map);
    virtual void reduce(int scale);
    
    // Getter :
    
    // Clearing :
    virtual void clear(int i= 42);
    virtual void removeRegionLowerThan(int region, int size);
    virtual std::list<Int2> removeRegion(int i, int j);
    virtual void removeRegionunconnectedTo(int region);
    
    // Filtering :
    virtual void reduce_filter(int value1, int value2, int told);
    virtual void reduce_cell_filter(int i, int j, int value1, int value2, int told, std::list<Int2> & backtrac );
    virtual int frontier(int cv1, int cv2, int cv);
    
    // Attributes :
    virtual Float2 center(int cellVal)const;
    
    // Cell attribute:
    virtual std::list<Int2> neighbor(int i, int j)const;
    
    // File interaction:
    virtual void load( const std::string & file, Float2 origin, float resolution );
    virtual void save( const std::string & file )const;
};
};

#endif // OGMAP_H
