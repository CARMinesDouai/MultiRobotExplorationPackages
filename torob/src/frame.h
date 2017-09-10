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

#ifndef FRAME_H
#define FRAME_H

#include "int2.h"
#include "float2.h"
#include "visimap.h"
#include "ogmap.h"
#include "color.h"

// #include "moinag.h"

#include <iostream>
#include <stdlib.h>

#include <SDL2/SDL.h>

namespace mia{
    
class Frame
{
protected:
    // Window parameters :
    Int2 a_halfSize;

    // 2 Dimention Camera parameters :
    Float2 a_position;
    float a_scale;

    // SDL Interface :
    SDL_Window * a_window;
    SDL_Renderer * a_renderer;
    bool a_sdlOk;

    // Drawing :
    Color a_bgColor, a_penColor;

    // InnerTools :
    inline int X(float x)const {
        return (int)((x-a_position.x)*a_scale) + a_halfSize.x;
    }

    inline int Y(float y)const {
        return -(int)((y-a_position.y)*a_scale) + a_halfSize.y;
    }

public:
    // Constructor, Destructor :
    Frame(const char * name, int size_x =640, int size_y =480, float scale= 1.0 );
    ~Frame();

    // Control :
    void scaleIn(){ a_scale*= 1.2f; }
    void scaleOut(){ a_scale*= 0.8f; }
    void moveTo(const Float2 & p){ a_position= p; }
    void movePixel(const Int2 & pix){ a_position+= Float2(pix.x/a_scale, -pix.y/a_scale); }

    // Windows :
    mia::Int2 windowSize();
    void refresh();

    // Getters and Setters :
    bool isInitialized()const{ return a_sdlOk; }
    Color penColor()const{ return a_penColor; }
    void penColor(const Color & c){ a_penColor= c; }
    Color bgColor()const{ return a_bgColor; }
    void bgColor(const Color & c){ a_bgColor= c; }
 
    inline float scale()const { return a_scale; };
    inline void scale(float newScale) { a_scale= newScale; };

    inline Int2 halfSize()const{ return a_halfSize; }
    inline Float2 position()const{ return a_position; }

    inline Int2 toPixel( const Float2 & point )const { return Int2( X(point.x), Y(point.y) ); }
    inline Float2 toWorld( const Int2 & pixel )const { return Float2( (pixel.x-a_halfSize.x)/a_scale + a_position.x, -(pixel.y-a_halfSize.y)/a_scale + a_position.y); }

    // 1-Dimention Drawing :
    void drawPoint( Float2 p, const Color &color);
    void drawPoints( const std::list<Float2> & lp, const Color &color);
    void drawLine( Float2 p1, Float2 p2, const Color &color);
    void drawLine( Float2 p1, Float2 p2, float thickness, const Color &color);
    
    // 2-Dimention Drawing :
    void drawRect( Float2 p1, Float2 p2, const Color &color);
    
    void drawPolygon(const std::list<Float2> & p, const Color &color);
    void fillPolygon(const std::list<Float2> & p, const Color &color);

    void drawCircle(Float2 center, float radius, const Color &color);
    void fillCircle(Float2 center, float radius, const Color &color);

    // Font :
    void write(const char * str, Float2 p, const Color &color);
    
    // Images :
    void printImage(const char * fileName);

    // References :
    void drawBasis(float unit= 1.);
    
    //MoInAg :
//     void drawParticle( const Particle & bod, const Color &color);
//     void drawParticle( const Particle & bod, const Color &color, const Float2 &delta );
// 
//     void drawAgent( const Moinag * pAgent, const Color & color= 0x808080FF, const Float2 & delta= 0.f );
// 
//     void drawVisibilityGraph(const Visibility & v, const Float2 & delta= 0.f, float localTheta= 0.f, const Float2 & localPosition= 0.f  );
// 
//     void drawGridMap( int ** const grid, int width, int height, Float2 origine, float resolution );
    
    void drawGridMap( const OGMap & map, const mia::Float2 & delta=Float2(0.f, 0.f), bool info= false );
    void drawVisiMap( const VisiMap & map, const mia::Float2 & delta=Float2(0.f, 0.f), bool info= false );
    void drawVisiMapObstacle( const VisiMap & vm, const Float2 & delta=Float2(0.f, 0.f) );
    
    /// Getter / setter :
    ///------------------
    //Float2 getCorner()const{ return a_center - (((Float2)Draw::getScreenSize()) / (a_unitSize + a_unitSize) ); }
};

};
#endif // FRAME_H
