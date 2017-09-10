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


#include "frame.h"
#include "math.h"

#include <SDL2_gfxPrimitives.h>
//#include <SDL2_gfxPrimitives_font.h>

//#include <SDL_image.h>
//#include <SDL_ttf.h>

#include <cstring>

using namespace :: mia;
using namespace :: std;

Frame::Frame(const char * name, int size_x, int size_y, float scale ):
    // Window parameters :
    a_halfSize(size_x/2, size_y/2),
    // 2 Dimention Camera parameters :
    a_position(0., 0.),
    a_scale(scale),
    // SDL Interface :
    a_window(nullptr),
    a_renderer(nullptr),
    a_sdlOk(true),
    // Drawing :
    a_bgColor(0x101010FF),
    a_penColor(0xA0A0A0FF)
{
    //  Initialize SDL 2 :
    if ( SDL_Init(SDL_INIT_VIDEO) != 0 ) {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        a_sdlOk= false;
    }

    // Initialize a window :
    a_window = SDL_CreateWindow( name,
                                SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                a_halfSize.x*2, a_halfSize.y*2,
                                SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE);

    if (a_window == nullptr) {
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        a_sdlOk= false;
    }

    // Initialize a renderer in the window
    a_renderer = SDL_CreateRenderer(a_window, -1, SDL_RENDERER_ACCELERATED);
    if (a_renderer == nullptr) {
        SDL_DestroyWindow(a_window);
        std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_Quit();

    }

//     // Initialize font :
//     if( a_sdlOk && TTF_Init() == -1 )
//     {
//         std::cout << "TTF_Init Error: " << TTF_GetError() << std::endl;
//         a_sdlOk= false;
//     }

//     TTF_Font * font;
//     font= TTF_OpenFont("../resources/font/Roboto-Light.ttf", 20);
//     if( a_sdlOk && !font ) {
//         std::cout << "TTF_OpenFont Error: " << TTF_GetError() << std::endl;
//         a_sdlOk= false;
//     }

    if( a_sdlOk )
    {
        SDL_SetRenderDrawColor(a_renderer, a_bgColor.red, a_bgColor.green, a_bgColor.blue, 255);//a_bgColor.alfa);
        SDL_RenderClear(a_renderer);
        SDL_RenderPresent(a_renderer);
    }
}

Frame::~Frame()
{
    if( a_renderer != nullptr )
        SDL_DestroyRenderer(a_renderer);
    if( a_window != nullptr )
        SDL_DestroyWindow(a_window);
    SDL_Quit();
}

// Control :

mia::Int2 Frame::windowSize() {
    mia::Int2 size;
    SDL_GetWindowSize( a_window, &(size.x), &(size.y) );
    a_halfSize= size;
    a_halfSize/= 2;
    return size;
}

void Frame::refresh() {
    windowSize();

    SDL_RenderPresent(a_renderer);
    SDL_SetRenderDrawColor(a_renderer, a_bgColor.red, a_bgColor.green, a_bgColor.blue, a_bgColor.alfa);
    SDL_RenderClear(a_renderer);
}

// Getters and Setters :

// 1-Dimention Drawing :

// 1-Dimention Drawing :
void Frame::drawPoint( Float2 p, const Color &color) {
    pixelRGBA(a_renderer, X(p.x), Y(p.y), color.red, color.green, color.blue, color.alfa);
}

void Frame::drawPoints( const std::list<Float2> & lp, const Color &color){
  for( std::list<Float2>::const_iterator it(lp.begin()), itEnd(lp.end()) ; it != itEnd ; ++it )
    pixelRGBA(a_renderer, X(it->x), Y(it->y), color.red, color.green, color.blue, color.alfa);
}

void Frame::drawLine( Float2 p1, Float2 p2, const Color &color) {
    lineRGBA(a_renderer, X(p1.x), Y(p1.y), X(p2.x), Y(p2.y), color.red, color.green, color.blue, color.alfa);
}

void Frame::drawLine( Float2 p1, Float2 p2, float thickness, const Color &color)
{
    Float2 thick( p1, p2 );
    thick.orthonormalize( thickness*0.5 );
    Sint16 vx[4], vy[4];
    
    Float2 p(p1-thick);
    vx[0]= X( p.x );
    vy[0]= Y( p.y );
    
    p= p1+thick;
    vx[1]= X( p.x );
    vy[1]= Y( p.y );
    
    p= p2+thick;
    vx[2]= X( p.x );
    vy[2]= Y( p.y );
    
    p= p2-thick;
    vx[3]= X( p.x );
    vy[3]= Y( p.y );
    
    filledPolygonRGBA(a_renderer, vx, vy, 4, color.red, color.green, color.blue, color.alfa);
}

void Frame::drawRect( Float2 pos, Float2 size, const Color &color)
{
    int x1(X(pos.x)),  y1(Y(pos.y)), x2(X(pos.x+size.x)), y2(Y(pos.y+size.y));
    lineRGBA(a_renderer, x1, y1, x2, y1, color.red, color.green, color.blue, color.alfa);
    lineRGBA(a_renderer, x2, y1, x2, y2, color.red, color.green, color.blue, color.alfa);
    lineRGBA(a_renderer, x2, y2, x1, y2, color.red, color.green, color.blue, color.alfa);
    lineRGBA(a_renderer, x1, y2, x1, y1, color.red, color.green, color.blue, color.alfa);
}

void Frame::drawBasis(float unit) {
    Color color(0xA0A0A0FF);
    drawLine( mia::Float2(-0.1, 0.), mia::Float2(unit, 0.), color);
    drawLine( mia::Float2(0.0, 0.), mia::Float2(0., unit), color);
}


// 2-Dimention Drawing :
void Frame::drawPolygon(const std::list<Float2> & p, const Color &color){
    int nbpoint= p.size(), i(0);
    Sint16 vx[nbpoint], vy[nbpoint];
    
    std::list<Float2>::const_iterator itEnd( p.end() );
    for( std::list<Float2>::const_iterator it( p.begin() ) ; it != itEnd; ++ it )
    {
        vx[i]= X(it->x);
        vy[i]= Y(it->y);
        ++i;
    }

    polygonRGBA (a_renderer, vx, vy, nbpoint, color.red, color.green, color.blue, color.alfa);
}

void Frame::fillPolygon(const std::list<Float2> & p, const Color &color){
    int nbpoint= p.size(), i(0);
    Sint16 vx[nbpoint], vy[nbpoint];
    
    std::list<Float2>::const_iterator itEnd( p.end() );
    for( std::list<Float2>::const_iterator it( p.begin() ) ; it != itEnd; ++ it )
    {
        vx[i]= X(it->x);
        vy[i]= Y(it->y);
        ++i;
    }

    filledPolygonRGBA(a_renderer, vx, vy, nbpoint, color.red, color.green, color.blue, color.alfa);
}

void Frame::drawCircle(mia::Float2 center, float radius, const Color &color) {
    circleRGBA(a_renderer, X(center.x), Y(center.y), radius*a_scale, color.red, color.green, color.blue, color.alfa);
}

void Frame::fillCircle(Float2 center, float radius, const Color &color) {
    filledCircleRGBA(a_renderer, X(center.x), Y(center.y), radius*a_scale, color.red, color.green, color.blue, color.alfa);
}

// Font :
void Frame::write(const char * str, Float2 p, const Color &color){
    stringRGBA (a_renderer, X(p.x), Y(p.y), str, color.red, color.green, color.blue, color.alfa);
}

// Images :
void Frame::printImage(const char * fileName) {
    SDL_Surface *bmp = SDL_LoadBMP( fileName );
    if (bmp == nullptr)
        std::cout << "SDL_LoadBMP Error: " << SDL_GetError() << std::endl;

    SDL_Texture *tex = SDL_CreateTextureFromSurface(a_renderer, bmp);

    if (tex == nullptr)
        std::cout << "SDL_CreateTextureFromSurface Error: " << SDL_GetError() << std::endl;
    else
        SDL_RenderCopy(a_renderer, tex, NULL, NULL);
    SDL_FreeSurface(bmp);
    SDL_DestroyTexture(tex);
}


// MoInAg :
// void Frame::drawAgent( const Moinag * pAgent, const Color & color, const Float2 & delta )
// {
//     // Get Body :
//     Particle body( pAgent->body() );
// 
//     {   // Draw trace :
//         std::list<Float2>::const_iterator it( pAgent->trace().begin() ), itEnd( pAgent->trace().end() );
//         Float2 last= *it;
//         fillCircle( last, body.radius*0.5f, color);
// 
//         for( ++it; it != itEnd; ++it ) {
//             drawLine( last, *it, color );
//             last= *it;
//             fillCircle( last, body.radius*0.5f, color);
//         }
// 
//         drawLine( last, body.position, color );
//         drawParticle( body, color );
//     }
//     drawVisibilityGraph( *(pAgent->visibility()) );
// 
//     // Set body in local basis :
//     body.position= 0.f;
//     body.theta= 0.f;
// 
//     // Show filterd obstacles :
// //     {
// //         std::list<mia::Float2>::const_iterator last= pAgent->ptrFilteredObstacleList()->begin();
// //         drawCircle( delta+ *last, pAgent->getEpsilon(), red);
// //
// //         for( std::list<mia::Float2>::const_iterator it= ++(pAgent->ptrFilteredObstacleList()->begin());
// //                 it != pAgent->ptrFilteredObstacleList()->end(); ++it ){
// //             drawCircle( delta+ *it, pAgent->getEpsilon(), red);
// //             drawLine( delta + *last, delta + *it, red);
// //             last= it;
// //         }
// //     }
// 
// 
//     Color flagColor[9]= {
//         Color(0x202020FF), // _undef,
//         Color(0xFF8080FF), // _obstacle,
//         Color(0x904040FF), // _hide_obstacle,
// 
//         Color(0xFFA020FF), // _safe,
//         Color(0x40FF40FF), // _exit,
//         Color(0x404040FF), // _waypoint & _path,
// 
//         Color(0x404040FF),
//         Color(0x404040FF),
//         Color(0x404040FF)
//     };
// 
//     Float2 localPos(pAgent->body().position);
//     float localAngle(pAgent->body().theta);
// 
//     drawVisibilityGraph( *(pAgent->visibility()), delta, localAngle, localPos);
//     
//     // Show obstacles :
//     Color red( 0xFF8080FF );
//     for( std::vector<mia::Particle>::const_iterator it= pAgent->obstacleList()->begin(); it != pAgent->obstacleList()->end(); ++it )
//         drawCircle( delta+it->position, it->radius, red );
// 
//     // drawCircle( pAgent->getTargetVertexPosition(), body.radius, 0x40A0A0FF );
// 
//     // Show local topology :
//     {
//         const BoostTopo * top= pAgent->local();
// 
//         BoostTopo::vertex_iterator vert, verEnd;
//         for( boost::tie(vert, verEnd)= boost::vertices( (*top) ) ; vert != verEnd ; ++vert ) {
//             drawCircle( delta + (*top)[*vert].position, (*top)[*vert].radius, flagColor[(*top)[*vert].flag]);
//         }
// 
//         BoostTopo::edge_iterator ited, end;
//         for( boost::tie(ited, end)= boost::edges( (*top) ) ; ited != end ; ++ited )
//         {
//             Float2 src( (*top)[boost::source(*ited, *top)].position), trg( (*top)[boost::target(*ited, (*top))].position );
//             drawLine(delta+src, delta+trg, flagColor[ (*top)[*ited].flag ]);
//         }
//     }
// 
//     // Show local to visibility :
//     {
// //         cout << "\n\t";
// 
//         std::list< pairOFvertices >::const_iterator it( pAgent->local2visibility().begin() );
//         std::list< pairOFvertices >::const_iterator end( pAgent->local2visibility().end() );
//         for( ; it != end; ++it )
//         {
// //             cout << it->first << ", " << it->second << " | ";
// 
//             Particle src( pAgent->local( it->first ) );
//             Particle trg( pAgent->visibility( it->second ) );
// 
//             drawLine( delta+src.position, delta+trg.position.toBasis(localAngle, localPos), Color(0x808080FF) );
//         }
//     }
// 
// 
// //     cout << endl;
// 
//     fillCircle( delta + pAgent->getControl(), body.radius*0.5f, Color(0x40FF40A0));
// 
//     drawParticle( body, color, delta );
//     drawCircle( delta,  pAgent->getPerceptionRadius(), color );
// }
// 
// void Frame :: drawVisibilityGraph(const Visibility & w, const Float2 & delta, float localTheta, const Float2 & localPosition)
// {
//     float eps( w.getEpsilon() ), halfeps(eps*0.75f);
//     collection<Visibility::Node>::const_iterator vert, verEnd;
//     for( boost::tie(vert, verEnd)= w.a_visibility.vertex_collection().content() ; vert != verEnd ; ++vert ) {
//         Float2 p( delta + (*vert).position.toBasis(localTheta, localPosition) );
//         drawCircle( p, eps, 0x801010FF );
//     }
// 
//     for( int i= w.a_visibility.edge_first_index() ; i < w.a_visibility.edge_limit(); i = w.a_visibility.edge_next_index(i) )
//     {
//         Float2 src( w.a_visibility.source_vertex(i).position );
//         Float2 trg( w.a_visibility.target_vertex(i).position );
//         if( w.a_visibility.edge(i).flag == Visibility::edge_obstacle )
//             drawLine(delta+src.toBasis(localTheta, localPosition), delta+trg.toBasis(localTheta, localPosition), 0x801010FF );
//         else
//             drawLine(delta+src.toBasis(localTheta, localPosition), delta+trg.toBasis(localTheta, localPosition), 0xA0A0A0FF );
//     }
// 
//     for( boost::tie(vert, verEnd)= w.a_visibility.vertex_collection().content() ; vert != verEnd ; ++vert ) {
//         Float2 p( delta + (*vert).position.toBasis(localTheta, localPosition) );
//         write( std::to_string( vert.index() ).c_str(), p+Float2(halfeps,-halfeps), 0xA0A0A0FF);
//     }
// }
// 
// void Frame :: drawParticle( const Particle & bod, const Color &color )
// {
//     Float2 orientation( cos(bod.theta), sin(bod.theta) );
// //    Color in(color);
// //    in.alfa= 128;
// //    fillCircle( bod.position, bod.radius, in );
// 
//     fillCircle( bod.position, bod.radius*0.666f, color );
//     drawCircle( bod.position, bod.radius, color );
//     drawLine( bod.position, bod.position + orientation * bod.radius, color );
// }
// 
// void Frame :: drawParticle( const Particle & bod, const Color &color, const Float2 & delta )
// {
//     Float2 orientation( cos(bod.theta), sin(bod.theta) );
// //    Color in(color);
// //    in.alfa= 128;
// //    fillCircle( delta+bod.position, bod.radius, in );
// 
//     fillCircle( delta+bod.position, bod.radius*0.666f, color );
//     drawCircle( delta+bod.position, bod.radius, color );
//     drawLine( delta+bod.position, delta+bod.position + orientation * bod.radius, color );
// }

void Frame :: drawGridMap( const OGMap & map, const mia::Float2 & delta, bool /*info*/ )
{
    // Unkwon
    {
        std::list<Float2> limit;
        limit.push_back( delta + map.origin() );
        limit.push_back( delta + map.origin() + Float2(map.width() * map.resolution(), 0) );
        limit.push_back( delta + map.origin() + Float2(map.width() * map.resolution(), map.height() * map.resolution() ) );
        limit.push_back( delta + map.origin() + Float2(0, map.height()*map.resolution()) );
        fillPolygon( limit, 0x606060FF );
    }
    
    const int nbc(13);
    Color color[nbc]= {
         Color(0x800000FF), Color(0x008000FF), Color(0x000080FF), Color(0x000000FF),
         Color(0x404000FF), Color(0x004040FF), Color(0x400040FF), Color(0x602000FF), 
         Color(0x600020FF), Color(0x303030FF)/*Color(0x006020FF)*/, Color(0x206000FF), Color(0x200060FF),
         Color(0x002060FF),
    };

    for( int i(0); i < map.width() ; ++i )
        for( int j(0); j < map.height() ; ++j )
    {
        Float2 o= delta + map.origin() + Float2(i*map.resolution(), j*map.resolution() );

        std::list<Float2> limit;
        limit.push_back( o );
        limit.push_back( o + Float2(map.resolution(), 0) );
        limit.push_back( o + Float2(map.resolution(), map.resolution() ) );
        limit.push_back( o + Float2(0, map.resolution() ) );
        
        switch( map[i][j] )
        {
            case -2:
               fillPolygon( limit, 0xC0C0C080 );
               break;
            case -1:
//                fillPolygon( limit, 0x606060FF );
                break;
            case 0:
                fillPolygon( limit, 0xC0C0C0FF );
                break;
            default:
                fillPolygon( limit, color[map[i][j]%nbc] );
        }
    }
}

// void Frame :: drawGridMap( int ** const grid, int width, int height, Float2 origine, float resolution )
// {
//     // Unkwon
//     {
//         std::list<Float2> limit;
//         limit.push_back( origine );
//         limit.push_back( origine + Float2(width*resolution, 0) );
//         limit.push_back( origine + Float2(width*resolution, height*resolution) );
//         limit.push_back( origine + Float2(0, height*resolution) );
//         fillPolygon( limit, 0x606060FF );
//     }
//     
//     const int nbc(14);
//     Color color[nbc]= {
//          Color(0x800000FF), Color(0x000080FF), Color(0x101010FF), Color(0x000000FF),
//          Color(0x404000FF), Color(0x004040FF), Color(0x400040FF), Color(0x602000FF), 
//          Color(0x600020FF), Color(0x006020FF), Color(0x206000FF), Color(0x200060FF),
//          Color(0x002060FF), Color(0x008000FF)
//     };
// 
//     for( int i(0); i < width ; ++i )
//         for( int j(0); j < height ; ++j )
//     {
//         Float2 o= origine + Float2(i*resolution, j*resolution);
// 
//         std::list<Float2> limit;
//         limit.push_back( o );
//         limit.push_back( o + Float2(resolution, 0) );
//         limit.push_back( o + Float2(resolution, resolution) );
//         limit.push_back( o + Float2(0, resolution) );
//         
//         switch( grid[i][j] )
//         {
//             case -1:
// //                fillPolygon( limit, 0x606060FF );
//                 break;
//             case 0:
//                 fillPolygon( limit, 0xC0C0C0FF );
//                 break;
//             default:
//                 fillPolygon( limit, color[grid[i][j]%nbc] );
//         }
//     }
// }

void Frame::drawVisiMap( const VisiMap & vm, const Float2 & delta, bool info )
{
    /*
    const unsigned int nbc(14);
    Color color[nbc]= {
         Color(0x800000FF), Color(0x606060FF), Color(0x000080FF), Color(0x000000FF),
         Color(0x404000FF), Color(0x004040FF), Color(0x400040FF), Color(0x602000FF), 
         Color(0x600020FF), Color(0x006020FF), Color(0x206000FF), Color(0x200060FF),
         Color(0x002060FF), Color(0x008000FF)
    };
    */
    
    Color cType[2][Node2::type_size]= {
        { Color(0x1010A0A0), Color(0xB01010A0), Color(0x10B010A0) }, // type_free, type_obstacle, type_frontier
        { Color(0x1010A010), Color(0xB0101010), Color(0x10B01010) } // type_free, type_obstacle, type_frontier
    };

    // Frame :
    drawRect( delta+vm.a_origin, vm.a_size, Color(0x1010AAFF) );
    
    Graph2::vertex_iterator vert, verEnd;
    for( boost::tie(vert, verEnd)= boost::vertices( vm.a_map ) ; vert != verEnd ; ++vert )
    {
//         cout << vm.a_map[*vert] << endl;
//         fillCircle( delta + vm.a_map[*vert], vm.a_epsilon, color[ (int)(*vert)%nbc ] );
//         drawCircle( delta + vm.a_map[*vert], vm.a_epsilon, color[ (int)(vm.a_map[*vert].type)%nbc ] );
        
        fillCircle( delta + vm.a_map[*vert], vm.a_epsilon, cType[1][ (int)(vm.a_map[*vert].type) ] );
        drawCircle( delta + vm.a_map[*vert], vm.a_epsilon, cType[0][ (int)(vm.a_map[*vert].type) ] );
    }

    Graph2::edge_iterator ited, end;
    for( boost::tie(ited, end)= boost::edges( vm.a_map ) ; ited != end ; ++ited )
    {
        Graph2::vertex_descriptor d_src( boost::source(*ited, vm.a_map) );
        Float2 src( vm.a_map[d_src] ), trg( vm.a_map[boost::target(*ited, vm.a_map)] );
        
//         drawLine(delta+src, delta+trg, color[ (int)(vm.a_map[d_src].type)%nbc ]);
        drawLine(delta+src, delta+trg, cType[0][ (int)(vm.a_map[d_src].type) ] );
    }
    
    if(info)
        for( boost::tie(vert, verEnd)= boost::vertices( vm.a_map ) ; vert != verEnd ; ++vert )
            write( std::to_string( (int)(*vert) ).c_str(), delta + vm.a_map[*vert], 0xA0A0A0FF);
}

void Frame::drawVisiMapObstacle( const VisiMap & vm, const Float2 & delta )
{   
    Color cType[2]= { Color(0xB01010FF), Color(0xB0101080) };

    // Frame :
    drawRect( delta+vm.a_origin, vm.a_size, Color(0x1010AAFF) );
    
    Graph2::vertex_iterator vert, verEnd;
    for( boost::tie(vert, verEnd)= boost::vertices( vm.a_map ) ; vert != verEnd ; ++vert )
        if( vm.a_map[*vert].type ==  mia::Node2::type_obstacle )
    {  
        fillCircle( delta + vm.a_map[*vert], vm.a_epsilon, cType[1] );
        drawCircle( delta + vm.a_map[*vert], vm.a_epsilon, cType[0] );
    }

    Graph2::edge_iterator ited, end;
    for( boost::tie(ited, end)= boost::edges( vm.a_map ) ; ited != end ; ++ited )
    {
        Graph2::vertex_descriptor d_src( boost::source(*ited, vm.a_map) );
        
        if( vm.a_map[d_src].type == mia::Node2::type_obstacle )
        {
            Float2 src( vm.a_map[d_src] ), trg( vm.a_map[boost::target(*ited, vm.a_map)] );
            drawLine(delta+src, delta+trg, cType[0] );
        }
    }
}
