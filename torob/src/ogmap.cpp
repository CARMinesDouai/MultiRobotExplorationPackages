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

#include <iostream>
#include "ogmap.h"

#include <fstream>
#include <algorithm>
#include <boost/graph/graph_concepts.hpp>

using namespace mia;
using namespace std;

OGMap::OGMap(int width, int height, const Float2 & origine, float resolution):
    a_width(width),
    a_height(height),
    a_origin(origine),
    a_resolution(resolution)
{
    a_grid= new int*[a_width];
    for( int i(0) ; i < a_width ; ++ i)
        a_grid[i]= new int[a_height];
}

OGMap::~OGMap()
{
    for( int i(0) ; i < a_width ; ++i )
        delete [] a_grid[i];
    delete [] a_grid;
}
    
// Test:
bool OGMap::is_cell(int i, int j )const
{
    return ( i >= 0 && i < a_width
	    && j >= 0 && j < a_height );
}

bool OGMap::is_neighbor_type(int i, int j, int type )const
{
    return ( (i > 0) && a_grid[i-1][j] == type )
            || ( (i+1 < a_width) && a_grid[i+1][j] == type )
            || ( (j > 0) && a_grid[i][j-1] == type )
            || ( (j+1 < a_height) && a_grid[i][j+1] == type );
}

void OGMap::initialize(const Data & map)
{
    for( int i(0) ; i < a_width ; ++i )
        delete [] a_grid[i];
    delete [] a_grid;

    //Get meta-data :
    a_width= map.flag(0);
    a_height= map.flag(1);

    a_resolution= map.value(0);
    a_origin= Float2(0.f, 0.f);
    if( map.value_size() >= 3 )
    {
       a_origin.x=  map.value(1);
       a_origin.y=  map.value(2);
    }
    
//    cout << "grid initialization:\n\t" << a_origin << ", " << a_resolution << ": " << a_width << ", " << a_height << endl;
    
    //Copy map:
    a_grid= new int * [a_width + 2];
    a_grid[0]= new int[a_height + 2];
    a_grid[a_width+1]= new int[a_height + 2];

    for( int i(0) ; i < a_width ; ++i )
    {
        a_grid[i+1]= new int[a_height + 2];
        for(int j(0); j < a_height ; ++j )
            a_grid[i+1][j+1]= map.flag( 2+i+j*a_width );
    }
    
    //Set frame :
    a_width+= 2;
    a_height+= 2;
    a_origin.x-=  a_resolution;
    a_origin.y-=  a_resolution;
    int maxi(a_width-1), maxj(a_height-1);
    
    for( int i(0) ; i < a_width ; ++i )
    { 
        a_grid[i][0]= -1;
        a_grid[i][maxj]= -1;
    }
    for( int j(0) ; j < a_height ; ++j )
    {
        a_grid[0][j]= -1;
        a_grid[maxi][j]= -1;
    }
    
    //Filtering
    reduce_filter(0, -1, 1);
    reduce_filter(-1, 0, 1);

    //Adapt frame :
//     for( int i(1) ; i < maxi ; ++i )
//     {
//         if( a_grid[i][1] == 0 )
//             a_grid[i][0]= 100;
// 
//         if( a_grid[i][maxj-1] == 0 )
//             a_grid[i][maxj]= 100;
//     }
//     
//     for( int j(1) ; j < maxj ; ++j )
//     {
//         if( a_grid[1][j] == 0 )
//             a_grid[0][j]= 100;
//         
//         if( a_grid[maxi-1][j] == 0 )
//             a_grid[maxi][j]= 100;
//     }
}

inline int reduce_value(int ** a_grid, int iStart, int jStart, int size){
  int v= 0;
  for(int i(iStart), iEnd(iStart+size) ; i < iEnd ; ++i)
    for(int j(jStart), jEnd(jStart+size) ; j < jEnd ; ++j){
      if( a_grid[i][j] > 0 )
	v= a_grid[i][j];
      else if( v < 1 && a_grid[i][j] < 0 )
	v= a_grid[i][j];
    }
  return v;
}

void OGMap::reduce(int scale){
  assert(scale > 0);
  
  std::cout << "OGMAP:: Reduce about: " << scale << endl;
  int ** new_grid;
  int new_width(a_width/scale), new_height(a_height/scale);

  std::cout << "From  (" << a_width  << ", " << a_height << ")";
  std::cout << " to (" << new_width  << ", " << new_height << ")" << endl;
  
  // Copy :
  new_grid= new int*[new_width];
  for( int i(0) ; i < new_width ; ++i){
    new_grid[i]= new int[new_height];
    int iScale= i*scale;
    for( int j(0) ; j < new_height ; ++j)
      new_grid[i][j]= reduce_value(a_grid, iScale, j*scale, scale);
  }

  // Clear :
  for( int i(0) ; i < a_width ; ++i )
    delete [] a_grid[i];
  delete [] a_grid;
  
  // Replace :
  a_grid= new_grid;
  a_width= new_width;
  a_height= new_height;
  a_resolution*= scale;
}

// Getter :


// Clearing :
void OGMap::clear(int i){
  removeRegionLowerThan(0, i);
  removeRegionunconnectedTo(0);
}

void OGMap::removeRegionLowerThan(int region, int size){

  std::list<Int2> saved;

  for( unsigned int i(0); i < a_width ; ++i ){
    for( unsigned int j(0); j < a_height ; ++j ){
      if( a_grid[i][j] == region ){
	std::list<Int2> r= OGMap::removeRegion(i, j);
	if( r.size() >= size ){
	  saved.splice (saved.end(), r);
	}
      }
    }
  }
  
  std::cout << "saved cell: " << saved.size() << std::endl;
  for( std::list<Int2>::const_iterator it( saved.begin()), itEnd(saved.end()) ; it != itEnd ; ++it)
    a_grid[it->x][it->y]= region;
}

std::list<Int2> OGMap::removeRegion(int i, int j){
  int r= a_grid[i][j];
  std::list<Int2> region;
  std::list<Int2> toVisit;
  toVisit.push_back( Int2(i, j) );
  a_grid[i][j]= -1;
  
  if( r == -1 )
    return region;
  
  int xx[]= {0, -1, 1, 0};
  int yy[]= {-1, 0, 0, 1};
  
  while( !toVisit.empty() ){
    Int2 cell= (*toVisit.begin());
    
    for(int i4(0) ; i4 < 4 ; ++i4 ){
      int ii= cell.x+xx[i4];
      int jj= cell.y+yy[i4];
      
      if( is_cell( ii, jj ) && a_grid[ii][jj] == r ){
	toVisit.push_back( Int2( ii, jj ));
	a_grid[ii][jj]= -1;
      }
    }
    
    region.push_back(cell);
    toVisit.pop_front();
  }
  
  return region;
}

void OGMap::removeRegionunconnectedTo(int region){
  int xx[]= {0, -1, 1, 0};
  int yy[]= {-1, 0, 0, 1};
  
  for( unsigned int i(0); i < a_width ; ++i ){
    for( unsigned int j(0); j < a_height ; ++j ){
      bool remove= (a_grid[i][j] != region && a_grid[i][j] != -1);
      for(int i4(0) ; remove && i4 < 4 ; ++i4 ){
	int ii= i+xx[i4];
	int jj= j+yy[i4];
	
	if( is_cell( ii, jj ) )
	  remove= a_grid[ii][jj] != region;
      }
      if( remove )
	a_grid[i][j]= -1;
    }
  }
}

// Filtering :
void OGMap::reduce_filter(int value1, int value2, int told)
{
    list<Int2> backtrac;

    for( int i(0) ; i < a_width ; ++i )
    {
        for(int j(0); j < a_height ; ++j )
        {
            if( a_grid[i][j] == value1 )
            {
                reduce_cell_filter( i, j, value1, value2, told, backtrac );
            }
        }
    }
    
    while( !backtrac.empty() )
    {
        Int2 i2= *(backtrac.begin());
        if( a_grid[i2.x][i2.y] == value1 )
            reduce_cell_filter( i2.x, i2.y, value1, value2, told, backtrac );
        backtrac.pop_front();
    }
}

void OGMap::reduce_cell_filter(int i, int j, int value1, int value2, int told, list<Int2> & backtrac)
{
    list<Int2> nbor= neighbor(i, j);

    int nbv1( 4 - nbor.size() );
    bool arev2(false);
    for( list<Int2>::iterator it(nbor.begin()), itEnd(nbor.end()); it != itEnd ; ++it )
    {
        if( a_grid[it->x][it->y] == value1 )
            nbv1+= 1;
        
        arev2= arev2 || (a_grid[it->x][it->y] == value2);
    }
    
    if( a_grid[i][j] == value1 && arev2 && nbv1 <= told )
    {
        a_grid[i][j]= value2;

        for( list<Int2>::iterator it(nbor.begin()), itEnd(nbor.end()); it != itEnd ; ++it )
            if( a_grid[it->x][it->y] == value1 )
                backtrac.push_back( *it );
    }
}

int OGMap::frontier(int cv1, int cv2, int fv)
{
    int count(0);
    
    for(int i(0) ; i < a_width ; ++i )
        for(int j(0) ; j < a_height ; ++j )
            if( a_grid[i][j] == cv1 )
    {
        bool existCv2(false);
        list<Int2> nbor= neighbor(i, j);
        for( list<Int2>::iterator it(nbor.begin()), itEnd(nbor.end()) ; it != itEnd ; ++it )
            existCv2= existCv2 || (a_grid[ it->x ][ it->y ] == cv2);
        
        if( existCv2 )
        {
            a_grid[i][j]= fv;
            ++count;
        }
    }

    return count;
}


// Attributes :
Float2 OGMap::center(int cellVal) const
{
    int count(0);
    Int2 center(0, 0);
    
    for( int i(0) ; i < a_width ; ++i )
        for(int j(0); j < a_height ; ++j )
            if( a_grid[i][j] == cellVal )
    {
        center+= Int2(i, j);
        ++count;
    }
            
    float invC= 1.0f/(float)(count);
    return gridToWorld( Float2( float(center.x)*invC, float(center.y)*invC ) );
}

// Cell attribute:
std::list<Int2> OGMap::neighbor(int i, int j)const
{
    list<Int2> nbor;

    if( i > 0 )
        nbor.push_back( Int2(i-1, j) );
    if( i < a_width-1 )
        nbor.push_back( Int2(i+1, j) );
    if( j > 0 )
        nbor.push_back( Int2(i, j-1) );
    if( j < a_height-1 )
        nbor.push_back( Int2(i, j+1) );

    return nbor;
}


// File interaction:
void OGMap::load( const std::string & file, Float2 origin, float resolution )
{
    cout << "OGMap load map " << file << endl;
    
    // Free :
    for( int i(0) ; i < a_width ; ++i )
        delete [] a_grid[i];
    delete [] a_grid;
    
    // Open file:
    std::ifstream is;
    is.open( file.c_str(), std::ios::in );

    // Set meta :
    a_origin= origin;
    a_resolution= resolution;

    char str[256];
    
//    is << "P5\n";

    is.getline( str, 256 ); // Type P5
//    cout << str << endl;
    
//  is << "# CREATOR: Khelifa map 0.050 m/pix\n"
    is.getline( str, 256 ); // # comment
//    cout << str << endl;
    
//    is << a_width << " " << a_height;
    
    is >> a_width >> a_height;

    is.read( str, 1 ); // get end_line
    is.getline( str, 256 ); // grey accurancy: 255
//    cout << str << endl;
    
    // Generate structure:

//    cout << "\tBuild " << a_width << "x" << a_height << " matrix" << endl;
 
    a_grid= new int*[a_width];
    for( int i(0) ; i < a_width ; ++ i)
        a_grid[i]= new int[a_height];
    
    // Read data :
    unsigned char c;
    list<unsigned char> val;
    
    int transform[255];
    transform[0]= 100;
    transform[205]= -1;
    transform[254]= 0;

    for( int j(a_height-1) ; j >= 0 ; --j )
        for( int i(0) ; i < a_width ; ++i )
        {
//              c= 0, 205 ou 254 
//              is << c;
            
            is >> c;
            a_grid[i][j]= transform[c];
        }

}

void OGMap::save( const std::string & file )const
{
  cout << "OGMap save map " << file << endl;

  // Open file:
  ofstream osfile;
  osfile.open( file, std::ofstream::out );
  
  char str[256];

  osfile << "P5\n";
  osfile << "# CREATOR: Torob::OGmap\n";
  osfile << a_width << " " << a_height << "\n";
  osfile << 255 << "\n";  // grey accurancy: 255

  // Write data :
  for( int j(a_height-1) ; j >= 0 ; --j )
    for( int i(0) ; i < a_width ; ++i )
    {
      switch( a_grid[i][j] ){
	case 0:
	  osfile << (char)254;
	  break;
	case 100:
	  osfile << (char)0;
	  break;
	default:
	  osfile << (char)205;
      }
    }

  osfile.close();
}
