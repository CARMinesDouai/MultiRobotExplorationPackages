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

#include "data.h"
#include <string.h>

using namespace mia;
using namespace std;

/// Constructor destructor :
///-------------------------
Data :: Data( const char * msg, int fSize, int vSize ):
        a_mesage_size( strlen(msg) ), a_flag_size(fSize), a_value_size(vSize)
{
    a_mesage= new char[ a_mesage_size + 1 ];
    strcpy(a_mesage, msg);
    
    a_flag= new int [max(a_flag_size, 1)];
    a_value= new float[max(a_value_size, 1)];
}

Data :: Data( const Data & other ):
        a_mesage_size( other.mesage_size() ), a_flag_size( other.flag_size() ), a_value_size( other.value_size() )
{
    a_mesage= new char[ a_mesage_size + 1 ];
    strcpy(a_mesage, other.mesage());
    
    a_flag= new int [max(a_flag_size, 1)];
    for( int i=0 ; i < a_flag_size; ++i )
        a_flag[i]= other.flag(i);

    a_value= new float[max(a_value_size, 1)];
    for( int i=0 ; i < a_value_size; ++i )
        a_value[i]= other.value(i);
}

Data :: Data():
        a_mesage_size(0), a_flag_size(0), a_value_size(0)
{
    a_mesage= new char[ a_mesage_size + 1 ];
    a_mesage[0]= 0;
    
    a_flag= new int [max(a_flag_size, 1)];
    a_value= new float[max(a_value_size, 1)];
}

Data :: ~Data()
{
    delete [] a_mesage;
    delete [] a_flag;
    delete [] a_value;
}

/// affectation :
///--------------
void Data :: initialize( int msgSize, int flagSize, int valueSize)
{
    // free arrays
    delete [] a_mesage;
    delete [] a_flag;
    delete [] a_value;
 
    // set new arrays of data
    a_mesage_size= msgSize;
    a_mesage= new char[ a_mesage_size + 1 ];
    bzero(a_mesage, a_mesage_size+1 );
    
    a_flag_size= flagSize;
    a_flag= new int [max(a_flag_size, 1)];
    
    a_value_size= valueSize;
    a_value= new float[max(a_value_size, 1)];
}

void Data :: initialize( const char * msg, int flagSize, int valueSize)
{
    // free arrays
    delete [] a_mesage;
    delete [] a_flag;
    delete [] a_value;
 
    // set new arrays of data
    a_mesage_size= strlen(msg);
    a_mesage= new char[ a_mesage_size + 1 ];
    strcpy(a_mesage, msg);
    
    a_flag_size= flagSize;
    a_flag= new int [max(a_flag_size, 1)];
    
    a_value_size= valueSize;
    a_value= new float[max(a_value_size, 1)];
}

void Data :: initialize( const Data & other )
{
    // free arrays
    delete [] a_mesage;
    delete [] a_flag;
    delete [] a_value;
 
    // set new arrays of data
    a_mesage_size= other.mesage_size();
    a_mesage= new char[ a_mesage_size + 1];
    strcpy(a_mesage, other.mesage());
    
    a_flag_size= other.flag_size();
    a_flag= new int [max(a_flag_size, 1)];
    for( int i=0 ; i < flag_size(); ++i )
        a_flag[i]= other.flag(i);
    
    a_value_size= other.value_size();
    a_value= new float[max(a_value_size, 1)];
    for( int i=0 ; i < a_value_size; ++i )
        a_value[i]= other.value(i);
}

/// Comparison :
///-------------

/*
bool Data :: operator==(const Data & other)const
{
        return type == other.type && mesage == other.mesage
                                && flag == other.flag && value == other.value;
}

bool Data :: a_tor!=(const Data & other)const
{
        return ! operator==(other);
}
*/

/// log : 
///-------
Data::Data( std::istream & log )
{
    log >> a_mesage_size >> a_flag_size >> a_value_size;

    a_mesage= new char[ a_mesage_size + 1 ];
    for( int i(0); i < a_mesage_size ; ++i )
        log >> a_mesage[i];
    a_mesage[a_mesage_size]= 0;

    a_flag= new int [max(a_flag_size, 1)];
    for( int i=0; i < a_flag_size; ++i )
        log >> a_flag[i];

    a_value= new float[max(a_value_size, 1)];
    for( int i=0; i < a_value_size; ++i )
        log >> a_value[i];
}

void Data::log(std::ostream & os)const
{
    os << a_mesage_size << " " << a_flag_size << " " << a_value_size << " ";

    os << a_mesage << "\n";

    if( a_flag_size > 0 )
    {
        os << a_flag[0];
        for (int i= 1; i < a_flag_size; ++i )
            os << " " << a_flag[i];
        os << "\n";
    }
    
    if( a_value_size > 0 )
    {
        os << a_value[0];
        for (int i= 1; i < a_value_size; ++i )
            os << " " << a_value[i];
        os << "\n";
    }
}

/// YAML :
///---------
Data::Data( const YAML::Node & yaml ):
    a_mesage_size(0),
    a_flag_size( yaml["flag"].size() ), a_value_size( yaml["value"].size() )
{
    std::string msg= yaml["mesage"].as<std::string >();
    a_mesage_size= msg.length();
    
    a_mesage= new char[max(a_mesage_size, 1)+1];
    strcpy( a_mesage, msg.c_str() );
    
    a_flag= new int [max(a_flag_size, 1)];
    for( int i=0; i < a_flag_size; ++i )
        a_flag[i]= yaml["flag"][i].as<int>();
    
    a_value= new float[max(a_value_size, 1)];
    for( int i=0; i < a_value_size; ++i )
        a_value[i]= yaml["value"][i].as<float>();
}




void Data::yaml(std::ostream & os, int tabulation)const
{
    char * tab= new char [ (tabulation+2)*4 ];
    strcpy(tab, "");
    for(int i(0); i < tabulation; ++i)
        strcat(tab, "    ");

    os << tab << "  - " << "mesage: " << mesage() << "\n";

    strcat(tab, "    ");

    if ( a_flag_size > 0 ){
        os << tab << "flag: [" << a_flag[0];
        for (int i= 1; i < a_flag_size; ++i )
            os << ", " << a_flag[i];
        os << "]\n";
    }

    if ( a_value_size > 0 ){
        os << tab << "value: [" << a_value[0];
        for (int i= 1; i < a_value_size; ++i )
            os << ", " << a_value[i];
        os << "]\n";
    }
}

ostream & mia :: operator<< ( ostream & os, const Data & d )
{
        os << d.mesage() << "([";
        
        if ( d.flag_size() > 0 ){
            os << d.flag(0);
            for (int i= 1; i < d.flag_size(); ++i )
                os << ", " << d.flag(i);
        }
        
        os << "], [";
        
        if ( d.value_size() > 0 ){
            os << d.value(0);
            for (int i= 1; i < d.value_size(); ++i )
                os << ", " << d.value(i);
        }
        
        os << "])";

        return os; 
}
