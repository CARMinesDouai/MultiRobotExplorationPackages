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

#ifndef MIA_GLOBAL_H
#define MIA_GLOBAL_H

#include <random>
#include <boost/concept_check.hpp>
//#include <yaml.h>
#include <yaml-cpp/yaml.h>

namespace mia{
    
class Global
{
protected :
    static std::default_random_engine as_random_generator;
    static bool as_initialized;

public :
    static YAML::Node config;

    static bool initialize( const char * path_config );

    static int random(int min, int max){
        std::uniform_int_distribution<int> distribution(min, max);
        return distribution(as_random_generator);
    }

//     std::default_random_engine generator;
// std::uniform_int_distribution<int> distribution(1,6);
// int dice_roll = distribution(generator);  // generates number in the range 1..6 
// auto dice = std::bind ( distribution, generator );
// int wisdom = dice()+dice()+dice();
};

};

#endif // GLOBAL_H
