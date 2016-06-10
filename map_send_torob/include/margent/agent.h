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

#ifndef MIA_AGENT_H
#define MIA_AGENT_H

#include "data.h"
#include <list>
#include <boost/graph/graph_concepts.hpp>

namespace mia{

/**
 *  the lead class of the project. An agent evolving in an environment witch call the Agent::precess() method at each step. The process methode
 *  allow the agent to actualize it action regarding the received perception.
 *
 */

// template<class TSensor, class TActuator >
// class agent{
//     /**
//     * \param TSensorStr sensor data structure
//     * \param TActuatorStr actuator data structure
//     */
//     typedef TSensor Sensor;
//     typedef TActuator  Actuator;
// 
//     // Main agent method:
//     //-------------------
//     virtual void born() =0;
//     virtual void act( const Sensor & sensor, Actuator & actuator ) =0;
//     virtual void die() =0;
// };

template<class TInteractStr>
class agent{
     /**
     * \param TInteractStr the data structure allowing interaction
     * \param tNbSensor the number of agent sensors
     * \param tNbActuator the number of agent actuator
     */
protected:
     const std::size_t a_sensor_size;
     const std::size_t a_actuator_size;

     agent<TInteractStr>( std::size_t sensor_size, std::size_t actuator_size):
         a_sensor_size(sensor_size),
         a_actuator_size(actuator_size)
     {}

public:

    virtual ~agent<TInteractStr>(){}

    // Interaction structure:
    //-----------------------
    typedef TInteractStr InteractStr;
//    typedef std::vector<TInteractStr> InteractStr;

//     typedef TInteractStr InteractStr;
//     typedef std::array<InteractStr, nbSensor> Sensor;
//     typedef std::array<InteractStr, nbActuator>  Actuator;

    // Main agent method:
    //-------------------
    virtual void born() =0;
    virtual void act( const InteractStr & sensor, InteractStr & actuator ) =0;
    virtual void die() =0;

    // sensor and actuator:
    //---------------------
    virtual std::size_t sensor_size()const{ return a_sensor_size; }
    virtual std::size_t actuator_size()const{ return a_actuator_size; }
     
    virtual InteractStr generate_sensor()const{ return InteractStr(a_sensor_size); };
    virtual InteractStr generate_actuator()const{ return InteractStr(a_actuator_size); };

    // Agent Descriptor:
    //------------------
//    static agent<TInteractStr, tNbSensor, tNbActuator>* nursery(const Descriptor &);

    // Agent supervision:
    //-------------------
//     virtual state;
//     virtual task;

    // In/out stream:
//     virtual state;
//     virtual task;
};

typedef agent< std::vector<std::list<Data>> > Agent;

};

#endif // AGENT_H
