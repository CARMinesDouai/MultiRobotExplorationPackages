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

#ifndef MIA_ENVIRONMENT_H
#define MIA_ENVIRONMENT_H

// System

// Boost:

// #include <boost/thread.hpp>

// MoInAg
#include "collection.h"
#include "agent.h"

namespace mia
{

/**
 *  Manadge a collection of agents (TODO: in several treads.)
 * \param TAgent the main application agent class. Could be mai::Agent
 * \param agentFactory is a function allowing the environment to create several kind of agents derived from TAgent. The function take 2 parameters: ( int id, int type) the new identifier of the created TAgent and a type attribute
 */

template<class TAgent, TAgent* tnursery(int) >
class listias_environment
{
public:
    typedef typename TAgent::InteractStr InteractStr;
//    typedef agent<typename TAgent::InteractStr, TAgent::nbSensor, TAgent::nbActuator> Basic;

    // Interaction structure:
    //-----------------------
//     typedef std::array<InteractStr, TAgent::nbSensor> Sensor;
//     typedef std::array<InteractStr, TAgent::nbActuator> Actuator;

protected:

    collection< TAgent* > a_agent;
    collection< InteractStr > a_agent_sensor;
    collection< InteractStr > a_agent_actuator;

public:
    // Constructor / Destructor :
    //---------------------------
    listias_environment<TAgent, tnursery>( int size= 256 ):
        a_agent(size)
    {
        /**
         *      \param size initialize a first size capacity of the TAgent collection
         */
    }

    virtual ~listias_environment<TAgent, tnursery>()
    {
//         /**
//          * Kill all TAgent by calling the die() method:
//          */
// 
//         for ( int iA= 0; iA < a_agent.limit(); ++iA )
//             if( a_agent.isId(iA) )
//                 // Process agent
//                 a_agent[iA]->die();

        /**
         * Delete all TAgent.
         */
        for ( int ia= 0; ia < a_agent.limit(); ++ia )
             if( isAgent(ia) )
                 delete a_agent[ia];
    }

    // getter:
    //--------
    virtual bool isAgent( int id )
    {
        /**
         * Tchek if an agent exist
         * \param id identifier of the agent
         */

        return a_agent.is_index( id );
    }
    
    virtual int createAgent( int type= 0 )
    {
        /**
         * Create an agent by calling the nursery fonction and call the agent born() method.
         * \param type is passed to the nursery function in order to identify the expected derived of TAgent to create
         */
        
        int idAgent= a_agent.push( tnursery( type ) );
        assert( idAgent == a_agent_sensor.push( a_agent[idAgent]->generate_sensor() ) );
        assert( idAgent == a_agent_actuator.push( a_agent[idAgent]->generate_actuator() ) );

        a_agent[idAgent]->born();
        return idAgent;
    }

    virtual void kill_agent(int id)
    {
        /**
         * kill an agent by calling the die() method and then delete it from the collection.
         * \param id identifier of the agent to kill
         */
        a_agent[id]->die();
        
        delete a_agent[id];
        
        a_agent.remove(id);
        a_agent_sensor.remove(id);
        a_agent_actuator.remove(id);
    }

    // Agent supervision:
    //-------------------
    virtual void processAgents()
    {
        //Clear agent sensor: 
        
        //Resolve agent action:
        
        //Spin one global physic laws:
        
        // process Agent by calling act method:
        for ( int iaEnd(a_agent.limit()), ia= 0; ia < iaEnd; ++ia )
            if( isAgent(ia) )
            {
                // agent act:
                a_agent[ia]->act( a_agent_sensor[ia], a_agent_actuator[ia] );
                
                // clear the agent sensors.
                for( int isensend( a_agent[ia]->sensor_size() ), isens(0); isens < isensend ; ++isens )
                    clearSensor( ia, isens );
            }
    }

//    virtual void processActions()= 0;
//     {
//         // For each Agent:
//         for ( int iaEnd(a_agent.limit()), ia= 0; ia < iaEnd; ++ia )
//             if( isAgent(ia) )
//             {
//                 // Process each actuator :
//                 for( int iact(0); iact < TAgent::nbActuator; ++iact )
//                     processAction( ia, iact, a_agent_actuator[ia][iact] );
//             }
//     }
    
//    virtual void processAction( int ia, int iact, const InteractStr & act_dsc )= 0;
//    {
        /**
         *  Process an agent actuator. The function is called for all agent in environement process method
         *  \param iag the actor agent identifier
         *  \param iact the agent actuator identifier
         *  \param act_dsc action descriptor
         *
         * By default, it do nothing. It is the main environment method to redefine.
         *
         */
//    }

    virtual void clearSensor( int ia, int isens)
    {
        /**
         *  Process an agent actuator. The function is called for all agent in environement process method
         *  \param iag the actor agent identifier
         *  \param isens the agent sensor identifier
         *
         *  By default, it call clear() method in the sensor. 
         *  Attention that require clear() implementation on the TAgent::InteractStr (as using std::list)  
         *
         */
        a_agent_sensor[ia][isens].clear();
    }

    virtual TAgent & agent( int ia )
    {
        return *a_agent[ia];
    }
    
    virtual const TAgent & agent( int ia )const
    {
        return *a_agent[ia];
    }
    
    virtual InteractStr & sensor( int ia )//, int isens )
    {
        return a_agent_sensor[ia];//[isens];
    }

    virtual const InteractStr & sensor( int ia )const//, int isens )const
    {
        return a_agent_sensor[ia];//[isens];
    }
};

};

#endif // MIA_ENVIRONMENT_H
