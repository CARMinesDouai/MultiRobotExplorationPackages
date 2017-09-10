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

#ifndef MIA_MOINAG_H
#define MIA_MOINAG_H

// MArgent :
#include "agent.h"
#include "fsm.h"

// SiPlEn :
#include "visibility.h"
#include "particle.h"
#include "float2.h"

namespace mia {

enum ESensor
{
    pcpt_communication= 0,
    pcpt_position,
    pcpt_movement,
    pcpt_map,
    pcpt_wall,
    pcpt_mobile,
    pcpt_control,
    pcpt_coopmap,

    pcpt_number
};

enum EActuator
{
    act_communication= 0,
    act_move,

    act_number
};

class Moinag : public Agent
{
protected:
    fsm<Moinag, InteractStr, InteractStr> a_stateMachine;
    std::list<Float2> a_filterObs;
    std::vector<mia::Particle> a_obstacle;

    int a_bodyId;
    Particle a_body, a_lastBody;
    float a_epsilon;

    Visibility a_topo;
    LocalModel a_localMap;
    int a_targetVertex;
    
    Float2 a_controlTarget;
    
    std::list<Float2> a_trace;
    float a_traceDistance2;

    int a_wait, a_waitCount;
    
    float a_perceptionRadius;

public:
    enum EAgent
    {
        agent_moinag= 0,

        agent_number
    };

    enum EState
    {
        state_wait= 0,
        state_default,
        state_new_target,

        state_number
    };



    Moinag(float eps, float robot_radius, float pcpt_radius);
    virtual ~Moinag();

    // Agent methods:
    //---------------
    virtual void born(){};
    virtual void act( const InteractStr &sensor, InteractStr &actuator );
    virtual void die(){};

    // Getter and Setter  :
    //---------------------
    virtual void setBodyIdentifier(int i){ a_bodyId= i; }
    virtual int getBodyIdentifier()const{ return a_bodyId; }
        
    virtual const Particle & body()const{ return a_body; }
    virtual float getEpsilon()const{ return a_topo.getEpsilon(); }
    virtual Float2 getControl()const{ return a_controlTarget; }

    virtual float getPerceptionRadius()const{ return  a_perceptionRadius; }
    
    virtual Float2 getTargetVertexPosition()const{
        if( a_topo.a_visibility.is_vertex_index( a_targetVertex ) )
            return a_topo.a_visibility.vertex( a_targetVertex ).position;
        return Float2();
    }
    virtual void setBodyRadius( float radius ){ a_body.radius= radius; }

//    virtual void setEpsilon(float epsilon){ a_epsilon= epsilon; }
    virtual void setEpsilon(float eps){ a_topo.setEpsilon(eps); }
    
    virtual void setPerceptionRadius(float value ){ a_perceptionRadius= value; }

    virtual const std::list<Float2> & trace()const{ return a_trace; }

    // Action :
    //---------
    
    // Perception :
    //-------------
    virtual void log( const InteractStr& sensor, std::ostream & os ) const;

    // Static Factory  :
    //------------------
    static Moinag * factory(int agentType);

    // State Machine :
    //----------------
    virtual int numberOfStates()const { return a_stateMachine.numberOfStates(); }   /// return the number of states
    virtual int getState()const { return a_stateMachine.getState(); }    /// return the current state number

    virtual int stateWait(const InteractStr& sens, InteractStr& act);
    virtual int stateDefault(const InteractStr& sens, InteractStr& act);
    virtual int stateNewTarget(const InteractStr& sens, InteractStr& act);

//     virtual int state1(float dTime);
//     virtual int state2(float dTime);

    // State Tools :
    //--------------
    virtual int processSensor(const InteractStr& sens);
    virtual int processLocal();
    virtual int processControl(InteractStr& act);

    // drawing         :
    //------------------
    virtual const std::vector<mia::Particle> * obstacleList() const { return &a_obstacle; };
    virtual const std::list<mia::Float2> * ptrFilteredObstacleList() const { return &a_filterObs; };
    
    virtual const BoostTopo * local() const { return &(a_localMap.a_local); };
    virtual const Visibility * visibility() const { return &(a_topo); };
    
    virtual Particle local( BoostTopo::vertex_descriptor i ) const { return Particle( a_localMap.a_local[i] ); };
    virtual Particle visibility( int i ) const { return Particle( a_topo.a_visibility.vertex(i).position, a_topo.getEpsilon() ); };
    virtual void load_visibility( const std::string &  file ){ a_topo.load(file); }
    virtual const std::list< pairOFvertices > & local2visibility()const{ return a_topo.a_loc2vis; };
};
};

#endif // MIA_MOINAG_H
