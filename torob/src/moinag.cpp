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

#include "global.h"
#include "moinag.h"
#include "tools.h"
#include <iostream>

#include <array>

using namespace :: mia;
using namespace :: std;

Moinag::Moinag(float eps, float robot_radius, float pcpt_radius):Agent(pcpt_number, act_number),
    a_stateMachine(this, state_number, &Moinag::stateDefault),
    a_body( 0.f, 0.f, robot_radius), //Global::config["robot"]["radius"].as<float>() ),
    a_epsilon( eps ), //Global::config["robot"]["epsilon"].as<float>() ),// 0.06f
    a_topo(eps, pcpt_radius),
    a_controlTarget( 100.f, 0.f ),
    a_traceDistance2( 0.8*0.8 ),
    a_wait(1),
    a_waitCount(0),
    a_perceptionRadius( pcpt_radius ) //Global::config["robot"]["perception"].as<float>() )
{
    a_stateMachine.setStateProcess( state_wait, &Moinag::stateWait );
    a_stateMachine.setStateProcess( state_new_target, &Moinag::stateNewTarget );
    a_stateMachine.setState(state_wait);

//     a_stateMachine.setStateProcess( state_2, &Moinag::state2 );
}

Moinag::~Moinag()
{
}

// Static Factory  :
//------------------
Moinag *  Moinag :: factory( int agentType )
{
    switch( agentType )
    {
    case agent_moinag :
        return new Moinag( Global::config["robot"]["epsilon"].as<float>(), Global::config["robot"]["radius"].as<float>(), Global::config["robot"]["perception"].as<float>() );
    default :
        return new Moinag( Global::config["robot"]["epsilon"].as<float>(), Global::config["robot"]["radius"].as<float>(), Global::config["robot"]["perception"].as<float>() );
    }
}

// state machine   :
//------------------
void Moinag :: act( const InteractStr &sensor, InteractStr &actuator ){
    for( int iact(0); iact < actuator_size() ; ++iact )
        actuator[iact].clear();

    cout << "Moinag process...\n";
    a_stateMachine.process(sensor, actuator);
    
    cout << "\t visibility map with " << a_topo.a_visibility.edge_size()  << " edges\n"; 
    cout << "Moinag process end" << endl;
}

// State Tools :
//--------------
int Moinag :: processSensor(const InteractStr& sensor) {

    list<Particle> obstacle;
    Particle move;
    Particle lastBody= a_body;

    int count(0);

//     for( list<Data>::const_iterator it= a_sensor[pcpt_communication].begin(); it != a_sensor[pcpt_communication].end(); ++it )
//         ;

    for( list<Data>::const_iterator it= sensor[pcpt_position].begin(); it != sensor[pcpt_position].end(); ++it )
    {
        a_body= Particle(*it);
    }

    for( list<Data>::const_iterator it= sensor[pcpt_movement].begin(); it != sensor[pcpt_movement].end(); ++it )
    {
        move= Particle( *it );
        a_body.position+= move.position;
        a_body.theta= reduceRadian( a_body.theta + move.theta);
        a_body.radius= a_body.radius + move.radius;
    }

    for( list<Data>::const_iterator it= sensor[pcpt_map].begin(); it != sensor[pcpt_map].end(); ++it )
    {
        //Get meta-data :
        int width( it->flag(0) ), height( it->flag(1) );
        float resolution( it->value(0) );
        
        cout << "\t" << it->mesage() << " " << width << "x" << height << endl;

        //Clear map :
        a_topo.clear();
        a_topo.setEpsilon(resolution*0.999f);
        
        int ** node = new int*[width];
        int obs_dist= 1;
        
        //Copy map:
        for( int i(0); i < width ; ++i )
        {
            node[i]= new int[height];
            for( int j(0); j < height ; ++j )
            {
                node[i][j]= it->flag(2+i+j*width);
            }
        }

        //threshold:
        for( int i(0); i < width ; ++i )
        {
            for( int j(0); j < height ; ++j )
            {
                if( node[i][j] > 0 )
                    node[i][j]= 100;
            }
        }
        
        //filtering too little free area;
        for(bool map_ok(false) ; !map_ok ; map_ok= true )
        {
            for( int i(0); i < width ; ++i )
                for( int j(0); j < height ; ++j )
                {
                    int nbFreeNeiborg(0);
                    int i1(max(0, i-1)), i2(min(i+1, width-1));
                    int j1(max(0, j-1)), j2(min(j+1, height-1));
                
                    nbFreeNeiborg+= ( node[i1][j] == 0 )?1:0;
                    nbFreeNeiborg+= ( node[i2][j] == 0 )?1:0;
                    nbFreeNeiborg+= ( node[i][j1] == 0 )?1:0;
                    nbFreeNeiborg+= ( node[i][j2] == 0 )?1:0;
                    
                    if( node[i][j] == 0 && ( nbFreeNeiborg == 0 ||
                        nbFreeNeiborg == 1 && ( node[i1][j] == -1 || node[i2][j] == -1 || node[i][j1] == -1 || node[i][j2] == -1 ) ) )
                    {
                        node[i][j]= -1;
                        map_ok= false;
                    }
                }
        }
        
        //filtering thick obstatcle definition:
        for( int i(0); i < width ; ++i )
        {
            for( int j(0); j < height ; ++j )
            {
                int i1(max(0, i-1)), i2(min(i+1, width-1));
                int j1(max(0, j-1)), j2(min(j+1, height-1));
                
                if( node[i][j] == 100 && node[i1][j] != 0 && node[i2][j] != 0
                    && node[i][j1] != 0 && node[i][j2] != 0 )
                node[i][j]= -1;
            }
        }
        
        //From grid to vector:
        for( int i(0); i < width ; ++i )
        {
            for( int j(0); j < height ; ++j )
            {
                if( node[i][j] == 100 )
                {
                    Float2 p(i*resolution, j*resolution);
                    node[i][j]= a_topo.add_corner(p);
                    
                    for( int iiend(i), ii= max(0, i-obs_dist); ii < iiend ; ++ii )
                        for( int jjend( min(height, j+1+obs_dist) ), jj= max(0, j-obs_dist); jj < jjend ; ++jj )
                        {
                            if( node[ii][jj] != -1 )
                            {
                                a_topo.add_edge( node[i][j], node[ii][jj], Visibility::edge_obstacle );
                            }
                        }
                    for( int jjend(j), jj= max(0, j-obs_dist); jj < jjend ; ++jj )
                    {
                        if( node[i][jj] != -1 )
                        {
                            a_topo.add_edge( node[i][j], node[i][jj], Visibility::edge_obstacle );
                        }
                    }
                }
                else node[i][j]= -1;
            }
        }

        // topo clean :
        a_topo.clean();
    }

    for( list<Data>::const_iterator it= sensor[pcpt_wall].begin(); it != sensor[pcpt_wall].end(); ++it )
    {
        obstacle.push_back( Particle( *it ) );
    }
    
    if( a_trace.empty() || distance2( *(a_trace.rbegin()), a_body.position ) > a_traceDistance2 )
        a_trace.push_back( a_body.position );

    a_obstacle.resize( obstacle.size() );
    std::copy( obstacle.begin(), obstacle.end(), a_obstacle.begin() );
    
    return count;
}


int Moinag :: processLocal() {
    //a_obstacle :
    float epsilon2( a_epsilon*a_epsilon ), doubleEpsilon2(epsilon2*4);
    float safeDist( ( a_epsilon+a_body.radius*1.1f ) ), _2safeDist2( safeDist*safeDist*4 );
    
    Particle::sortAngle( a_obstacle );

    if( a_obstacle.size() > 1 ){

        a_filterObs= Particle::polarFiltering( a_obstacle, a_epsilon);

        vector<Particle> filterOdst( a_filterObs.size() );
        int size(0);
        for( list<Float2>::iterator it= a_filterObs.begin(); it != a_filterObs.end() ; ++it ){
            filterOdst[size].position= *it;
            filterOdst[size].radius= a_epsilon;
            ++size;
        }

        // Initialize local entrance/exit :
        list< int > section;
        
        for(int i= 1; i < filterOdst.size(); ++i){
            if( distance2(filterOdst[i].position, filterOdst[i-1].position) > _2safeDist2 )
            {
                section.push_back(i-1);
                section.push_back(i);
            }
        }
        if( distance2( filterOdst[0].position, filterOdst[size-1].position ) > _2safeDist2 )
        {
            section.push_front(0);
            section.push_back( filterOdst.size()-1 );
        }
        else{
            section.push_back( *section.begin() );
            section.pop_front();
        }

        
        if( section.size() == 0 ){
            // The farest obstacle from the agent :
            int iMax1(0), iMax2(0);
            float max2= 0.f;
            
            for(int i= 1; i < filterOdst.size(); ++i){
                float dist2= filterOdst[i].position.length2();
                if( dist2 > max2 )
                {
                    iMax1= i;
                    max2= dist2;   
                }
            }
            max2= 0.f;

            // The farest obstacle from the first farest obstacle :
            for(int i= 1; i < filterOdst.size(); ++i){
                float dist2= distance2Between( filterOdst[iMax1].position, filterOdst[i].position );
                if( dist2 > max2 )
                {
                    iMax2= i;
                    max2= dist2;   
                }
            }
            
            section.push_back( iMax1 );
            section.push_back( iMax2 );
            section.push_back( iMax2 );
            section.push_back( iMax1 );
        }
        
        
        // Rafine section (target multiple linear regression) :
        list<int>::iterator it2, it1= section.begin();
        while(  it1 != section.end() ){
            it2= it1;
            ++it2;
            
            int select(-1);
            float maxDist= 0.0;
            float angle= ( filterOdst[*it2].position - filterOdst[*it1].position ).angle();
            
            for(int i= *it1 ; i != *it2 ; i= (i+1)%filterOdst.size() )
            {   
                Float2 p= filterOdst[i].position.toBasis( angle, filterOdst[*it1].position );
                p.y= fabsf( p.y );
                if( p.y >  maxDist ){
                    maxDist= p.y;
                    select= i;
                }
            }
            
            if( maxDist > a_epsilon ){        
                section.insert( it2, select );
                section.insert( it2, select );
            }
            else{
                ++it1;
                ++it1;
            }
        }

        // TODO linear Regression section per section ? :
        
        // Build local topology :

        a_localMap.initialize( a_body, filterOdst, section, a_epsilon, safeDist );
        // TODO initializeObstacles with list<Float2> wall, float epsilon, foat safeDistance : using particle fusion.

        a_localMap.actualize();
    }

    return boost::num_vertices( a_localMap.a_local );
}

int Moinag :: processControl(InteractStr& actuator)
    {
        float angle( a_controlTarget.angle() );
        float dist( a_controlTarget.length() );
        
        float rotation= 1.f;
        float speed= 2.0f;
        
        if( angle < 0.f ){
            rotation= -1;
            angle*= -1.f;
        }

        if( angle < 0.01f )
            rotation= 0.f;
        else if( angle < 0.1f )
            rotation*= 0.2f;
        else if( angle < 0.5f )
            rotation*= 0.8f;
        else if( angle < 1.2f )
            rotation*= 1.4f;
        
        if( dist < a_epsilon ){
            speed= 0.f;
            rotation= 0.f;
        }
        else if( dist < a_body.radius )
            speed*= 0.4f;

        Data control( "control", 0, 4 );
        control.a_value[0]= speed; // speed;
        control.a_value[1]= rotation; // rotation;
        control.a_value[2]= 0.f; // scale;
        control.a_value[3]= 0.f; // brake;

        actuator[ act_move ].push_back( control );

        return 1;
    }

// State Machine :
//----------------
int Moinag :: stateWait(const InteractStr & sensor, InteractStr &actuator){
    cout << "\tMoinag State Wait" << "\n";

    if( a_waitCount < 0 || a_waitCount > a_wait )
        a_waitCount= 0;

    int nbMsg= processSensor(sensor);
    processLocal();

    ++a_waitCount;

    if( a_waitCount == a_wait ){
        a_waitCount= 0;
        return state_new_target;
    }
    
    return state_wait;
}

int Moinag :: stateNewTarget(const InteractStr & sensor, InteractStr &actuator)
{
    cout << "\tMoinag State New Target" << "\n";
    
    stateDefault(sensor, actuator);
    
//     a_targetVertex= a_topo.a_visibility.random_vertex_index();
//     assert( a_topo.a_visibility.is_vertex_index( a_targetVertex ) );
//     cout << "target vertex :" << a_targetVertex << "\n";

    return state_default;
}

int Moinag :: stateDefault(const InteractStr & sensor, InteractStr &actuator) {
    cout << "\tMoinag State Default" << "\n";

    int nbMsg= processSensor(sensor);
    processLocal();

//     cout << ""\n"\t- Position: " << a_body.position << " angle: " << a_body.theta << "\n";
//     cout << "\t- Actualize visibility graph:" << "\n";
    a_topo.actualize( &a_localMap, a_body.position );

//     cout << "\t- " << a_topo.a_visibility.vertex_size() << " vertices and " << a_topo.a_visibility.edge_size() << " edges" << "\n";

//    a_topo.print(cout);

//     cout << "\t- Define control:" << "\n";
    
    a_controlTarget= a_localMap.getDirection( Float2( 100.f, 0.f ), LocalModel::particle_exit ).first.position;
    processControl(actuator);

//     a_targetVertex= a_topo.a_visibility.random_vertex_index();
//     assert( a_topo.a_visibility.is_vertex_index( a_targetVertex ) );

    return state_default;
}


void Moinag :: log( const InteractStr& sensor, std::ostream & os ) const
{
    /// log perceived data
    os << sensor.size() << "\n";
    
    for(unsigned int i(0) ; i < sensor.size() ; ++i )
    {
        os << sensor[i].size() << "\n";
        for( list<Data>::const_iterator itEnd( sensor[i].end() ), it( sensor[i].begin() );
            it != itEnd ; ++it )
        {
            it->log(os);
        }
    }
}

