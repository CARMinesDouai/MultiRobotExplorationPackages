#ifndef MIA_FSM_H
#define MIA_FSM_H

#include <assert.h>

namespace mia
{

/// template for finit state machine associated to Class methode

template<class Obj, class TSensor, class TActuator>
class fsm
{
    Obj * a_target;  /// Class entity

    const int a_nboState; /// total number of States
    int a_state; /// curent state number

    typedef int (Obj::*ptrState)(const TSensor &, TActuator &);
    ptrState * a_stateMachine; /// table of state process functions : typedef int (Obj::*ptrStateProcess)(float)

//	stateRunner();

public:
    // Constructor Destructor:
    //------------------------
    fsm<Obj, TSensor, TActuator>(Obj * target, int nboState, ptrState defaultStateProcess ):
        a_target(target),
        a_nboState(nboState),
        a_state(0)
    {
        /** Initialize the fsm with :
         * \param subject Class entity
         * \param nboState number of states (constant and positive)
         * \param defaultStateProcess the intial process functions set to all states
        **/

        assert( 0 < a_nboState );

        a_stateMachine= new ptrState[a_nboState];
        for( int i=0; i < a_nboState; ++i)
            a_stateMachine[i]= defaultStateProcess;
    }

    virtual ~fsm<Obj,TSensor, TActuator>()
    {
        delete [] a_stateMachine;
    }

    // Process:
    //---------
    virtual void process(const TSensor & sensor, TActuator & actuator )
    {
        /// process method : call the current state process and actualize the current state.
        a_state= ( (a_target->*a_stateMachine[ a_state ])(sensor, actuator) ) % a_nboState;
    }

    // Getter and Setter:
    //-------------------
    virtual int numberOfStates()const {
        return a_nboState;    /// return the number of states
    }
    virtual int getState()const {
        return a_state;    /// return the current state number
    }
    virtual void setState( int iState ) {
        a_state= iState;    /// force the current state number to iState
    }
    virtual bool isInState( int st )const {
        return (st == a_state);    /// return true if the current state number is st
    }

    virtual void setStateProcess(int iState, ptrState process )
    {
        /** associate a process methode of the entity a_subject to a specifica state numbre
         * \param iState state number conserned
         * \param process pointer on the methode
        **/

        assert( 0 <= iState && iState < a_nboState );
        a_stateMachine[ iState ]= process;
    }
};

};

#endif // MIA_FSM_H
