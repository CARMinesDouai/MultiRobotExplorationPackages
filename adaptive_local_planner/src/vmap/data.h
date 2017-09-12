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

#ifndef mia_DATA_H
#define mia_DATA_H

// System :
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace mia {
/**
 * Data give a structure to all type of transmited informations. It is used for the interactions between the agent and the environement.
 *
 */

// class AbData
// {
// public:
//     virtual int mesage_size()const = 0;
//     virtual const char * mesage()const = 0;
//     virtual int flag_size()const = 0;
//     virtual int flag(int i)const = 0;
//     virtual int value_size()const = 0;
//     virtual float value(int i)const = 0;
// 
// //    virtual void initialize(const AbData &)= 0;
// };

class Data //: public AbData
{
public:
        /// a key in how to process the Data
        int a_mesage_size, a_flag_size, a_value_size;

        /// string mesage
        char * a_mesage;

        /// table of interger values
        int * a_flag;

        /// table of floating point values
        float * a_value;

        // Constructor destructor :
        //-------------------------
        Data( const char * msg, int flagSize, int valueSize);
        Data( const Data & other );
        Data( std::istream & log );
        Data( const YAML::Node & yaml );
        Data();

        virtual ~Data();

        // AbData getter :
        //----------------
        virtual int mesage_size()const{ return a_mesage_size; };
        inline virtual const char * mesage()const{ return a_mesage; };
        inline virtual int flag_size()const{ return a_flag_size; };
        inline virtual int flag(int i)const{ return a_flag[i]; };
        inline virtual int value_size()const{ return a_value_size; };
        inline virtual float value(int i)const{ return a_value[i]; };

        // AbData setter :
        //----------------
//        virtual int mesage_size(){ return a_mesage_size; };
//        inline virtual const char * mesage(){ return a_mesage; };
//        inline virtual int flag_size(){ return a_flag_size; };
        inline virtual void flag(int i, int f){ a_flag[i]= f; };
//        inline virtual int value_size(){ return a_value_size; };
        inline virtual void value(int i, float v){ a_value[i]= v; };

        // AbData initialization :
        //------------------------
//        virtual void initialize(const AbData &);
        virtual void initialize( int, int, int );
        virtual void initialize( const Data & );
        virtual void initialize( const char * msg= "", int flagSize= 0, int valueSize= 0 );

        virtual const Data & operator=(const Data & d)
        {
            initialize(d);
            return *this;
        }

        // Comparison :
        //-------------
/*
        virtual bool operator==(const Data &)const;
        virtual bool operator!=(const Data &)const;
*/

        // Stream :
        //---------
        void log(std::ostream & os)const;
        void yaml(std::ostream & os, int tabulation= 0)const;
        friend std::ostream & operator<< ( std::ostream & os, const mia::Data & d );
};

/*
class DataProcessor
{
protected:
        DataProcessor(){}

public:
        virtual void processData( const Data & da )= 0;
};
*/
/*
template<class TEnvironment >
class data_processor : public DataProcessor
{
        protected:
        TEnvironment * a_environment;

        typedef void (TEnvironment::*ptrDataProcess)( const Data & );

        int a_dataProcessorSize;
        ptrDataProcess * a_dataProcessor;

        public:
        /// Constructor Destructor:
        ///------------------------
        data_processor<TEnvironment> ( TEnvironment * env, int nboDataProcessor= 1 ):
                a_environment( env ),
                a_dataProcessorSize( nboDataProcessor )
        {
                assert( 0 < a_dataProcessorSize );
                a_dataProcessor= new ptrDataProcess[ a_dataProcessorSize ];
        }

        virtual ~data_processor<TEnvironment>()
        {
                delete [] a_dataProcessor;
        }

        /// Process:
        ///---------
        virtual void processData( const Data & da )
        {
                assert( 0 <= da.a_reference && da.a_reference < a_dataProcessorSize );
                (a_environment->*a_dataProcessor[ da.a_reference ])( da );
        }

        /// Getter and Setter:
        ///-------------------
        virtual int numberOfDataProcessor()const{ return a_dataProcessorSize; }

        virtual void setDataProcessor(int iDP, ptrDataProcess process )
        {
                assert( 0 <= iDP && iDP < a_dataProcessorSize );
                a_dataProcessor[ iDP ]= process;
        }
};
*/

/// Stream :
///---------
std::ostream & operator<< ( std::ostream & os, const mia::Data & d );
};

#endif // DATA_H
