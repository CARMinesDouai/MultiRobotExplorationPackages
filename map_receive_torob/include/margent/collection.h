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

#ifndef MIA_COLLECTION_H
#define MIA_COLLECTION_H

#include <list>
#include<random>

//#include <iostream>
#include <math.h>
#include <assert.h>

#include "global.h"

// #include <algorithm>

/**
 * Template Collection allocate an flexible array.
 *
 * TODO: reprendre le patron sans pointer et avec un array<bool> a_exist;
 *
 */

//extern std::pair r;
namespace mia {
/*
class Identified
{
 int a_id;
public :

 /// getter and setter :
 ///--------------------
 virtual inline void setId(int id){ a_id= id; };
 virtual inline int getId()const{ return a_id; };

 /// Order :
 ///--------
 virtual inline bool operator==(const Identified & n)const{ return ( a_id == n.a_id ); }
 virtual inline bool operator!=(const Identified & n)const{ return ( a_id != n.a_id ); }
 virtual inline bool operator<(const Identified & n)const{ return ( a_id < n.a_id ); }
 virtual inline bool operator>(const Identified & n)const{ return ( a_id > n.a_id ); }
 virtual inline bool operator<=(const Identified & n)const{ return ( a_id <= n.a_id ); }
 virtual inline bool operator>=(const Identified & n)const{ return ( a_id >= n.a_id ); }
};
*/

template<class Obj>
class collection
{
public :

    class iterator {
        friend collection<Obj>;
    protected :
        collection<Obj> * target;
        int a_index;

    public :
        iterator():target(nullptr),a_index(0) {};
        iterator(collection<Obj> * trg, int i ):target(trg),a_index(i) {};
        virtual ~iterator() {};

        virtual Obj & operator*() {
            return (*target)[a_index];
        }
        virtual const Obj & operator*()const {
            return (*target)[a_index];
        }

        virtual const iterator & operator++() {
            a_index= target->next_index(a_index);
            return *this;
        }

        virtual bool operator==( iterator it )const {
            return it.target == target && it.a_index == a_index;
        }
        virtual bool operator!=( iterator it )const {
            return it.target != target || it.a_index != a_index;
        }
        virtual int index()const {
            return a_index;
        }
    };

    class const_iterator {
        friend collection<Obj>;
    protected :
        const collection<Obj> * target;
        int a_index;

    public :
        const_iterator():target(nullptr),a_index(0) {};
        const_iterator(const collection<Obj> * trg, int i ):target(trg),a_index(i) {};
        virtual ~const_iterator() {};

        virtual const Obj & operator*()const {
            return (*target)[a_index];
        }

        virtual const const_iterator & operator++() {
            a_index= target->next_index(a_index);
            return *this;
        }

        virtual bool operator==( const_iterator it )const {
            return it.target == target && it.a_index == a_index;
        }
        virtual bool operator!=( const_iterator it )const {
            return it.target != target || it.a_index != a_index;
        }
        virtual int index()const {
            return a_index;
        }
    };

private :
    Obj ** a_item;
    std::list<int> a_freeId;
    int a_limit, a_capacity;

public :
    /// Constructor destructor :
    ///-------------------------
    collection(int cap = 16)
    {
        a_limit= 0;
        a_capacity= cap;
        a_item = new Obj*[a_capacity];
        for (int i= 0; i < a_capacity; i++)
            a_item[i]= nullptr;
        a_freeId.clear();
    };

    virtual ~collection()
    {
        for (int i= 0; i < a_capacity; i++)
            if ( a_item[i] != nullptr )
                delete a_item[i];

        delete [] a_item;
    };

    /// Clear :
    ///----------------
    void clear()
    {
        for (int i= 0; i < a_limit; i++)
            if ( a_item[i] != nullptr )
                delete a_item[i];

        a_freeId.clear();
        a_limit= 0;
    }

    /// Master getter :
    ///----------------
    collection<Obj>::iterator begin() {
        return iterator( this, first_index() );
    }
    collection<Obj>::iterator end() {
        return iterator(this, limit() );
    }

    collection<Obj>::const_iterator begin() const {
        return const_iterator( this, first_index() );
    }
    collection<Obj>::const_iterator end() const {
        return const_iterator(this, limit() );
    }

    std::pair< collection<Obj>::iterator, collection<Obj>::iterator > content() {
        return std::pair< collection<Obj>::iterator, collection<Obj>::iterator >( begin(), end() );
    }
    std::pair< collection<Obj>::const_iterator, collection<Obj>::const_iterator > content() const {
        return std::pair< collection<Obj>::const_iterator, collection<Obj>::const_iterator >( begin(), end() );
    }

    /// assignement :
    ///--------------
    virtual const collection<Obj> & operator=(const collection<Obj> & colec)
    {
        // Delete :
        for (int i= 0; i < a_capacity; i++)
            if ( a_item[i] != nullptr )
                delete a_item[i];

        delete [] a_item;

        // Init parameters :
        a_limit= colec.a_limit;
        a_capacity= colec.a_capacity;

        // init structure :
        a_item = new Obj*[a_capacity];
        for (int i= 0; i < a_capacity; i++)
            a_item[i]= nullptr;
        a_freeId.clear();

        // Copie :
        for( int i= colec.first_index(); i < colec.limit(); i= colec.next_index(i) )
        {
            a_item[i]= new Obj;
            (*a_item[i])= colec[i];
        }
        a_freeId= colec.a_freeId;

        return *this;
    }

    /// Initialize :
    ///-------------
    void initialize(int size= 0)
    {
        for (int i= 0; i < a_capacity; i++)
            if ( a_item[i] != nullptr )
                delete a_item[i];

        delete [] a_item;

        a_limit= 0;
        a_freeId.clear();

        a_capacity= std::max(size, 1);
        a_item = new Obj*[a_capacity];

        for (int i= 0; i < a_capacity; ++i)
            a_item[i]= nullptr;
    }

    void initialize( int size, const Obj & obj )
    {
        initialize(size);
        for (int i= 0; i < size; i++)
            push(obj);
    }

    void initialize(int size, Obj* tab)
    {
        initialize(size);
        for (int i= 0; i < size; i++)
            push( tab[i] );
    }

    /// std standard :
    ///---------------
    int push(const Obj &obj= Obj())
    {
        int newid;
        if( a_freeId.empty() )
        {
            newid= a_limit;
            a_limit++;
            if( a_limit >= a_capacity )
                resize( a_capacity*2 );
        }
        else
        {
            newid= a_freeId.front();
            a_freeId.pop_front();
        }

        a_item[newid]= new Obj;
        (*a_item[newid])= obj;

        return newid;
    }

    /// getter and setter :
    ///--------------------
    inline int size()const {
        return a_limit - a_freeId.size();
    }
    inline bool empty()const {
        return ( size() < 1 );
    }
    inline int capacity()const {
        return a_capacity;
    }
    inline int limit()const {
        return a_limit;
    }
    
    int position_to_index( int p )const
    {
        std::list<int>::const_iterator it(a_freeId.begin()), itEnd( a_freeId.end() );
        while( it != itEnd && p > *it ){
            ++p;
            ++it;
        }
        return p;
    }

    inline int first_index()const
    {
        int id= 0;
        while( (id < a_limit) && a_item[id] == nullptr )
            ++id;

        return id;
    }

    std::list<int> list_index()const{
        std::list<int> l;
        for( int i=0; i < limit(); ++i )
            if( is_index(i) )
                l.push_back(i);
        return l;
    }

    inline int next_free_index()const
    {
        if( a_freeId.empty() )
            return a_limit;
        else
            return a_freeId.front();
    }
    
    inline int next_index(int id)const
    {
        ++id;
        while( (id < a_limit) && a_item[id] == nullptr )
            ++id;
        return id;
    }

    

    void set( int i, const Obj &obj= Obj() )
    {
        if( i < a_limit && a_item[i] != nullptr )
            *a_item[i] = obj;
        else if( i < a_limit )
        {
            a_item[i]= new Obj();
            *a_item[i] = obj;
            a_freeId.remove(i);
        }
        else
        {
            if( i < a_capacity )
                resize( pow(2, (int)(sqrt(i))+1) );

            for(a_limit; a_limit < i; ++a_limit)
                a_freeId.push_back(a_limit);

            a_item[i]= new Obj();
            *a_item[i] = obj;

            ++a_limit;
        }
    }

    Obj get(int i)const {
        assert( is_index(i) );
        return (*a_item[i]);
    }

    inline Obj & operator[](int i) {
        assert( is_index(i) );
        return (*a_item[i]);
    }

    inline const Obj & operator[](int i)const {
        assert( is_index(i) );
        return (*a_item[i]);
    }
    
//        inline const Obj & operator()(int i)const{ return (*a_item[i]); }

    void remove(int i)
    {
        assert( is_index(i) );

        delete a_item[i];
        a_item[i]= nullptr;

        if( i == (a_limit - 1) )
        {
            --a_limit;

            while( a_freeId.back() == (a_limit - 1))
            {
                --a_limit;
                a_freeId.pop_back();
            }
            
        }
        else
        {
            a_freeId.push_back(i);
            a_freeId.sort();
            a_freeId.unique();
        }
        
        if( a_capacity > 3 && a_limit <= a_capacity/2 && a_limit - a_freeId.size() < 3*a_capacity/8 )
            resize( a_capacity/2 );
    }

    /// Boolean test :
    ///---------------
    inline bool is_index(int i)const {
        return (i >= 0) && (i < a_limit) && a_item[i] != nullptr;
    }

    /// Size :
    ///-------
    void resize(int size)
    {
        // remove all out elements :
        if( size < a_capacity ){
            for( int i= size; i < limit(); i= next_index(i) )
                remove(i);
        }

        int minCap= std::min(size, a_capacity);
        
        Obj * oldItem[minCap];

        // save elements
        for (int i= 0; i < minCap; i++)
            oldItem[i]= a_item[i];

        // create new structure
        delete [] a_item;
        a_item= new Obj*[size];

        // restore elements
        for (int i= 0; i < minCap; i++)
            a_item[i]= oldItem[i];

        // generate nullptr if size < a_capacity (minCap==a_capacity)
        for (int i= minCap; i < size; i++)
            a_item[i]= nullptr;

        a_capacity= size;
    }
    
    int random_index()const
    {
        return position_to_index( Global::random(0, size()-1) );
    }
    
    // Compression :
    //--------------
    /*        virtual array<int> compress()
            {
                    array<int> newId( limit() );
                    for(int i(0); i < newId.size(); ++i)
                            newId[i]= i;

                    for( std::list<int>::iterator it( a_freeId.begin() ); it != a_freeId.end(); ++it )
                            if( *it < a_nextId )
                    {
                            a_nextId-= 1;
                            a_item[ *it ]= a_item[ a_nextId ];
                            a_item[ a_nextId ]= nullptr;
                            newId[a_nextId]= *it;

                            while( !isId( a_nextId-1 ) )
                                    a_nextId-= 1;
                    }

                    a_freeId.clear();

                    return newId;
            }*/
};

};

#endif // MIA_COLLECTION_H
