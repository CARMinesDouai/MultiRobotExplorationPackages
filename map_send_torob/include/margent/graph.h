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

#ifndef MIA_GRAPH_H
#define MIA_GRAPH_H

#include <assert.h>
#include <boost/graph/graph_concepts.hpp>

#include "collection.h"
//#include "array.h"

//#include <boost/iterator/iterator_concepts.hpp>

namespace mia {


template< class VertexType, class EdgeType >
class graph
{
protected:
    collection< VertexType > a_vertex;
    collection< std::list<int> > a_out_edges;
    collection< std::list<int> > a_sources;

    collection< EdgeType > a_edge;
    collection< std::pair<int,int> > a_edge_descriptor;

//  TODO : int a_dimention;
//     collection< array< std::list<int> > > a_out_edges;
//     collection< array< std::list<int> > > a_sources;
// 
//     array< collection< EdgeType > > a_edge;
//     array< collection< std::pair<int,int> > > a_edge_descriptor;

//  TODO:  bool a_undirected -> to put in assert to forbid some functionnality

public:

    // Contructor destructor :
    graph<VertexType, EdgeType>( int vertexCap, int edgeCap ):
        a_vertex(vertexCap),
        a_out_edges(vertexCap),
        a_sources(vertexCap),
        
        a_edge(edgeCap),
        a_edge_descriptor(edgeCap)
    {};

    graph<VertexType, EdgeType>( int vertexCap= 128 ):
        graph<VertexType, EdgeType>(vertexCap, vertexCap*4)
    {};

    virtual ~graph<VertexType, EdgeType>() {};

    // Vertex Getter and Setter :

    virtual const VertexType & vertex(int i)const{
        return a_vertex[i];
    }

    virtual VertexType & vertex(int i){
        return a_vertex[i];
    }
    
    virtual bool is_vertex_index(int i)const{
        return a_vertex.is_index(i);
    }
    
    virtual const std::list<int> & list_source_index( int i )const {
        return a_sources[i];
    }
    
    virtual std::list<int> & list_source_index( int i ) {
        return a_sources[i];
    }

    virtual int vertex_size()const{
        assert( a_vertex.size() == a_sources.size() );
        assert( a_vertex.size() == a_out_edges.size() );
        return a_vertex.size();
    }
    virtual int vertex_first_index()const{
        assert( a_vertex.first_index() == a_sources.first_index() );
        assert( a_vertex.first_index() == a_out_edges.first_index() );
        return a_vertex.first_index();
    }
    virtual int vertex_limit()const{
        assert( a_vertex.limit() == a_sources.limit() );
        assert( a_vertex.limit() == a_out_edges.limit() );
        return a_vertex.limit();
    }
    virtual int vertex_next_index(int i)const{
        assert( a_vertex.next_index(i) == a_sources.next_index(i) );
        assert( a_vertex.next_index(i) == a_out_edges.next_index(i) );
        return a_vertex.next_index(i);
    }
    virtual int random_vertex_index()const{ return a_vertex.random_index(); }
    
    // Edge Getter and Setter :
    
    virtual const EdgeType & edge(int i)const{
        return a_edge[i];
    }

    virtual EdgeType & edge(int i){
        return a_edge[i];
    }

    virtual bool is_edge_index(int i)const{
        return a_edge.is_index(i);
    }

    virtual int edge_size()const{
        assert( a_edge.size() == a_edge_descriptor.size() );
        return a_edge.size();
    }
    virtual int edge_first_index()const{
        assert( a_edge.first_index() == a_edge_descriptor.first_index() );
        return a_edge.first_index();
    }
    virtual int edge_limit()const{
        assert( a_edge.limit() == a_edge_descriptor.limit() );
        return a_edge.limit();
    }
    virtual int edge_next_index(int i)const{
        assert( a_edge.next_index(i) == a_edge_descriptor.next_index(i) );
        return a_edge.next_index(i);
    }
    virtual int random_edge_index()const{ return a_edge.random_index(); }

    virtual int source_index( int ei )const {
        return a_edge_descriptor[ei].first;
    }
    
    virtual int target_index( int ei )const {
        return a_edge_descriptor[ei].second;
    }

    virtual const VertexType & source_vertex( int ei )const {
        return a_vertex[ a_edge_descriptor[ei].first ];
    }

    virtual const VertexType & target_vertex( int ei )const {
        return a_vertex[ a_edge_descriptor[ei].second ];
    }

    virtual VertexType & source_vertex( int ei ) {
        return a_vertex[ a_edge_descriptor[ei].first ];
    }
    virtual VertexType & target_vertex( int ei ) {
        return a_vertex[ a_edge_descriptor[ei].second ];
    }

    virtual int neighbour_index( int vi, int ei )const {
        if( a_edge_descriptor[ei].first == vi )
            return a_edge_descriptor[ei].second;
        assert( a_edge_descriptor[ei].second == vi );
        return a_edge_descriptor[ei].first;
    }

    virtual const VertexType & neighbour( int vi, int ei )const {
        return a_vertex[ neighbour_index(vi, ei) ];
    }

    virtual VertexType & neighbour( int vi, int ei ) {
        return a_vertex[ neighbour_index(vi, ei) ];
    }

    virtual collection<VertexType> & vertex_collection() {
        return a_vertex;
    }

    virtual const collection<VertexType> & vertex_collection() const {
        return a_vertex;
    }

    virtual collection<EdgeType> & edge_collection() {
        return a_edge;
    }
    
    virtual const collection<EdgeType> & edge_collection() const {
        return a_edge;
    }

/*
    virtual std::pair< collection<VertexType>::iterator, collection<VertexType>::iterator > vertex_content() {
        return a_vertex.content();
    }

    virtual std::pair< collection<EdgeType>::iterator, collection<EdgeType>::iterator > edge_content() {
        return a_edge.content();
    }
*/

/*
    virtual std::list<int> list_edge_index()const {
        return a_edge.list_index();
    }
*/

    virtual int count_edges()const {
        return a_edge.size();
    }

    virtual std::list<int> list_edge_index( int src, int trg )const {
        std::list<int> l;
        std::list<int>::const_iterator it= a_out_edges[src].begin();
        std::list<int>::const_iterator itEnd= a_out_edges[src].end();
        for( ; it != itEnd ; ++it )
            if( a_edge_descriptor[*it].second == trg )
                l.push_back(*it);
        return l;
    }

    virtual int count_edges( int src, int trg )const {
        int count(0);
        std::list<int>::const_iterator it= a_out_edges[src].begin();
        std::list<int>::const_iterator itEnd= a_out_edges[src].end();
        for( ; it != itEnd ; ++it )
            if( a_edge_descriptor[*it].second == trg )
                ++count;
        return count;
    }

    virtual int get_edge_index( int src, int trg )const {
        std::list<int>::const_iterator it= a_out_edges[src].begin();
        std::list<int>::const_iterator itEnd= a_out_edges[src].end();
        for( ; it != itEnd ; ++it )
            if( a_edge_descriptor[*it].second == trg )
                return *it;
        return -1;
    }


    virtual const std::list<int> & out_edge_indexes( int src ) const {
        return a_out_edges[src];
    }


//     virtual std::pair< std::list<int>::iterator, std::list<int>::iterator > out_edge_index( int src ) {
//         return std::pair<std::list<int>::iterator, std::list<int>::iterator>( a_vertex_descriptor[src].edges.begin(), a_vertex_descriptor[src].edges.end() );
//     }
// 
//     virtual std::pair< std::list<int>::const_iterator, std::list<int>::const_iterator > out_edge_index( int src ) const {
//         return std::pair<std::list<int>::const_iterator, std::list<int>::const_iterator>( a_vertex_descriptor[src].edges.begin(), a_vertex_descriptor[src].edges.end() );
//     }

    virtual std::list<int> make_in_edge_indexes( int trg )const {
        std::list<int> list_in;
        std::list<int>::const_iterator srctend= a_sources[trg].end();
        for( std::list<int>::const_iterator srct= a_sources[trg].begin(); srct != srctend; ++srct )
        {

            std::list<int>::const_iterator ieend= a_out_edges[*srct].end();
            for( std::list<int>::const_iterator ie= a_out_edges[*srct].begin(); ie != ieend; ++ie )
                if( target_index( *ie ) == trg )
                    list_in.push_back( *ie );
        }
        return list_in;
    }


    // Node manipulation :
//    std::pair< mia::collection<NodeType>::iterator, mia::collection<NodeType>::iterator > nodes(){ return node.content(); }

    virtual int add_vertex() {
        int ni= a_vertex.push();
        assert( ni == a_out_edges.push() );
        assert( ni == a_sources.push() );
        return ni;
    }

    virtual void remove_vertex(int i) {
        a_vertex.remove(i);
        a_sources.remove(i);
        a_out_edges.remove(i);
    }

    virtual void clear_vertex(const int i) {
        {   // Remove in_edge (from sources) :
            std::list<int>::iterator iSource( a_sources[i].begin() ), iSourceEnd( a_sources[i].end() );
            for( ; iSource != iSourceEnd ; ++iSource) {
                std::list<int> listEdge= list_edge_index( *iSource, i );
                std::list<int>::iterator it( listEdge.begin() ), itEnd( listEdge.end() );
                for( ; it != itEnd; ++it )
                {
                    a_edge.remove(*it);
                    a_edge_descriptor.remove(*it);
                    a_out_edges[ *iSource ].remove( *it );
                }
            }
            a_sources[i].clear();
        }

        std::list<int> listTarget;

        {   // Remove out_edge from :
            std::list<int>::iterator iEdge( a_out_edges[i].begin() ),
                iEdgeEnd( a_out_edges[i].end() );
            for( ; iEdge != iEdgeEnd ; ++iEdge) {
                listTarget.push_back( target_index( *iEdge ) );

                a_edge.remove( *iEdge );
                a_edge_descriptor.remove(*iEdge);
            }
            a_out_edges[i].clear();
        }

        {   // Remove i as a source :
            listTarget.sort();
            listTarget.unique();

            std::list<int>::iterator iTrg( listTarget.begin() ), iTrgEnd( listTarget.end() );
            for( ; iTrg != iTrgEnd ; ++iTrg)
                a_sources[*iTrg].remove(i);
        }
    }

    virtual std::pair<int, int> verify_vertex(int i) {
        std::list<int> listTarget1;
        {
            std::list<int>::iterator it, itEnd( a_out_edges[i].end() );
            for( it= a_out_edges[i].begin() ; it != itEnd ; ++it ) {
                assert( a_edge_descriptor[*it].first == i );
                listTarget1.push_back( a_edge_descriptor[*it].second );
            }
        }
        listTarget1.sort();
        listTarget1.unique();

        std::list<int> listTarget2;
        {
            for( int iv= a_vertex.first_index(); iv < a_vertex.limit() ; iv= a_vertex.next_index(iv) ) {
                bool isTarget= false;

                std::list<int>::iterator it, itEnd( a_sources[iv].end() );
                for( it= a_sources[iv].begin(); !isTarget && it != itEnd; ++it ) {
                    isTarget= isTarget || (*it) == i;
                }
                if( isTarget ) listTarget2.push_back(iv);
            }
        }
        listTarget2.sort();
        listTarget2.unique();

        assert( listTarget1 == listTarget2 );

        std::list<int> listSource2;
        for( int iv= a_vertex.first_index(); iv < a_vertex.limit() ; iv= a_vertex.next_index(iv) ) {
            bool isSource= false;

            std::list<int>::iterator it, itEnd( a_out_edges[iv].end() );
            for( it= a_out_edges[iv].begin(); !isSource && it != itEnd; ++it ) {
                if ( a_edge_descriptor[*it].second == i )
                    listSource2.push_back(iv);
            }
        }

        assert( a_sources[i] == listSource2 );

        return std::pair<int, int>( listTarget1.size(), listSource2.size() );
    }

    // Edge manipulation :
//     std::pair< collection<EdgeType>::iterator, collection<EdgeType>::iterator > edges(){ return edge.content(); }

    virtual int add_edge( int src, int trg )
    {
        if( a_vertex.is_index(src), a_vertex.is_index( trg ) ) {
            // Create the new edge
            int ei= a_edge.push();
            assert( ei == a_edge_descriptor.push( std::pair<int,int>(src,trg) ) );

            // Actualize source edges contener
            a_out_edges[src].push_back(ei);

            // Actualize target sources contener
            a_sources[trg].push_back(src);
            a_sources[trg].sort();
            a_sources[trg].unique();

            return ei;
        }
        else {
            return -1;
        }
    }

    virtual void remove_edge( int e )
    {
        int src( source_index(e) ), trg( target_index(e) );

        // Remove from edge collection :
        a_edge.remove(e);
        a_edge_descriptor.remove(e);

        // Remove edge index from source :
        a_out_edges[src].remove(e);

        // Remove source index from target :
        if( get_edge_index(src, trg) < 0 )
            a_sources[trg].remove(src);
    }
};


template< class VertexType, class EdgeType >
class undirected_graph
{
protected:
    collection< VertexType > a_vertex;
    collection< std::list<int> > a_out_edges;

    collection< EdgeType > a_edge;
    collection< std::pair<int,int> > a_edge_descriptor;
 
    static int neg_twin( int ei ){ return -(ei+1); }
    static int pos_twin( int ei ){ return (-ei)-1; }

    static int _mt( int ei){ // master twin
        if( ei < 0 )
            return pos_twin(ei);
        return ei;
    }

public:

    // Contructor destructor :
    undirected_graph<VertexType, EdgeType>( int vertexCap, int edgeCap ):
        a_vertex(vertexCap),
        a_out_edges(vertexCap),
  
        a_edge(edgeCap),
        a_edge_descriptor(edgeCap)
    {};

    undirected_graph<VertexType, EdgeType>( int vertexCap= 128 ):
        undirected_graph<VertexType, EdgeType>(vertexCap, vertexCap*4)
    {};

    virtual ~undirected_graph<VertexType, EdgeType>() {};

    virtual void clear()
    {
        a_vertex.clear();
        a_out_edges.clear();

        a_edge.clear();
        a_edge_descriptor.clear();
    }
    
    static int twin( int ei){ 
        if( ei < 0 )
            return pos_twin(ei);
        return neg_twin(ei);
    }

    // Vertex Getter and Setter :

    virtual const VertexType & vertex(int i)const{
        return a_vertex[i];
    }

    virtual VertexType & vertex(int i){
        return a_vertex[i];
    }

    virtual bool is_vertex_index(int i)const{
        return a_vertex.is_index(i);
    }

    virtual int vertex_size()const{
        assert( a_vertex.size() == a_out_edges.size() );
        return a_vertex.size();
    }
    virtual int vertex_first_index()const{
        assert( a_vertex.first_index() == a_out_edges.first_index() );
        return a_vertex.first_index();
    }
    virtual const VertexType &  first_vertex()const{
        assert( a_vertex.first_index() == a_out_edges.first_index() );
        return a_vertex[ a_vertex.first_index() ];
    }
    virtual VertexType &  first_vertex(){
        assert( a_vertex.first_index() == a_out_edges.first_index() );
        return a_vertex[ a_vertex.first_index() ];
    }
    virtual int vertex_limit()const{
        assert( a_vertex.limit() == a_out_edges.limit() );
        return a_vertex.limit();
    }
    virtual int vertex_next_index(int i)const{
        assert( a_vertex.next_index(i) == a_out_edges.next_index(i) );
        return a_vertex.next_index(i);
    }
    virtual int vertex_second_index()const{
        return vertex_next_index( vertex_first_index() );
    }
    virtual int random_vertex_index()const{ return a_vertex.random_index(); }
    
    // Edge Getter and Setter :
    
    virtual const EdgeType & edge(int ei)const{
        return a_edge[_mt(ei)];
    }
    virtual EdgeType & edge(int ei){
        return a_edge[_mt(ei)];
    }

    virtual bool is_edge_index(int i)const{
        return a_edge.is_index(i);
    }

    virtual int edge_size()const{
        assert( a_edge.size() == a_edge_descriptor.size() );
        return a_edge.size();
    }

    virtual int edge_first_index()const{
        assert( a_edge.first_index() == a_edge_descriptor.first_index() );
        return a_edge.first_index();
    }
    virtual int edge_limit()const{
        assert( a_edge.limit() == a_edge_descriptor.limit() );
        return a_edge.limit();
    }
    virtual int edge_next_index(int i)const{
        assert( a_edge.next_index(i) == a_edge_descriptor.next_index(i) );
        return a_edge.next_index(i);
    }
    virtual int edge_second_index()const{
        return edge_next_index( edge_first_index() );
    }
    virtual int random_edge_index()const{ return a_edge.random_index(); }

    virtual int source_index( int ei )const {
        if( ei < 0 )
            return a_edge_descriptor[pos_twin(ei)].second;
        return a_edge_descriptor[ei].first;
    }
    
    virtual int target_index( int ei )const {
        if( ei < 0 )
            return a_edge_descriptor[pos_twin(ei)].first;
        return a_edge_descriptor[ei].second;
    }

    virtual const VertexType & source_vertex( int ei )const {
        return a_vertex[ source_index(ei) ];
    }

    virtual const VertexType & target_vertex( int ei )const {
        return a_vertex[ target_index(ei) ];
    }

    virtual VertexType & source_vertex( int ei ) {
        return a_vertex[ source_index(ei) ];
    }
    virtual VertexType & target_vertex( int ei ) {
        return a_vertex[ target_index(ei) ];
    }

    virtual collection<VertexType> & vertex_collection() {
        return a_vertex;
    }

    virtual const collection<VertexType> & vertex_collection() const {
        return a_vertex;
    }

    virtual collection<EdgeType> & edge_collection() {
        return a_edge;
    }
    
    virtual const collection<EdgeType> & edge_collection() const {
        return a_edge;
    }

/*
    virtual std::pair< collection<VertexType>::iterator, collection<VertexType>::iterator > vertex_content() {
        return a_vertex.content();
    }

    virtual std::pair< collection<EdgeType>::iterator, collection<EdgeType>::iterator > edge_content() {
        return a_edge.content();
    }
*/

    virtual std::list<int> list_edge_index()const {
        return a_edge.list_index();
    }

    virtual int count_edges()const {
        return a_edge.size();
    }

    virtual std::list<int> list_edge_index( int src, int trg )const {
        std::list<int> l;
        std::list<int>::const_iterator it, itEnd= a_out_edges[src].end();
        for( it= a_out_edges[src].begin() ; it != itEnd ; ++it )
            if( target_index(*it) == trg )
                l.push_back(*it);
        return l;
    }

    virtual int count_edges( int src, int trg )const {
        int count(0);
        std::list<int>::const_iterator it, itEnd= a_out_edges[src].end();
        for( it= a_out_edges[src].begin() ; it != itEnd ; ++it )
            if( target_index(*it) == trg )
                ++count;
        return count;
    }

    virtual int get_edge_index( int src, int trg )const {
        std::list<int>::const_iterator it, itEnd= a_out_edges[src].end();
        for( it= a_out_edges[src].begin() ; it != itEnd ; ++it )
            if( target_index(*it) == trg )
                return *it;
        return -1;
    }


    virtual const std::list<int> & out_edge_indexes( int src ) const {
        return a_out_edges[src];
    }

    // Node manipulation :
//    std::pair< mia::collection<NodeType>::iterator, mia::collection<NodeType>::iterator > nodes(){ return node.content(); }

    virtual int add_vertex() {
        int ni= a_vertex.push();
        assert( ni == a_out_edges.push() );
        return ni;
    }

    virtual void remove_vertex(int i) {
        a_vertex.remove(i);
        a_out_edges.remove(i);
    }

    virtual void clear_vertex(const int i)
    {
        std::list<int> edge= out_edge_indexes(i);
        std::list<int>::iterator it, itEnd( edge.end() );
        for(it= edge.begin() ; it != itEnd; ++it )
        {
            remove_edge(*it);
        }
    }

    virtual int add_edge( int src, int trg )
    {
        assert( a_vertex.is_index(src) && a_vertex.is_index( trg ) );
        
        // Create the new edge
        int ei= a_edge.push();
        assert( ei == a_edge_descriptor.push( std::pair<int,int>(src,trg) ) );

        // Actualize source edges contener
        a_out_edges[src].push_back( ei );
        if( src != trg )
            a_out_edges[trg].push_back( neg_twin(ei) );

        return ei;

    }

    virtual void remove_edge( int e )
    {
        int src( source_index(e) ), trg( target_index(e) );

//         std::cout << "\nremove edge :" << e << "(" << src << ", " << trg << ")" << std::endl;
        
        // Remove from edge collection :
        a_edge.remove( _mt(e) );
        a_edge_descriptor.remove( _mt(e) );

        // Remove out edge indexes :
        a_out_edges[src].remove( e );
        if( src != trg  )
            a_out_edges[trg].remove( twin(e) );
    }
};

};

#endif // MIA_COLLECTION_H
