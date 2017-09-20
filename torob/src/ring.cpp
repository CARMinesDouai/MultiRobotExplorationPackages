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
#include "ring.h"
#include "impact.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

  // accessor :
  //-----------
float Ring :: angle(int i)const{
  return a_angle0+a_angleStep*i;
}

mia::Float2 Ring :: center(int i)const{
    return mia::Float2::direction( angle(i) ) * a_distance;
}

mia::Float2 Ring :: limit(int i)const{
    return mia::Float2::direction( angle(i)-(a_angleStep*0.5f) ) * a_distance;
}

float Ring :: angle()const{
  return a_angleStep;
}

mia::Float2 Ring :: wayOut(mia::Float2 target)const{
  float refDist2= std::max( target.length2(), a_distance*a_distance ) * 16;
  mia::Float2 intermediate(0.f, 0.f);
  for( int i= 0; i < a_size ; ++i )
    if( a_state[i] == state_free ){
      float dist2= target.distance2( center(i) );
      if( dist2 < refDist2 ){
        intermediate= center(i);
        refDist2= dist2;
      }
    }

  return intermediate;
}

  // setter :
  //---------
void Ring :: resize(int s){
  a_size= s;
  delete [] a_state;
  a_state= new EState[a_size];
  for( int i=0 ; i < a_size ; ++i )
    a_state[i]= state_unknow;
  // compute derived attributes
  a_angleStep= (float)( (2.0*M_PI)/(double)(a_size) );
  a_angle0= -M_PI+( (a_size%2)*0.5f*a_angleStep );
}

  // ROS function:
  //--------------
std::list<mia::Float2> Ring :: scan_subscriber( const sensor_msgs::LaserScan& scan, tf::TransformListener * transform ){
  // refine scan:
  std::list<mia::Float2> dot= scan_subscriber_blanck(scan, transform);

  // detecte obstructed directions:
  for(int i(0); i < a_size ; ++i ){
    mia::Segment2 refSegment( mia::Float2(0.f, 0.f), center(i) );
    mia::Float2 contact;
    a_state[i]= state_free;
    for( std::list<mia::Float2>::iterator it(dot.begin()), itEnd(dot.end()); a_state[i] == state_free && it != itEnd ; ++it ){
      if( mia::impact( refSegment, mia::Circle2(*it, a_radius), contact ) )
        a_state[i]= state_obstruct;
    }
  }

  // turn free to unknown:
  float deltaAngle= a_angleStep*0.5f;
  for(int i(0); i < a_size ; ++i ){
    if( a_state[i] == state_free && ( angle(i)-deltaAngle < scan.angle_min || angle(i)+deltaAngle > scan.angle_max ) )
      a_state[i]= state_unknow;
  }

  return dot;
}

std::list<mia::Float2> Ring :: scan_subscriber_blanck( const sensor_msgs::LaserScan& scan, tf::TransformListener * transform )const{
  std::list<mia::Float2> dot;
  float angle( scan.angle_min );

  // Wait for appropriate Transform :
  bool transform_ok= true;
  tf::StampedTransform scanToRing;

  if ( a_frame_id.compare( scan.header.frame_id ) != 0
    && !transform->waitForTransform( scan.header.frame_id, a_frame_id,
                                     scan.header.stamp, ros::Duration(0.5) )
  ){
    cerr << "Scan transform: " << a_frame_id
         << " -> " << scan.header.frame_id << " unvailable." << endl;
    transform_ok= false;
  }
  else{
    try{
      transform->lookupTransform( a_frame_id, scan.header.frame_id,
                                  scan.header.stamp, scanToRing );
    }
    catch (tf::TransformException ex){
      transform_ok= false;
      ROS_ERROR("%s",ex.what());
    }
  }

  // Get scan
  float min_dist= 0.5*a_radius;

  if( transform_ok ){// with transform :
    for(int i(0), size(scan.ranges.size()); i < size; ++i )
    {
      if( !isnan( scan.ranges[i]) && scan.ranges[i] > min_dist ){
        mia::Float2 v( (mia::Float2::direction( angle ) * scan.ranges[i]) );
        tf::Vector3 rosv( v.x, v.y, 0.f );
        rosv= scanToRing*rosv;
        dot.push_back( mia::Float2(rosv.x(), rosv.y()));
      }
      angle+= scan.angle_increment;
    }
  }
  else{// default :
    for(int i(0), size(scan.ranges.size()); i < size; ++i )
    {
      if( !isnan( scan.ranges[i]) && scan.ranges[i] > min_dist )
        dot.push_back( mia::Float2::direction( angle ) * scan.ranges[i] );
      angle+= scan.angle_increment;
    }
  }

  return dot;
}

std::list<mia::Float2> Ring :: ring_subscriber( const torob_msgs::Ring& ring){
  a_frame_id= ring.header.frame_id;
  a_distance= ring.distance;
  a_radius= ring.radius;
  resize( ring.state.size() );
  for(int i= 0 ; i < a_size ; ++i )
    a_state[i]= (EState)(ring.state[i]);

  std::list<mia::Float2> target;
  for( int i= 0; i < ring.targetX.size() ; ++i )
    target.push_front( mia::Float2( ring.targetX[i], ring.targetY[i] )  );

  return target;
}

void Ring :: publish_ring( ros::Publisher & publisher, const std_msgs::Header& header, const std::list<mia::Float2> &target  ){
  torob_msgs::Ring rmsg;
  rmsg.header= header;
  rmsg.header.frame_id= a_frame_id;
  rmsg.state.resize( a_size );
  rmsg.distance= a_distance;
  rmsg.radius= a_radius;
  for( int i= 0; i < a_size ; ++i)
    rmsg.state[i]= (int)(a_state[i]);

  int t_size= target.size();
  rmsg.targetX.resize( t_size );
  rmsg.targetY.resize( t_size );
  std::list<mia::Float2>::const_iterator it= target.begin();
  for(int i=0 ; i < t_size ; ++i ){
    rmsg.targetX[i]= it->x;
    rmsg.targetY[i]= it->y;
    ++it;
  }



  publisher.publish( rmsg );
}
