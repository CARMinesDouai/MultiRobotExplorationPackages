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

// Boost :
// #include "tools.h"
//
// #include <boost/geometry.hpp>
// #include <boost/geometry/geometries/register/point.hpp>

#ifndef RING_H
#define RING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <torob_msgs/Ring.h>

#include "float2.h"

class Ring {
public:
  enum EState{
    state_free= 0,
    state_obstruct,
    state_unknow,
  };

protected:
  //attributs:
  float a_distance, a_radius;
  int a_size;
  EState * a_state;
  std::string a_frame_id;

  //derived attributes:
  float a_angleStep, a_angle0;

public:

  // constructor :
  //--------------
  Ring(float distance= 1.f, float radius= 0.2f, int size= 0, const char * frame_id= "base_link"):
      a_distance(distance),
      a_radius(radius),
      a_size(size),
      a_frame_id(frame_id){
    a_state= new EState[a_size];
    for( int i=0 ; i < a_size ; ++i )
      a_state[i]= state_unknow;
    // compute derived attributes
    a_angleStep= (float)( (2.0*M_PI)/(double)(a_size) );
    a_angle0= -M_PI+( (a_size%2)*0.5f*a_angleStep );
  }
  virtual ~Ring(){ delete [] a_state; }

  // accessor :
  //-----------
  virtual float distance()const{ return a_distance; }
  virtual float radius()const{ return a_radius; }
  virtual int size()const{ return a_size; }
  virtual EState operator[](int i)const{ return a_state[i]; }
  virtual EState & operator[](int i){ return a_state[i]; }

  virtual bool isFree(int i)const{ a_state[i] == state_free; }

  virtual float angle(int i)const;
  virtual mia::Float2 center(int i)const;
  virtual mia::Float2 limit(int i)const;
  virtual float angle()const;

  virtual mia::Float2 wayOut(mia::Float2 target)const;
  // setter :
  //---------
  virtual void resize(int s);

  // ROS function:
  //--------------
  virtual std::list<mia::Float2> scan_subscriber( const sensor_msgs::LaserScan& scan, tf::TransformListener * transform );
  virtual std::list<mia::Float2> scan_subscriber_blanck( const sensor_msgs::LaserScan& scan, tf::TransformListener * transform )const;
  virtual std::list<mia::Float2> ring_subscriber( const torob_msgs::Ring& scan);
  virtual void publish_ring( ros::Publisher & publisher, const std_msgs::Header& header, const std::list<mia::Float2> &target );
};


#endif // RING_H
