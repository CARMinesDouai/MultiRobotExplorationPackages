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

#ifndef MIA_ROSVMAP_H
#define MIA_ROSVMAP_H

#include <string>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <torob_msgs/VectorMap.h>
#include <torob_msgs/Data.h>

#include "visimap.h"

class RosVmap
{
private:
    // Attribute:

public:
    // Attribute:
    mia::VisiMap visimap;
    float a_min_scan_distance, a_max_scan_distance;

    // Constructor Destructor:
    RosVmap();
    ~RosVmap();
   
    // setter:
    void setEpsilon( float e ){ visimap.setEpsilon(e); }
    int add_vertex(const mia::Float2 & position, mia::Node2::EType t );
    
    // getter:
    float getEpsilon()const{ return visimap.getEpsilon(); }
    
    bool free_segment(const mia::Float2 & A, const mia::Float2 & B );
    bool safe_move( const mia::Float2 & pos );
    bool is_vertex_on_segment(const mia::Float2 & A, const mia::Float2 & B, mia::Node2::EType t, mia::Float2 & output );
    mia::Float2 get_closest_vertex(const mia::Float2 & A, mia::Node2::EType t);

    // subscriber:
    void map_subscriber( const nav_msgs::OccupancyGrid& map );
    void data_subscriber( const torob_msgs::Data& vmap );
    void vmap_subscriber( const torob_msgs::VectorMap& vmap );
    std::list<mia::Float2> scan_subscriber( const sensor_msgs::LaserScan& scan, bool closed= false );
    
    // publisher:
    void publish_data( ros::Publisher & publisher, const std_msgs::Header& header );
    void publish_vmap( ros::Publisher & publisher, const std_msgs::Header& header );
    void publish_poses( ros::Publisher & publisher, const std_msgs::Header& header, mia::Node2::EType t= mia::Node2::type_frontier );
    void publish_points( ros::Publisher & publisher, const std_msgs::Header& header, mia::Node2::EType t= mia::Node2::type_frontier );
    void publish_frontier( ros::Publisher & publisher, const std_msgs::Header& h, float radius);
    
    // Map manipulation:
    void merge( const RosVmap & toMerge, const mia::Float2 & translation );
 
    // Margent interface:
    void toTorobData( const mia::Data & d, torob_msgs::Data * td );
    void toMargentData( const torob_msgs::Data & td, mia::Data * d );

    void save_visibility(const std::string & file_path)const{ visimap.save( file_path ); }
    
};

#endif // MIA_ROSVMAP_H
