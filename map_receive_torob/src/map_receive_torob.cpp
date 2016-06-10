/*
 * Copyright (c) 2015, Khelifa Baizid <Khelifa.Baizid@mines-douai.fr and Zhi Yan zhi.yan@mines-douai.fr>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>
#include "visimap.h"
#include "ogmap.h"

using namespace :: mia;
using namespace std;

//some global variables
// Maps:
VisiMap visimap;              //guillaume torob map
std::string trb_map_frame_;   // map resolution

#define MAX_BUFFER_SIZE 20240 //10240
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))// compute linear index for given map coords
//we define a structure handling the robot
struct robot_in{
    string ip;
    short  id;
    string out_map_topic;
    string out_pose_topic;
    geometry_msgs::Pose2D inital_pose;
    geometry_msgs::Pose2D current_pose;
};

//ros::Subscriber map_subscriber;     //creating map subscribers
vector<ros::Publisher>  map_metadata_subs;
vector<ros::Publisher>  grap_to_map_publisher;
vector<ros::Publisher>  robot_model_publisher;

vector<nav_msgs::OccupancyGrid> map_; //a vector handling the incoming maps;
//std::string map_topic_;
vector<robot_in> robots_in_map_ips;   //a vector handling robot to be connect to;
vector<ros::Publisher> map_publisher; //ros::Publisher map_publisher;
vector<ros::Publisher> pose_publisher;//ros::Publisher pose_publisher of the other robots
int xrpose_, yrpose_, trpose_;        // initia position of the robot
int xr_currt_pose_, yr_currt_pose_;   // initia position of the robot

float resolution_;                    // of the map
bool  map_recieved_flag;              // is a map is received
bool  map_is_compressed_;             // is a map is received
bool  use_torob_map_;                 // in case we are using torob map
int   width_, height_;                // map size
//about socket
int socket_desc;
struct sockaddr_in server;
bool conect_to_server;
bool socket_created;

int trb_offset_x, trb_offset_y;                  // map resolution
int trb_map_width_pixel, trb_map_height_pixel;   // map resolution
double trb_resolution_;                          // map resolution
int map_receive_inc;
//we initialize a map
void init_map(){
    for(int i=0;i<2;i++)
    {
        map_[i].info.resolution           = resolution_;
        map_[i].info.origin.position.x    =  0.0;
        map_[i].info.origin.position.y    =  0.0;
        map_[i].info.origin.position.z    =  0.0;
        map_[i].info.origin.orientation.x =  0.0;
        map_[i].info.origin.orientation.y =  0.0;
        map_[i].info.origin.orientation.z =  0.0;
        map_[i].info.origin.orientation.w =  1.0;
        map_[i].info.width                =  width_;
        map_[i].info.height               =  height_;
        map_[i].data.resize(map_[i].info.width*map_[i].info.height);
    }
}
//uncopress a vector or int
vector<int> simpleUncompressionVector(vector<int> in)
{
    vector<int> out;
    //if the vector is emty we return it
    if(!in.size()) return in;

    int s=in.size();
    int v=0;
    int inc_i=0;
    for(inc_i=0;inc_i<s;inc_i+=2)
    {
       v=in[inc_i];
       //now we check how much elements are similar before changing and we stor them
       int val_id=0;
       if(inc_i+1<s)
       while(val_id<in[inc_i+1]){
           out.push_back(v);
           val_id++;
       }
    }
    return out;
}

/*void clearMap()
{
    for (int y=0; y<map_.info.height; y++)
        for (int x=0; x< map_.info.width; x++)
        {
            map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
        }
}*/

void putpixel(int x, int y, int c, short id)
{
    if( 0 <= x && x < map_[id].info.width && 0 <= y && y < map_[id].info.height )
        map_[id].data[MAP_IDX(map_[id].info.width, x, y)] = 100;
}
void bhm_line(int x1,int y1,int x2,int y2,int c, short id)
{
 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
 dx=x2-x1;
 dy=y2-y1;
 dx1=fabs(dx);
 dy1=fabs(dy);
 px=2*dy1-dx1;
 py=2*dx1-dy1;
 if(dy1<=dx1)
 {
    if(dx>=0){
        x=x1;
        y=y1;
        xe=x2;
    }else{
        x=x2;
        y=y2;
        xe=x1;
    }
    putpixel(x,y,c, id);
    for(i=0;x<xe;i++)
    {
        x=x+1;
        if(px<0){
            px=px+2*dy1;
        }else{
        if((dx<0 && dy<0) || (dx>0 && dy>0)){
            y=y+1;
        }else{
            y=y-1;
        }
        px=px+2*(dy1-dx1);
        }
        //delay(0);
        putpixel(x,y,c, id);
    }
 }else{
    if(dy>=0){
        x=x1;
        y=y1;
        ye=y2;
    }else{
        x=x2;
        y=y2;
        ye=y1;
    }
    putpixel(x,y,c, id);
    for(i=0;y<ye;i++)
    {
        y=y+1;
        if(py<=0){
            py=py+2*dx1;
        }else{
        if((dx<0 && dy<0) || (dx>0 && dy>0))
        {
            x=x+1;
        }else{
            x=x-1;
        }
        py=py+2*(dx1-dy1);
        }
        //delay(0);
        putpixel(x,y,c, id);
    }
 }
}

void publishMapAsGraph(const mia::VisiMap & visimap, float robot_x, float robot_y, short id)//&(_moinag.torob->visibility()->a_visibility);
{
  //cout<<"publish graph: " << endl;
  //std::vector<float> torob_graph;
  //read the graph

  int nbVertex= boost::num_vertices(visimap.a_map);

  visualization_msgs::MarkerArray marray_g;

  //graph nodes
  visualization_msgs::Marker map_nd;
  map_nd.header.frame_id= "map";
  map_nd.header.stamp   = ros::Time::now();
  map_nd.id             = 0;
  map_nd.ns             = "totob";
  map_nd.type           = visualization_msgs::Marker::SPHERE;
  map_nd.pose.position.x= 0.0;
  map_nd.pose.position.y= 0.0;
  map_nd.pose.position.z= 0.0;
  map_nd.scale.x        = 0.1;
  map_nd.scale.y        = 0.1;
  map_nd.scale.z        = 0.1;
  map_nd.color.r        = 1.0;
  map_nd.color.g        = 0;
  map_nd.color.b        = 0.0;
  map_nd.color.a        = 1.0;
  map_nd.lifetime       = ros::Duration(0);
  map_nd.action         = visualization_msgs::Marker::ADD;

  int iv= 0;
  mia::Graph2::vertex_iterator vt, vtEnd;
  for( boost::tie(vt, vtEnd)= boost::vertices(visimap.a_map) ; vt != vtEnd ; ++vt )
    if( visimap.a_map[*vt].type == mia::Node2::type_obstacle )
    {
        map_nd.id= iv;
        ++iv;

        map_nd.pose.position.x = visimap.a_map[*vt].x;
        map_nd.pose.position.y = visimap.a_map[*vt].y;

        marray_g.markers.push_back( visualization_msgs::Marker(map_nd) );
    }

  //graph edges
  visualization_msgs::Marker map_edge;
  map_edge.header.frame_id = "map";
  map_edge.header.stamp    = ros::Time::now();
  map_edge.action  = visualization_msgs::Marker::ADD;
  map_edge.ns      = "totob";
  map_edge.id      = 0;
  map_edge.type    = visualization_msgs::Marker::LINE_STRIP;
  map_edge.scale.x = 0.1;
  map_edge.scale.y = 0.1;
  map_edge.scale.z = 0.1;
  map_edge.color.r = 0.0;
  map_edge.color.g = 0.0;
  map_edge.color.b = 1.0;
  map_edge.color.a = 1.0;

  //clearMap();


  geometry_msgs::Point min, max;
  {
    boost::tie(vt, vtEnd)= boost::vertices(visimap.a_map);

    min.x= visimap.a_map[*vt].x;
    min.y= visimap.a_map[*vt].y;
    max.x= visimap.a_map[*vt].x;
    max.y= visimap.a_map[*vt].y;

    for( ++vt; vt != vtEnd; ++vt)
    {
        if( visimap.a_map[*vt].x < min.x )
            min.x= visimap.a_map[*vt].x;
        if( visimap.a_map[*vt].y < min.y )
            min.y= visimap.a_map[*vt].y;

        if( visimap.a_map[*vt].x > max.x )
            max.x= visimap.a_map[*vt].x;
        if( visimap.a_map[*vt].y > max.y )
            max.y= visimap.a_map[*vt].y;
    }

    trb_map_width_pixel= 2+(int)((max.x-min.x)/trb_resolution_);
    trb_map_height_pixel= 2+(int)((max.y-min.y)/trb_resolution_);
  }

  int ie = nbVertex;

  mia::Graph2::edge_iterator et, etEnd;
  int nbEdges(0);

  for( boost::tie(et, etEnd)= boost::edges( visimap.a_map ) ; et != etEnd ; ++et )
    {
        int source= boost::source( *et, visimap.a_map );
        int target= boost::target( *et, visimap.a_map );

        if( visimap.a_map[source].type == mia::Node2::type_obstacle &&  visimap.a_map[target].type == mia::Node2::type_obstacle )
        {
            ++nbEdges;

            map_edge.id = ie;
            ++ie;

            map_edge.points.clear();
            geometry_msgs::Point p;

            p.x = visimap.a_map[source].x;
            p.y = visimap.a_map[source].y;
            p.z = 0.0;
            map_edge.points.push_back(p);

            //====================================>>>>>>>>>>=========================MAP RELATED CODE
            //first point of the line
            //p.x = rand() % (int)trb_map_width_metre;
            //p.y = rand() % (int)trb_map_height_metre;
            int x1( 1+(int)((p.x-min.x)/trb_resolution_) ), y1( 1+(int)((p.y-min.y)/trb_resolution_) );
            //cout<<" x1 "<<p.x<<endl;
            //cout<<" y1 "<<p.y<<endl;
            //====================================<<<<<<<<<<=========================MAP RELATED CODE
            p.x = visimap.a_map[target].x;
            p.y = visimap.a_map[target].y;
            map_edge.points.push_back(p);

            //====================================>>>>>>>>>>=========================MAP RELATED CODE
            //p.x = rand() % (int)trb_map_width_metre;
            //p.y = rand() % (int)trb_map_height_metre;
            //second point of the line
            int x2( 1+(int)((p.x-min.x)/trb_resolution_) ), y2( 1+(int)((p.y-min.y)/trb_resolution_) );

            //cout<<trb_map_width_pixel<<" trb_map_width_metre  x1 "<<p.x<<endl;
            //cout<<" y2 "<<x1<<endl;
            //cout<<" x2 "<<x2<<endl;
            //cout<<trb_map_height_pixel<<"trb_map_height_metre y2 "<<y2<<endl;
            //Shifting to the midle of the map //bhm_line(int(x1), int(y1), int(x2), int(y2), 1);

            int int_x1(x1);//+trb_map_width_pixel/2);
            int int_y1(y1);//+trb_map_height_pixel/2);
            int int_x2(x2);//+trb_map_width_pixel/2);
            int int_y2(y2);//+trb_map_height_pixel/2);

            if( int_x1 < 0 || int_y1 < 0 || int_x2 < 0 || int_y2 < 0 )
            {
                //cerr << " ERROR edge [" << source << ", " << target << "] n°" << ie;
                //cerr << " Segment (" << x1 << ", " << y1 << ") -> (" << x2 << ", " << y2 << ")";
                //cerr << " Pixel (" << int_x1 << ", " << int_y1 << ") -> (" << int_x2 << ", " << int_y2 << ")" << endl;
            }
            else{ bhm_line( int_x1, int_y1, int_x2, int_y2, 1, id);//ex. fixed line: bhm_line( 0, 0, 300, 700, 1);
                //cout<<" ... drawing ... "<<x2<<endl;
            }

            //========================================================================MAP RELATED CODE
            marray_g.markers.push_back( visualization_msgs::Marker(map_edge) );
        }
    }

    //====================================>>>>>>>>>>=========================MAP RELATED CODE
    // Set the header information on the map
    map_[id].header.stamp = ros::Time::now();
    map_[id].header.frame_id = trb_map_frame_;

    //_map_publisher.publish(map_[id]);
    //_map_publisher_m.publish(map_[id].info);

    //cout<<trb_map_width_pixel<<" :mmmmmm: "<<trb_map_height_pixel<<endl;cout<<trb_offset_x<<" :mmmmmmmmmm: "<<trb_offset_y<<endl;
    //====================================<<<<<<<<<<=========================MAP RELATED CODE
    //robot model as a cube
    visualization_msgs::Marker robot_md;
    robot_md.header.frame_id = "odom";
    robot_md.header.stamp = ros::Time::now();
    robot_md.id = 0;
    robot_md.ns = "totob";
    robot_md.type = visualization_msgs::Marker::CUBE;
    robot_md.pose.position.x = robot_x;
    robot_md.pose.position.y = robot_y;
    robot_md.pose.position.z = 0.0;
    robot_md.scale.x = 0.9;robot_md.scale.y = 0.9;robot_md.scale.z = 0.9;
    robot_md.color.r = 0.0;  robot_md.color.g = 1.0;robot_md.color.b = 0.0;robot_md.color.a = 1.0;
    robot_md.lifetime = ros::Duration(0);


    map_nd.action = visualization_msgs::Marker::DELETE;
    /*for (; id < marker_count_; id++)
    {
    map_nd.id = id;
    marray_g.markers.push_back(visualization_msgs::Marker(map_nd));
    }*/
    //marker_count_ = marray_g.markers.size();
    //we publish the graph

    grap_to_map_publisher[id].publish(marray_g);
    //we publish the robot model as a cube
    robot_model_publisher[id].publish(robot_md);


   cerr<<"\t-publishing graph size: "<<marray_g.markers.size()<<endl;
//   cout<<"\npublish graph end." << endl;
}

//convert a vector to map
void convertToMap(vector<int> in, short id)
{
    if(!use_torob_map_)
    {
        int s = map_[id].data.size();
        int sm=in.size();
        int v=0;
        for(int i=0;i<s;i++){
            v=in[i];
            if      (v==0)   v= 100;
            else if (v==204) v=-1;
            else if (v==255) v= 0;
            //if(i<sm)
            map_[id].data[i] = v;
            //cout<<s<<" <=s ============== : "<<i<<" "<<in[i]<<" "<<sm<<endl;
        }
        //cout<<s<<" <=s ====dddddddddd========== "<<" "<<id<<endl;
    }else //converting to torob map
    {
        //convert to int
        vector<int> inin;
        for (int i=0; i < in.size(); i+=2)
        {
            inin.push_back(in[i]*255 + in[i+1]);
        }
        in.resize(inin.size());
        for (int i=0; i < inin.size(); i++){
            in[i] = inin[i];
            //if( i<20)cout<<i<<" i "<<in[i]<<endl;
        }
        int fs = in[0];
        int vs = in[1];

        Data d_in("map", fs, vs);

        for (int i=0; i < fs; i++)
        {
            d_in.flag(i, in[2+i]); //all edges +  metadata
            //cout<<" <=s in[2+i] "<<" "<<in[2+i]<<endl;
        }
        for (int i = 0; i < vs; i++)
        {
            d_in.value(i, (float)(in[2+fs+i])/100.f); //all position of all nodes and map size origine etc.
            //cout<<" <=s in[2+i] "<<" "<<(float)(in[2+fs+i])/100.f<<endl;
        }
        //
        //cerr << "visimap.load2(d_in);" << d_in << endl;
        visimap.load2(d_in);
        //visimap.save("reveived_visimap.vm");//
        cout<<" <=s ====publishMapAsGraph========== "<<" "<<id<<endl;
        publishMapAsGraph(visimap, 0.f, 0.f, id);//0.f is the robot position
    }
}
//recieive a socket
int receive_map(int socket, short id)
{ // Start function 
    //cout<<"from receive_map 0"<<endl;

    int buffersize = 0, recv_size = 0,size = 0, read_size, packet_index =1,stat, w, h, px, py, pxc, pyc;
    //int write_size;
    //Find the size of the map
    do{
    stat = read(socket, &size, sizeof(int));
    //cout<<"do read socket size "<<size<<endl;
    //sleep(1);
    }while(stat<0);

    if(size<=0){
        cout<<"bad size is ============== : "<<size<<endl;
        return 0;
    }

    //Find the width of the map
    do{
    w = read(socket, &width_, sizeof(int));
    //cout<<"do width_ socket "<<w<<" "<<width_<<endl;
    //sleep(1);
    }while(w<0);

    //Find the height of the map
    do{
    h = read(socket, &height_, sizeof(int));
    //cout<<"do height_ socket "<<h<<" "<<height_<<endl;
    //sleep(1);
    }while(h<0);

    //Find the inital pose of the robot x
    do{px = read(socket, &xrpose_, sizeof(int));//cout<<"the robot is in xrpose_ "<<xrpose_<<endl;
    }while(px<0);

    //Find the inital pose of the robot y
    do{py = read(socket, &yrpose_, sizeof(int));//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;
    }while(py<0);

    //Find the current pose of the robot x
    do{pxc = read(socket, &xr_currt_pose_, sizeof(int));//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;
    }while(pxc<0);

    //Find the current pose of the robot y
    do{pyc = read(socket, &yr_currt_pose_, sizeof(int));//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;
    }while(pyc<0);

    robots_in_map_ips[id].inital_pose.x      = (double) ((double) xrpose_/(double) 100);
    robots_in_map_ips[id].inital_pose.y      = (double) ((double) yrpose_/(double) 100);
    robots_in_map_ips[id].inital_pose.theta  = (double) trpose_;//not used till now

    robots_in_map_ips[id].current_pose.x     = (double) ((double) xr_currt_pose_/(double) 100);
    robots_in_map_ips[id].current_pose.y     = (double) ((double) yr_currt_pose_/(double) 100);
    robots_in_map_ips[id].current_pose.theta = (double) trpose_;//not used till now

    cout<<"the robot is in rpose_ "<<robots_in_map_ips[id].inital_pose.x<<" "<<robots_in_map_ips[id].inital_pose.y<<endl;
    cout<<"the robot is in rpose_ "<<width_<<" "<<height_<<endl;

    map_[id].info.width                      =  width_;
    map_[id].info.height                     =  height_;

    map_[id].info.origin.position.x          = 0;//-width_*resolution_/2;
    map_[id].info.origin.position.y          = 0;//-height_*resolution_/2;

    map_[id].data.resize(width_*height_);

    buffersize = MAX_BUFFER_SIZE;
    if(size<MAX_BUFFER_SIZE){
        buffersize = size;
    }

    char map_recieved[buffersize],verify = '1';//50176
    cout<<"from receive_map 11.0 : buffersize is"<<buffersize<<endl;

    char map_recieved_torob[buffersize];

    char buffer []= "got it";
    //printf("Packet received.\n");
    //printf("Packet size: %i\n",stat);
    //printf("Image size: %i\n",size);printf(" \n");

    //Send our verificationsize signal
    do{
    stat = write(socket, &buffer, sizeof(int));
    }while(stat<0);

    //printf("Reply sent\n");printf(" \n");

    //cout<<"from receive_map 2"<<endl;

    //========Loop while we have not received the entire map yet

    struct timeval timeout = {10,0};

    fd_set fds;
    int buffer_fd;//, buffer_out;
    int fromi;
    //int whole_map_size;
    fromi=0;
    vector<int> val;
    val.resize(size);
    if(!use_torob_map_)
    {
        while(recv_size < size) {//while(packet_index < 2){

            cerr << "Error while opening the file 0" <<endl;
            FD_ZERO(&fds);
            FD_SET(socket,&fds);
            //cout<<"from receive_map 11.1 : stat is"<<buffersize<<endl;
            buffer_fd = select(FD_SETSIZE,&fds,NULL,NULL,&timeout);
            cout<<"from receive_map 2.2 buffer_fd is: "<<buffer_fd<<endl;
            cerr << "Error while opening the file 1" <<endl;

            if (buffer_fd < 0){
                printf("error: bad file descriptor set.\n");
                cerr << "Error while opening the file 2: " <<buffer_fd<<endl;//sleep(1);
                return 0;
                break;
            }
            if (buffer_fd == 0){
                printf("error: buffer read timeout expired.\n");
                cerr << "Error while opening the file 3: " <<buffer_fd<<endl;//sleep(1);
                return 0;
                break;
            }
            if (buffer_fd > 0)
            {
                //cout<<"from receive_map 2.3"<<endl;
                cerr << "Error while opening the file 4: " <<buffer_fd<<endl;
                do{
                    read_size = read(socket,map_recieved, buffersize);
                    cerr << "read(socket,map_recieved, buffersize); >>>>>>>>>>>>>>>>>" << buffersize << endl;
                    cerr << "Error while opening the file  6 i+fromi "<<endl;
                }while(read_size <0);
                //sleep(5);
                cerr << "Error while opening the file 5" <<endl;
                unsigned int int_val=0;
                cerr << "Error while opening the file 6: " <<fromi<<endl;
                for(int i=0;i<read_size;i++){
                    //cout<<read_size<<" <==read_size : from receive_map 2.5 "<<i<<" :i+fromi==> "<<i+fromi<<" : "<<map_.data.size()<<endl;
                    //cerr <<i<< " Error while opening the file  6 i+fromi "<< i+fromi<<" "<<map_[id].data.size()<<" "<<size<<endl;
                    unsigned char c = (unsigned char) map_recieved[i];
                    cerr << "Error while opening the file 6 i: " <<i<<endl;
                    if(i+fromi < size){
                        int_val      = (unsigned int)c;
                        val[i+fromi] = int_val;
                    }
                    //if((int)(c)==255)
                }

                fromi+=read_size;

                //printf("Packet number received: %i\n",packet_index);
                //printf("Packet size: %i\n",read_size);
                //cout<<"from receive_map 3"<<endl;

                //if(read_size !=write_size) {printf("error in read write\n");}
                //cout<<read_size<<"... from receive_map 2.5 ..."<<write_size<<endl;
                //Increment the total number of bytes read
                recv_size += read_size;
                packet_index++;
                //printf("Total received map size: %i\n",recv_size);printf(" \n");
                //cerr << "Error while opening the file " << packet_index<<" read_size "<<read_size<<endl;
                map_recieved_flag=true;
                if(read_size<=0)
                    return 0;
                    //break;
                //sleep(1);
                }
        }
     }else//using torob map
     {
        while(recv_size < size) {//while(packet_index < 2){

            //cerr << "Error while opening the file 0t" <<endl;
            FD_ZERO(&fds);
            FD_SET(socket,&fds);
            //cout<<"from receive_map 11.1 : stat is"<<buffersize<<endl;
            buffer_fd = select(FD_SETSIZE,&fds,NULL,NULL,&timeout);
            //cout<<"from receive_map 2.2 buffer_fd is: "<<buffer_fd<<endl;
            //cerr << "Error while opening the file 1" <<endl;

            if (buffer_fd < 0){
                printf("error: bad file descriptor set.\n");
                cerr << "Error while opening the file 2: " <<buffer_fd<<endl;//sleep(1);
                return 0;
                break;
            }
            if (buffer_fd == 0){
                printf("error: buffer read timeout expired.\n");
                cerr << "Error while opening the file 3: " <<buffer_fd<<endl;//sleep(1);
                return 0;
                break;
            }
            if (buffer_fd > 0)
            {
                //cout<<"from receive_map 2.3t"<<endl;
                //cerr << "Error while opening the file 4: " <<buffer_fd<<endl;
                do{
                    read_size = read(socket,map_recieved_torob, buffersize);
                    //cerr << "read(socket,map_recieved,  buffersize*sizeof(int)); >>>>>>>>>>>>>>>>>" <<  buffersize*sizeof(int) << endl;
                    //cerr << "Error while opening the file  6 i+fromi "<<endl;
                }while(read_size <0);
                //sleep(5);
                //cerr << "Error while opening the file 5t" <<endl;
                int int_val=0;
                //cerr << "Error while opening the file 6: " <<fromi<<endl;
                for(int i=0;i<read_size;i++){
                    //cout<<read_size<<endl;//" <==read_size : from receive_map 2.5 "<<i<<" :i+fromi==> "<<i+fromi<<" : "<<map_.data.size()<<endl;
                    //cerr <<i<< " Error while opening the file  6 i+fromi "<< i+fromi<<" "<<map_[id].data.size()<<" "<<size<<endl;
                    unsigned char c = (unsigned char) map_recieved_torob[i];
                    //cerr <<i+fromi<<" : "<<read_size<< " Error while opening the file 6t i: " <<i<<" "<<buffersize<<endl;
                    if(i+fromi < size){
                        int_val = (int)c;
                        //cerr <<i+fromi<<endl;
                        //cerr <<i<< " =:= " << int_val<<endl;
                        val[i+fromi] = int_val;
                    }
                    //if((int)(c)==255)
                }
                //cerr << endl;

                fromi+=read_size;

                //printf("Packet number received: %i\n",packet_index);
                //printf("Packet size: %i\n",read_size);
                //cout<<"from receive_map 3"<<endl;

                //if(read_size !=write_size) {printf("error in read write\n");}
                //cerr<<read_size<<"... from receive_map 2.5t ..."<<endl;
                //Increment the total number of bytes read
                recv_size += read_size;
                packet_index++;
                //printf("Total received map size: %i\n",recv_size);printf(" \n");
                //cerr << "Error while opening the file " << packet_index<<" read_size "<<read_size<<endl;
                map_recieved_flag=true;
                if(read_size<=0)
                    return 0;
                    //break;
                //sleep(1);
                }
        }

    }

    cerr<<"from receive_map "<<map_is_compressed_<<endl;
    if(map_is_compressed_){
        vector<int> map_uncompressed = simpleUncompressionVector(val);
        //cout<<map_uncompressed.size()<<" from receive_map 2.6: "<<endl;
        convertToMap(map_uncompressed, id);
    }else{
        convertToMap(val, id);
    }
    cout<<val.size()<<" <=val map_uncompressed size is ============== : "<<endl;

    //fclose(image);
    cerr << "Map successfully Received " <<map_receive_inc<<endl;
    map_receive_inc++;
    //printf("Map successfully Received!\n");
    fflush (stdout);
    //sleep(1);
    return 1;
}

bool init(){
    //char *parray;
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM/*|O_NONBLOCK*/, 0);

    if (socket_desc == -1) {
        printf("Could not create socket");
        return false;
    }
    //close(socket_desc);ERROR edge
    return true;
}
void close_socket(void)
{
    //cerr << "Error while opening the file 111" <<endl;
    int sutd = shutdown(socket_desc, SHUT_RD);
    close(socket_desc);
    cout<<"We closed socket"<<endl;
}
bool process_recieving_map(robot_in rb)
{
    //cout<<"process_recieving_map 0"<<endl;
    memset(&server,0,sizeof(server));
    char * ipip = new char[rb.ip.size() + 1];
    //ipip = "10.1.16.244";
    std::copy(rb.ip.begin(), rb.ip.end(), ipip);
    ipip[rb.ip.size()] = '\0'; // don't forget the terminating 0
    cout<<" geting map from robot's id "<<rb.id<<" ip "<<rb.ip<<endl;
    // don't forget to free the string after finished using it

    server.sin_addr.s_addr = inet_addr(ipip);
    server.sin_family = AF_INET;
    server.sin_port = htons(8889);
    delete[] ipip;
    //Connect to remote server
    if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        cout<<strerror(errno);
        //close(socket_desc);
        puts(" Connect Error");
        cout<<connect(socket_desc, (struct sockaddr *)&server, sizeof(server))<<endl;
        conect_to_server=false;
        return false;
    }else{
        conect_to_server=true;  rb.inital_pose.x=0;rb.inital_pose.y=0;rb.inital_pose.theta=0;

        puts(" Connected\n");
        receive_map(socket_desc, rb.id);
        cout<<"processing"<<endl;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_receive_torob");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //get robot handler from the maunch file

    robot_in rb;
    //map_topic_="/map";private_nh.getParam("map_topic", map_topic_);
    int s;//size of the robot handler

    XmlRpc::XmlRpcValue string_list;
    private_nh.getParam("robots_handler", string_list);
    map_is_compressed_ = false;
    private_nh.getParam("map_is_compressed", map_is_compressed_);
    cout<<" map_is_compressed_]*********** "<<map_is_compressed_<<endl;

    use_torob_map_ = true;
    private_nh.getParam("use_torob_map", use_torob_map_);

    if(use_torob_map_) map_is_compressed_ = false;

    if ((string_list.getType() == XmlRpc::XmlRpcValue::TypeArray) /*&& (string_list.size() == 4)*/) {
    s = (int)string_list.size()/3;
    robots_in_map_ips.resize(s);

    cout<<" We have "<<s<<" robots "<<endl;
    for (int i=0; i<s; i++) {
        ROS_ASSERT(string_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        rb.id = i;
        //string idstr;idstr = static_cast<ostringstream*>( &(ostringstream() << rb.id+1) )->str();
        rb.ip = static_cast<std::string>(string_list[i]);rb.out_map_topic=static_cast<std::string>(string_list[i+s]);
        rb.out_pose_topic=static_cast<std::string>(string_list[i+s*2]);
        rb.inital_pose.x = 0;rb.inital_pose.y=0;rb.inital_pose.theta=0;
        rb.current_pose.x = 0;rb.current_pose.y=0;rb.current_pose.theta=0;
        robots_in_map_ips[i] = rb;
        cout<<" robots_in_map_ips[i] is]*********** "<<robots_in_map_ips[i].ip<<" rb.out_map_topic "<<rb.out_map_topic<<endl;
    }
    }else{cout<<" ERROR: ROBOT HQNDLER NOT SUITABLE!!! PLEASE CHECK YOUR LAuNCH FILE "<<endl;}


    //create the map publisher
    map_publisher.resize(s);
    map_metadata_subs.resize(s);
    pose_publisher.resize(s);
    grap_to_map_publisher.resize(s);
    robot_model_publisher.resize(s);

    map_.resize(s);
    for (int i=0; i<s; i++){
        map_metadata_subs[i]     = nh.advertise<nav_msgs::MapMetaData>(robots_in_map_ips[i].out_map_topic+"_metadata", 10, true);
        map_publisher[i]         = nh.advertise<nav_msgs::OccupancyGrid>(robots_in_map_ips[i].out_map_topic, 10, true);
        pose_publisher[i]        = nh.advertise<geometry_msgs::PoseStamped>(robots_in_map_ips[i].out_pose_topic, 10, true);
        grap_to_map_publisher[i] = nh.advertise<visualization_msgs::MarkerArray>(robots_in_map_ips[i].out_map_topic + "_graph",10);
        robot_model_publisher[i] = nh.advertise<visualization_msgs::Marker>(robots_in_map_ips[i].out_map_topic + "_robot_torob",10);

    }
    resolution_        = 0.050;
    conect_to_server   = false;
    map_recieved_flag  = false;
    width_             = 224;
    height_            = 192;
    xrpose_            = 0;
    yrpose_            = 0;
    yr_currt_pose_     = 0;
    yr_currt_pose_     = 0;
    trpose_            = 0;
    trb_resolution_    = resolution_;
    init_map();

    //socket_created     = init();
    map_receive_inc = 0;
    short ip=0;
    //ros::Rate loop_rate(10.0);//0.5);
    while( nh.ok() )
    {
        if(init()){
            if(process_recieving_map(robots_in_map_ips[ip])){
                map_publisher[ip].publish(map_[ip]);
                map_metadata_subs[ip].publish(map_[ip].info);
                geometry_msgs::PoseStamped P; P.pose.position.x=robots_in_map_ips[ip].current_pose.x;
                P.pose.position.y=robots_in_map_ips[ip].current_pose.y;
                pose_publisher[ip].publish(P);
                cout<<ip<<" ***********[ map published ]*********** "<<robots_in_map_ips[ip].out_map_topic<<" "<<map_is_compressed_<<endl;
            }
            close_socket();
        }

        ip++;
        if(ip>s-1) ip=0;
        ros::spinOnce();
        //loop_rate.sleep();
    }
    close_socket();
    return 0;
};

