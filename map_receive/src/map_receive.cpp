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

#include <nav_msgs/OccupancyGrid.h>
#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>   
#include<sys/ioctl.h>
#include<unistd.h>  
#include<iostream>
#include<fstream>
#include<errno.h>
using namespace std;

#define MAX_BUFFER_SIZE 20240 //10240
//we define a structure handling the robot
struct robot_in{
    string ip;
    short  id;
    string out_map_topic;
    string out_pose_topic;
    string out_goal_topic;
    geometry_msgs::Pose2D goal_pose;
    geometry_msgs::Pose2D current_pose;
};

//ros::Subscriber map_subscriber;       //creating map subscribers
vector<ros::Publisher>  map_metadata_subs;
vector<nav_msgs::OccupancyGrid> map_; //a vector handling the incoming maps;
//std::string map_topic_;
vector<robot_in> robots_in_map_ips;   //a vector handling robot to be connect to;
vector<ros::Publisher> map_publisher; //ros::Publisher map_publisher;
vector<ros::Publisher> pose_publisher; //ros::Publisher pose_publisher of the other robots
vector<ros::Publisher> goal_publisher; //ros::Publisher pose_publisher of the other robots

int xrpose_, yrpose_, trpose_;        // initia position of the robot
int xr_currt_pose_, yr_currt_pose_;        // initia position of the robot

float resolution_;
bool  map_recieved_flag;
bool map_is_compressed_;
int   width_, height_;
//about socket
int socket_desc;
struct sockaddr_in server;
bool conect_to_server;
bool socket_created;
int idmapreceived;
//we initialize a map
void init_map(){
    for(int i=0;i<2;i++)
    {
        map_[i].info.resolution           = resolution_;
        map_[i].info.origin.position.x    = -5.0;
        map_[i].info.origin.position.y    = -5.0;
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
//convert a vector to map
void convertToMap(vector<int> in, short id)
{
    int s  = map_[id].data.size();
    int sm =in.size();
    if (sm < s){ cerr<<s<<" <== s and sm ==> "<<sm<<endl;s = sm;}
    int v  = 0;
    for(int i=0;i<s;i++){
        v         = in[i];
        if      (v==0)   v= 100;
        else if (v==204) v=-1;
        else if (v==255) v= 0;
        //if(i<sm)
        map_[id].data[i] = v;
        //cerr <<"/"<<i<< ": " << v;
    }
    //cout<<s<<" <=s ====dddddddddd========== "<<" "<<id<<endl;
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
    do{px = read(socket, &xrpose_, sizeof(int));}while(px<0);
//cout<<"the robot is in xrpose_ "<<xrpose_<<endl;

    //Find the inital pose of the robot y
    do{py = read(socket, &yrpose_, sizeof(int));}while(py<0);//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;


    //Find the current pose of the robot x
    do{pxc = read(socket, &xr_currt_pose_, sizeof(int));}while(pxc<0);//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;


    //Find the current pose of the robot y
    do{pyc = read(socket, &yr_currt_pose_, sizeof(int));//cout<<"the robot is in yrpose_ "<<yrpose_<<endl;
    }while(pyc<0);

    robots_in_map_ips[id].goal_pose.x      = (double) ((double) xrpose_/(double) 100);
    robots_in_map_ips[id].goal_pose.y      = (double) ((double) yrpose_/(double) 100);
    robots_in_map_ips[id].goal_pose.theta  = (double) trpose_;//not used till now

    robots_in_map_ips[id].current_pose.x     = (double) ((double) xr_currt_pose_/(double) 100);
    robots_in_map_ips[id].current_pose.y     = (double) ((double) yr_currt_pose_/(double) 100);
    robots_in_map_ips[id].current_pose.theta = (double) trpose_;//not used till now

    //cout<<"the robot is in rpose_ "<<robots_in_map_ips[id].goal_pose.x<<" "<<robots_in_map_ips[id].goal_pose.y<<endl;

    map_[id].info.width                      =  width_;
    map_[id].info.height                     =  height_;

    map_[id].info.origin.position.x          = -width_*resolution_/2;
    map_[id].info.origin.position.y          = -height_*resolution_/2;

    map_[id].data.resize(width_*height_);

    buffersize = MAX_BUFFER_SIZE;
    if(size<MAX_BUFFER_SIZE){
        buffersize = size;
    }

    char map_recieved[buffersize],verify = '1';//50176
    //cout<<"from receive_map 11.0 : stat is"<<buffersize<<endl;

    //printf("Packet received.\n");
    //printf("Packet size: %i\n",stat);
    //printf("Image size: %i\n",size);printf(" \n");

    char buffer[] = "Got it";

    //Send our verificationsize signal
    do{
    stat = write(socket, &buffer, sizeof(int));
    }while(stat<0);

    //printf("Reply sent\n");printf(" \n");

    //cout<<"from receive_map 2"<<endl;

    //========Loop while we have not received the entire map yet
    double timeout_ = 10.0;
    if (!map_is_compressed_) timeout_ = 50.0;
    struct timeval timeout = {timeout_,0};

    fd_set fds;
    int buffer_fd;//, buffer_out;
    int fromi;
    //int whole_map_size;
    fromi=0;
    vector<int> val;
    val.resize(size);
    while(recv_size < size) {//while(packet_index < 2){

        //cerr << "Error while opening the file 0" <<endl;
        FD_ZERO(&fds);
        FD_SET(socket,&fds);
        //cout<<"from receive_map 11.1 : stat is"<<buffersize<<endl;
        buffer_fd = select(FD_SETSIZE,&fds,NULL,NULL,&timeout);
        //cout<<"from receive_map 2.2 buffer_fd is: "<<buffer_fd<<endl;
        //cerr << "Error while opening the file 1" <<endl;

        if (buffer_fd < 0){
            printf("error: bad file descriptor set.\n");
            cerr << "Error while opening the file == 2: " <<buffer_fd<<endl;//sleep(1);
            return 0;
            break;
        }
        if (buffer_fd == 0){
            printf("error: buffer read timeout expired.\n");
            ;cerr << "Error while opening the file == 3: " <<buffer_fd<<endl;//sleep(1)
            return 0;
            break;
        }
        if (buffer_fd > 0)
        {
            //cout<<"from receive_map 2.3"<<endl;
            //cerr << "Error while opening the file 4: " <<buffer_fd<<endl;
            do{
                read_size = read(socket,map_recieved, buffersize);
                //cerr << "Error while opening the file  6 i+fromi "<<endl;
            }while(read_size <0);
            //sleep(5);
            //cerr << "Error while opening the file 5" <<read_size<<endl;
            unsigned int int_val=0;
            //cerr << "Error while opening the file 6: " <<fromi<<endl;
            for(int i=0;i<read_size;i++){
                //cout<<read_size<<" <==read_size : from receive_map 2.5 "<<i<<" :i+fromi==> "<<i+fromi<<" : "<<map_.data.size()<<endl;
                //cerr <<i<< " Error while opening the file  6 i+fromi "<< i+fromi<<" "<<map_[id].data.size()<<" "<<size<<endl;
                unsigned char c = (unsigned char) map_recieved[i];
                //cerr << "Error while opening the file 6 i: " <<i<<endl;
                if(i+fromi < size){
                    int_val      = (unsigned int)c;
                    val[i+fromi] = int_val;
                    //cout<<int_val<<" , "<<endl;
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
            //recv_size += read_size;
            packet_index++;
            //printf("Total received map size: %i\n",recv_size);printf(" \n");

            //cerr << "Error while opening the file " << packet_index<<" read_size "<<read_size<<endl;
            map_recieved_flag=true;
            if(read_size<=0){
                break;//return 0;
                cout<<" what is the ************************: "<<endl;

                }
            //sleep(1);
            }
    }
    //cout<<" $$$$$$$$$$$$$$$$$$$ it is here dddddddddddd: "<<endl;

    //cout<<"from receive_map"<<endl;
    if(map_is_compressed_){
        vector<int> map_uncompressed = simpleUncompressionVector(val);
        cout<<map_uncompressed.size()<<" +++++++++ compressed data size is: "<<val.size()<<endl;
        convertToMap(map_uncompressed, id);
    }else{
        cout<<" $$$$$$$$$$$$$$$$$$$ it is here 1: "<<endl;

        convertToMap(val, id);
    }
    //cout<<val.size()<<" <=val map_uncompressed size is ============== : "<<map_uncompressed.size()<<endl;

    //fclose(image);
    cerr << "Map successfully Received " <<idmapreceived<<endl;
    idmapreceived++;
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
    //close(socket_desc);
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
        conect_to_server=true;  rb.goal_pose.x=0;rb.goal_pose.y=0;rb.goal_pose.theta=0;

        puts(" Connected\n");
        receive_map(socket_desc, rb.id);
        cout<<"processing"<<endl;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_receive");
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
    if ((string_list.getType() == XmlRpc::XmlRpcValue::TypeArray) /*&& (string_list.size() == 4)*/) {
    s = (int)string_list.size()/3;
    robots_in_map_ips.resize(s);

    cout<<" We have "<<s<<" robots "<<endl;
    for (int i=0; i<s; i++) {
        ROS_ASSERT(string_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        rb.id = i;
        //string idstr;idstr = static_cast<ostringstream*>( &(ostringstream() << rb.id+1) )->str();
        rb.ip = static_cast<std::string>(string_list[i]);rb.out_map_topic=static_cast<std::string>(string_list[i+s]);
        rb.out_pose_topic = static_cast<std::string>(string_list[i+s*2]);
        if(i == 0) rb.out_goal_topic = "/tb1/goal";
        if(i == 1) rb.out_goal_topic = "/tb3/goal";
        rb.goal_pose.x = 0;rb.goal_pose.y=0;rb.goal_pose.theta=0;
        rb.current_pose.x = 0;rb.current_pose.y=0;rb.current_pose.theta=0;
        robots_in_map_ips[i] = rb;
        cout<<" robots_in_map_ips[i] is]*********** "<<rb.out_pose_topic<<" rb.out_map_topic "<<rb.out_map_topic<<endl;
    }
    }else{cout<<" ERROR: ROBOT HQNDLER NOT SUITABLE!!! PLEASE CHECK YOUR LAuNCH FILE "<<endl;}


    //create the map publisher
    map_publisher.resize(s);
    map_metadata_subs.resize(s);
    pose_publisher.resize(s);
    goal_publisher.resize(s);

    map_.resize(s);
    for (int i=0; i<s; i++){
        map_metadata_subs[i]  = nh.advertise<nav_msgs::MapMetaData>(robots_in_map_ips[i].out_map_topic+"_metadata", 10, true);
        map_publisher[i]      = nh.advertise<nav_msgs::OccupancyGrid>(robots_in_map_ips[i].out_map_topic, 10, true);
        pose_publisher[i]     = nh.advertise<geometry_msgs::PoseStamped>(robots_in_map_ips[i].out_pose_topic, 10, true);
        goal_publisher[i]     = nh.advertise<geometry_msgs::PoseStamped>(robots_in_map_ips[i].out_goal_topic, 10, true);
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
    init_map();

    //socket_created     = init();
    idmapreceived=0;
    short ip=0;
    ros::Rate loop_rate(0.5);//10.0);//
    while( nh.ok() )
    {
        if(init()){
            if(process_recieving_map(robots_in_map_ips[ip])){
                map_publisher[ip].publish(map_[ip]);
                map_metadata_subs[ip].publish(map_[ip].info);
                geometry_msgs::PoseStamped P;
                P.pose.position.x = robots_in_map_ips[ip].current_pose.x;
                P.pose.position.y = robots_in_map_ips[ip].current_pose.y;
                P.header.frame_id = "/tb1/map";
                pose_publisher[ip].publish(P);
                //publishing the current assigned goal
                P.pose.position.x = robots_in_map_ips[ip].goal_pose.x;
                P.pose.position.y = robots_in_map_ips[ip].goal_pose.y;
                goal_publisher[ip].publish(P);

                cout<<robots_in_map_ips[ip].current_pose.y<<" ***********[ map published ]*********** "<<robots_in_map_ips[ip].out_map_topic<<endl;
            }
            //cout<<" map_is_compressed is $$$$$$$$$$$$$$$$$$$ **** "<<map_is_compressed_<<endl;

            close_socket();
        }

        ip++;
        if(ip>s-1) ip=0;
        ros::spinOnce();
        loop_rate.sleep();
    }
    close_socket();
    return 0;
};


