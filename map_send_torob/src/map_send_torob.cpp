/*
 * Copyright (c) 2015, Khelifa Baizid <Khelifa.Baizid@mines-douai.fr>
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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include<fstream>
#include<errno.h>
#include <tf/transform_listener.h>

#include "visimap.h"
#include "ogmap.h"


#define PORT "8889"
#define STDIN 0
#define MAX_BUFFER_SIZE 30240//10240
typedef unsigned char uchar;

using namespace :: mia;
using namespace std;

//some global variables
// Maps:
OGMap   gridmap;             //guillaume torob map
VisiMap visimap;             //guillaume torob map

std::string map_topic_, pose_topic_;
struct sockaddr name;        // namr of the socket
int inc;                     // how manytimes we send the map
nav_msgs::OccupancyGrid map_;// incoming map topic

int width_;                  // map topic width
int height_;                 // map topic heilght

int socket_desc, new_socket = 0;//, c, read_size, buffer = 0;
struct sockaddr_in server , client;
bool socket_is_created;      // did the socket created?
bool map_exists;             // map existing?
bool compress_map_;          // in case we wana send a compresssed map
bool use_torob_map_;         // in case we are using torob map
bool pose_exists;            // map existing?
bool pose_is_needed_;
int x_init_pose_;  int y_init_pose_;
int x_currt_pose_; int y_currt_pose_;
string ip_;
int idmap_sent;
//this function compress an integer vector

vector<int> simpleCompressionVector(vector<int> in)
{
    vector<int> out;
    //in the incoming vector is empty we return
    if(!in.size()) return in;

    int s=in.size();
    int v=0;
    int inc_i=0;
    while(inc_i<s){
        v=in[inc_i];
        out.push_back(v);
        //now we check how much elements are similar before changing and we stor them
        int val_id=1;//cout<<"inc_i+val_id "<<inc_i+val_id<<endl;
        while(v==in[inc_i+val_id] && val_id<254)
        {//cout<<"val_id "<<val_id<<endl;
            val_id++;
            //cout<<"simpleCompressionVector"<<endl;

        }
        inc_i=inc_i+val_id;
        out.push_back(val_id);
    }
    cout<<"after while loop of compression"<<endl;
    //for(int i=0;i<out.size(); i++)cout<<" cm "<<out[i]<<endl;
    return out;
}
//this function uncompress an integer vector(just for testing, IT IS NOT USED HERE)
vector<int> simpleUncompressionVector(vector<int> in)
{
    vector<int> out;
    //in the incoming vector is empty we return
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

vector<int> reduiceToChair(vector<int> in)
{
    int s = in.size();
    vector<int> out;
    out.resize(s*2);
    int inc=0;
    for (int i=0; i < 2*s; i+=2)
    {
        if(in[inc]<256)
        {
            out[i]   = in[inc];
            out[i+1] = 0;
        }else{
            out[i]   = 255;
            out[i+1] = in[inc]-255;
            if(out[i+1]>255) cerr<<i<<" : "<<in[inc]<<" !!!!!!!!!!!!!! PROBLEM WITH VALUE IN TOROB MAP !!!!!!!!!!!!!"<<endl;
        }
        inc++;
    }
    //sleep(5);
    return out;
}

vector<int> convertToChar(vector<int> in)
{
    vector<int> out;
    for (int i=0; i < in.size(); i++)
    {
        //cout<<i<<" "<<in[i]<<endl;
        if(in[i] >= 255){
            int m  = in[i]/255;
            int mu = in[i] % 255;

            out.push_back(m);
            out.push_back(mu);
        }else{
            out.push_back(0);
            out.push_back(in[i]);
        }
    }
    return out;
}

//this function gets a compressed map
vector<int> getCompressedMap()
{
    if(!use_torob_map_)
    {
        int whole_map_size=(int)height_*(int)width_;
        vector<int>  val;
        val.resize(whole_map_size);
        //we loop to save the map in a vector
        for (int i=0; i < width_; i++)
        {
            for (int j=0; j<height_; j++)
            {
                //printf("\nWriting to pixel %d %d in map",i,j);
                int p = (int) map_.data[i+j*width_];
                //cout<<p<<" map dim "<<(int)map_.info.height <<" "<<(int)map_.info.width <<endl;
                val[i+j*width_] = p;

                if     (p == 100) val[i+j*width_] = 0;
                else if(p ==  -1) val[i+j*width_] = 204;
                else if(p ==   0) val[i+j*width_] = 255;
                else cerr<<i<<" value out "<<p<<endl;
            }
        }

        cerr<< "size of vector<int>  val :" << val.size() << endl;

        //we compress the map vector
        if(compress_map_ == true){
            return simpleCompressionVector(val);
        }else{
            return val;
        }
        //for (int i=0; i < val_compresssed.size(); i++)cout<<i<<" : "<<val_compresssed[i]<<endl;
        //vector<int> un_val_compresssed = simpleUncompressionVector(val_compresssed);
        //cout<<whole_map_size<<" ======= compressing ====== "<<val_compresssed.size()<<" uncompressed "<<un_val_compresssed.size()<<endl;
        //cout<<"width_ "<<width_<<" width_ "<<height_<<endl;
    }
    else //using torob map
    {
        Data out = visimap.save2();
        visimap.save("send_visimap.vm");
//        visimap.load2(out);
//        visimap.save("testload_visimap.vm");

//        cerr<<"= = = = = = = = = = = = = map data = = = = = = = = = = = = = = :"<< out<< endl;
        vector<int>  val;
        int fs = out.flag_size();
        int vs = out.value_size();
        val.resize(2 + fs + vs);

        //Get min:
        float minV= 0;
        /*for (int i = 0; i < vs; i++)
        {
            minV= min( minV, out.value(i) );
        }

        for (int i = 0; i < vs; i++)
        {
            cerr<< " vi] :"<< out.value(i);
            val[2 + fs + i] = (int)( (out.value(i)-minV) * 100.f); //all position of all nodes and map size origine etc.
        }*/

        for (int i = 0; i < vs; i++)
        {
            //cerr<< " vi] :"<< out.value(i)<<" == "<<(int)(out.value(i))<<endl;
            val[2 + fs + i] = (int)(out.value(i)*100.f); //all position of all nodes and map size origine etc.
        }

        val[0] = fs;
        val[1] = vs;
        for (int i=0; i < fs; i++)
        {
            val[2+i] = out.flag(i); //all edges +  metadata
        }
        for (int i = 0; i < vs; i++)
        {
            //cerr<< " vi] :"<< out.value(i);
            val[2 + fs + i] = (int)(out.value(i)*100.f); //all position of all nodes and map size origine etc.
        }

        //cerr<< "val :"<< val[0];
        //for( int i= 0; i < fs; i++)cerr << ", " << val[i];
        //cerr << "..."<< endl;
        //for( int i= 0; i < vs; i++)cerr << ", " << val[i];
        //cerr << "..." << endl;

        cerr<< "size of vector<int>  val :" << val.size() << endl;

        return val;
    }
}

//this function gets a compressed map
int send_map(int socket){
    //cout<<"sending function ......."<<endl;
    int size, read_size, stat, packet_index;
    int i     = 0;
    int fromi = 0;
    int map_size_sent = 0;

    vector<int> map_compresssed;
    //cout<<"======= uncompressing ====== "<<endl;
    //setting the size of the buffer
    int size_buffer    = MAX_BUFFER_SIZE;
    int whole_map_size = 0;
    map_compresssed    = getCompressedMap();

    //if(use_torob_map_) map_compresssed = reduiceToChair(map_compresssed);
    if(use_torob_map_)
    {
        map_compresssed    = convertToChar(map_compresssed);
    }

    whole_map_size = map_compresssed.size();

    vector<int> test_uncmpst=simpleUncompressionVector(map_compresssed);
    cout<<width_*height_<<" :======= uncompressing test ======: "<<map_compresssed.size()<<" : "<<test_uncmpst.size()<<endl;

    if(map_compresssed.size()<MAX_BUFFER_SIZE){
        size_buffer = map_compresssed.size();
    }else if (map_compresssed.size()>MAX_BUFFER_SIZE*2)
        size_buffer = 20240;
    //if(use_torob_map_) size_buffer = 10240;

    unsigned char send_buffer[size_buffer];//read_buffer[256];// 50176
    unsigned char send_buffer_torob[size_buffer];//read_buffer[256];// 50176
    packet_index = 1;
    //read teh size of the buffer
    read_size = sizeof(send_buffer);
    if(use_torob_map_)read_size = sizeof(send_buffer_torob);

    cout<< "whole_map_size "<<whole_map_size<<" read_size "<<read_size<<" : "<<sizeof(int)<<endl;

    //First we wrtite the size
    if(use_torob_map_)
    {
        int whole_map_size_torob = whole_map_size;
        stat = write(socket, (void *) &whole_map_size_torob, sizeof(int));
    }else{
        stat = write(socket, (void *) &whole_map_size, sizeof(int));
    }

    //Second we wrtite the  width of the map
    stat = write(socket, (void *) &width_, sizeof(int));
    //third we wrtite the height of the map
    stat = write(socket, (void *) &height_, sizeof(int));
    //fourth we wrtite the x intial coordinate of the robot
    stat = write(socket, (void *) &x_init_pose_, sizeof(int));
    //fifth we wrtite the y intial coordinate of the robot
    stat = write(socket, (void *) &y_init_pose_, sizeof(int));
    //sexth we wrtite the x intial coordinate of the robot
    stat = write(socket, (void *) &x_currt_pose_, sizeof(int));
    //seventh we wrtite the y intial coordinate of the robot
    stat = write(socket, (void *) &y_currt_pose_, sizeof(int));
    //loop for the whole size
    int REMAININGDATASIZE = 0;
    bool send_ones        = false;
    bool obtimize_buffer  = false;
    if(!send_ones)
    {
        if(obtimize_buffer){
            while(map_size_sent<whole_map_size){
                int toThisElement;
                toThisElement = 0;
                for(i = 0; i < read_size; i++) {
                    if(i+fromi<whole_map_size-read_size){
                        send_buffer[i]= (unsigned char)(map_compresssed[i+fromi]);
                        //if(i+fromi<10) cerr<<i<<" :: "<<map_compresssed[i+fromi]<<endl;
                    }else
                        toThisElement = i;
                    //cout<<i<<" i+fromi is: "<<i+fromi<<" whole_map_size" << whole_map_size<<endl;
                }
                char send_buffer_remain[toThisElement];
                for(i = 0; i < toThisElement; i++) {
                    send_buffer_remain[i] = (char)send_buffer[i];
                    REMAININGDATASIZE=i;
                }
                //cout<<i<<" fromi is: "<<fromi<< endl;
                fromi+=read_size;
                //cout<<i<<" fromi is:: "<<fromi<< endl;
                map_size_sent+=read_size;
                //cout<< read_size<<" after break "<<map_size_sent<<" fromi "<<fromi<<endl;
                //cout<<" read_size = sizeof(send_buffer) "<<sizeof(send_buffer)<<endl;

                //Send data through our socket
                if(toThisElement==0)
                    do{
                        stat = write(socket, send_buffer, read_size);
                    }while (stat < 0);
                else
                    do{
                        stat = write(socket, send_buffer_remain, toThisElement);
                    }while (stat < 0);
                //we print some onfo about the socket
                //printf("Packet Number: %i\n",packet_index);
                //printf("Packet Size Sent: %i\n",read_size);printf(" \n");
                //cerr<<" REMAININGDATASIZE "<<REMAININGDATASIZE<<endl;
                packet_index++;
                //Zero out our send buffer
                bzero(send_buffer, sizeof(send_buffer));
            }
        }else
        { //not optimized
            //int tothisval=0;
            if(use_torob_map_)
            {
                while(map_size_sent<whole_map_size){
                    //cout<<i<<" fromi is: "<<map_size_sent<<" i+map_size_sent " <<i+map_size_sent<<endl;
                    //if(packet_index==1)
                    bool first = true;
                    for(i = 0; i < read_size; i++) {
                        if(i+map_size_sent < whole_map_size)
                            send_buffer_torob[i] = map_compresssed[i+map_size_sent];
                            //send_buffer[i] = (unsigned char)255;
                        //if(first)cerr<<i<<" cpying....... "<<endl;

                        first=false;
                        //if(tothisval<10000) cout<<i<<" "<<map_compresssed[i+map_size_sent]<<endl;tothisval++;
                    }

                    cerr<<read_size<< "val :"<< send_buffer_torob[0];
                    //for( int i= 1; i < read_size; i++)cerr << ", " << send_buffer_torob[i];
                    cerr << "..."<< endl;
                    //for( int i= 280; i < 320; i++) cerr << ", " << send_buffer_torob[i];
                    cerr << "..." << endl;
                    //fromi+=read_size;
                    //cout<<i<<" fromi is:: "<<fromi<< endl;
                    map_size_sent += read_size;
                    //cout<< read_size<<" after break "<<map_size_sent<<" fromi "<<fromi<<endl;
                    //cout<<" read_size = sizeof(send_buffer) "<<sizeof(send_buffer)<<endl;

                    //Send data through our socket
                    short reading_times=0;
                    do{
                        stat = write(socket, send_buffer_torob, read_size*sizeof(int));
                        cerr<<i<<" writing....... "<<stat << "/" << read_size*sizeof(int) <<endl;
                        reading_times++;
                        if(reading_times>5) break;
                    }while (stat < 0);

                    //we print some onfo about the socket
                    printf("Packet Number: %i\n",packet_index);
                    //printf("Packet Size Sent: %i\n",read_size);printf(" \n");
                    packet_index++;
                    //Zero out our send buffer
                    bzero(send_buffer_torob, sizeof(send_buffer));
                }
            }else //use greed map
            {
                while(map_size_sent<whole_map_size){
                    //cout<<i<<" fromi is: "<<map_size_sent<<" i+map_size_sent " <<i+map_size_sent<<endl;
                    //if(packet_index==1)
                    bool first = true;
                    for(i = 0; i < read_size; i++) {
                        if(i+map_size_sent < /*read_size*/whole_map_size)
                            send_buffer[i] = (unsigned char)(map_compresssed[i+map_size_sent]);
                            //send_buffer[i] = (unsigned char)255;
                        //if(first)cerr<<i<<" cpying....... "<<endl;

                        first=false;

                        //if(tothisval<10000) cout<<i<<" "<<map_compresssed[i+map_size_sent]<<endl;tothisval++;
                    }


                    //fromi+=read_size;
                    //cout<<i<<" fromi is:: "<<fromi<< endl;
                    map_size_sent += read_size;
                    //cout<< read_size<<" after break "<<map_size_sent<<" fromi "<<fromi<<endl;
                    //cout<<" read_size = sizeof(send_buffer) "<<sizeof(send_buffer)<<endl;

                    //Send data through our socket
                    short reading_times=0;
                    do{
                        stat = write(socket, send_buffer, read_size);
                        cerr<<i<<" writing....... "<<stat<<endl;
                        reading_times++;
                        if(reading_times>5) break;
                    }while (stat < 0);

                    //we print some onfo about the socket
                    printf("Packet Number: %i\n",packet_index);
                    //printf("Packet Size Sent: %i\n",read_size);printf(" \n");
                    packet_index++;
                    //Zero out our send buffer
                    bzero(send_buffer, sizeof(send_buffer));
                }

            }
        }
    }
    else // we send the whole package ones
    {
        size_buffer = map_compresssed.size();
        unsigned char send_buffer_all[size_buffer];
        for(i = 0; i < whole_map_size; i++) {
            send_buffer_all[i] = (char)(map_compresssed[i]);
            //cout<<i<<" i+fromi is: "<<i+fromi<<" whole_map_size" << whole_map_size<<endl;
        }

        //cout<<i<<" fromi is: "<<fromi<< endl;
        fromi+=read_size;
        //cout<<i<<" fromi is:: "<<fromi<< endl;
        map_size_sent+=read_size;
        //cout<< read_size<<" after break "<<map_size_sent<<" fromi "<<fromi<<endl;
        //cout<<" read_size = sizeof(send_buffer) "<<sizeof(send_buffer)<<endl;

        //Send data through our socket
        do{
            stat = write(socket, send_buffer_all, size_buffer);
        }while (stat < 0);

        //we print some onfo about the socket
        printf("Packet Number: %i\n",packet_index);
        //printf("Packet Size Sent: %i\n",read_size);printf(" \n");
        packet_index++;
        //Zero out our send buffer
        bzero(send_buffer_all, sizeof(size_buffer));
    }

    idmap_sent++;
    if(use_torob_map_)
        cerr<<idmap_sent<< " We sent a map with size of: "<<read_size<<" "<<whole_map_size<<" REMAININGDATASIZE "<<REMAININGDATASIZE<<endl;
    else
        cerr<< " We sent a map with size of: "<<read_size<<" "<<whole_map_size<<" REMAININGDATASIZE "<<REMAININGDATASIZE<<endl;

 }

int init()
{
    //cout<<"init function ....... 0"<<endl;
    //Create socket
    socket_desc = socket(AF_INET, SOCK_STREAM, 0); //| SOCK_NONBLOCK
    if (socket_desc == -1)
    {
        printf("Could not create socket");

    }else socket_is_created=true;

    char * ipip = new char[ip_.size() + 1];
    std::copy(ip_.begin(), ip_.end(), ipip);

    //Prepare the sockaddr_in structure
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = inet_addr(ipip);//"10.1.16.75"
    server.sin_port        = htons( 8889 );

    delete[] ipip;
    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        puts("bind failed");
        cout<<"bind failed because ....... "<<bind(socket_desc,(struct sockaddr *)&server , sizeof(server))<<endl;
        return 0;
    }
    puts("bind done");

    //Listen
    listen(socket_desc , 20);
    puts("Waiting for incoming connections...");

    //int status, sock, adrlen, new_sd;
    //struct timeval waitd;          // the max wait time for an event
    //fd_set read_flags,write_flags; // the flag sets to be used

    //Accept and incoming connection

    //close(socket_desc);
    //fflush(stdout);
    return 1;
}
void map_subscriber_call_back(const nav_msgs::OccupancyGrid& map)
{
    //_moinag->map_subscriber(map);
    cout<<"recieving local map "<<endl;

    if(!use_torob_map_)
    {
        map_      = map;
        width_    = map_.info.width;
        height_   = map_.info.height;
    }
    else //using torob map
    {
        cerr << "load map to vector :" << endl;
        Data brut("map", 2+(map.info.width*map.info.height), 4);

        brut.a_flag[0]= map.info.width;
        brut.a_flag[1]= map.info.height;

        int countN(0), count0(0), count50(0), count100(0), sum(0);
        for( int end(map.info.width*map.info.height), i(0) ; i < end ; ++i)
        {
            int celVal= (int)( map.data[i] );
            brut.a_flag[2+i]= celVal; //(celVal > 50?100:0);

            countN+= (celVal < 0?1:0);
            count0+= ((celVal == 0)?1:0);
            count50+= ((0 < celVal && celVal < 50)?1:0);
            count100+= ((50 <= celVal && celVal <= 100)?1:0);
            sum+= celVal;
        }

        cout << "map_subscriber : " << map.header.frame_id << "\n\t" << countN << ", " << count0 << ", " << count50 << ", " << count100 << "= " << sum <<  endl;

        brut.a_value[0]= map.info.resolution;

        double roll, pitch, yaw;
        tf::Quaternion quat;
        quat[0]         = map.info.origin.orientation.x;
        quat[1]         = map.info.origin.orientation.y;
        quat[2]         = map.info.origin.orientation.z;
        quat[3]         = map.info.origin.orientation.w;
        tf::Matrix3x3( quat ).getRPY(roll, pitch, yaw);

        brut.a_value[1] = map.info.origin.position.x;
        brut.a_value[2] = map.info.origin.position.y;
        brut.a_value[3] = yaw;

        //cout << "\torigine: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0. | "
          //   << roll << ", " << pitch << ", "<< yaw << "]" << endl;

        //cout << "\torigine: [ 1"<<endl;
        gridmap.initialize( brut );
        //cerr << "\torigine: [ 2"<<endl;
        for (int i = 0; i<4; i++)
            cerr<<brut.a_value[i]<<endl;

        visimap.initialize( gridmap );
        //cerr << "\torigine: [ 3"<<endl;
    }
    map_exists=true;
}

void pose_subscriber_call_back(const geometry_msgs::PoseStamped& msg)
{
    //_moinag->map_subscriber(map);
    cout<<"recieving local pose "<<endl;

    x_currt_pose_ = (int) (msg.pose.position.x*100);//cm
    y_currt_pose_ = (int) (msg.pose.position.y*100);//cm
    pose_exists   = true;
}

void closeSocket()
{
    close(socket_desc);
    //sleep(1);
    //return 0;
}

int process()
{
    cout<<socket_is_created<<" <= socket_is_created::::map_exists iss: "<<map_exists<<endl;
    if(socket_is_created && map_exists && pose_exists)
    {
        //cout<<socket_is_created<<" <= socket_is_created:: 2 ::map_exists iss: "<<map_exists<<endl;
        cout<<" <= wating for some robots to connect to our socket"<<endl;
        int c = 0;
        //cout<<new_socket<<" WORKINF map................<====== "<<sel<<endl;
        c = sizeof(struct sockaddr_in);
        if((new_socket = accept(socket_desc, (struct sockaddr *)&client,(socklen_t*)&c))){
            puts("Connection accepted");
        }

        fflush(stdout);

        if (new_socket<0)
        {
            perror("_...__Accept Failed__..._");
            return 0;
        }

        cout<<"sending now ....... "<<inc<<endl;

        send_map(new_socket);

        inc++;
        //}else
        //    cout<<result<<">>>>>>>>>>>>>>>>>>>> NOT WORKINF map ================"<<endl;
        //sleep(1);
        close(new_socket);
        return 1;
         //}
    }
    else{
    if(!socket_is_created){
        //close(socket_desc);
        //init();
        cout<<"!!!!!!!!!!-- PROBLEM WITH SOCKET --!!!!!!!!!!!"<<endl;
    }else{
        cout<<"!!!!!!!!!!-- WAITING FOR THE MAP --!!!!!!!!!!!"<<endl;
    }

    return 0;
    }
}

int main(int argc, char **argv) {
    cout<<"we want tosend a map ....... 0"<<endl;
    ros::init(argc, argv, "map_send_torob");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    map_topic_="/map";
    private_nh.getParam("map_topic", map_topic_);
    pose_topic_="/posegmapping";
    private_nh.getParam("pose_topic", pose_topic_);
    pose_is_needed_ = true;
    private_nh.getParam("pose_is_needed", pose_is_needed_);

    compress_map_ = false;
    private_nh.getParam("compress_map", compress_map_);
    use_torob_map_= true;
    private_nh.getParam("use_torob_map", use_torob_map_);

    ip_ = "10.1.16.75";
    private_nh.getParam("ip", ip_);
    double x = 0.0, y = 0.0;
    private_nh.getParam("x_initial", x);
    private_nh.getParam("y_initial", y);
    x_init_pose_ = 0; y_init_pose_ = 0;
    x_init_pose_ = (int) (x*100);y_init_pose_ = (int) (y*100);

    x_currt_pose_ = 0; y_currt_pose_ = 0;

    inc=0;//how many times we send the map

    ros::Subscriber map_subscriber  = nh.subscribe( map_topic_, 10, map_subscriber_call_back );
    ros::Subscriber pose_subscriber = nh.subscribe( pose_topic_, 10, pose_subscriber_call_back );

    socket_is_created=false;
    //initializing the map
    width_      = 224;
    height_     = 192;
    map_exists  = false;
    pose_exists = false;
    if(!pose_is_needed_)pose_exists=true;

    idmap_sent=0;
    //calling the init()
    init();

    //ros::Rate loop_rate(10.0);//0.5
    while(nh.ok())
    {
        cout<<"sending map ....... "<< map_topic_<<"  "<<compress_map_<<endl;
        if(process()){
            if(!pose_is_needed_)pose_exists = true;
            else                pose_exists = false;
        }else{
            cout<<"closing the Socket and init "<<endl;
            closeSocket();init();
        }
        ros::spinOnce();
        //loop_rate.sleep();
    }
    closeSocket();
    return 0;
};
