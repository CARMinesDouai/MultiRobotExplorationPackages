/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/
#include <ros/ros.h>
#define MLOG ROS_INFO
#ifdef __cplusplus
extern "C"
{
#endif
#include "bridge/data_portal.h"

#ifdef __cplusplus
}
#endif
#include "helpers/helper.h"
#include "multi_master_bridge/NeighbourId.h"
#include "multi_master_bridge/NeighbourList.h"
#define EXPIRED_TIME 5 //s

ros::Publisher pub;
multi_master_bridge::NeighbourList neighbors;
std::string publish_to,_interface,map_update_,map_local_;
double _init_x,_init_y,_init_z,_sending_rate;
static int sockfd=-1;
struct inet_id_ id;

void neighbors_discover(const multi_master_bridge::NeighbourList::ConstPtr& msg)
{
	neighbors = *msg;
}
void send_newmap(const multi_master_bridge::MapData::ConstPtr& msg)
{
	// send amap
	MapDataHelper hp;
	
 	multi_master_bridge::MapData* data = new  multi_master_bridge::MapData(*msg);
	
	//data->position.x = _init_x;
	//data->position.y = _init_y;
	//data->position.z = _init_z;
	data->ip = string(inet_ntoa(id.ip));
	hp.consume((void*)data);
	struct portal_data_t d = hp.getPortalDataFor(inet_ntoa(id.ip));
	d.publish_to = (char*)publish_to.c_str();
	d.hash = MapDataHelper::hash();
	//send data to all active neighbour
	for(int i = 0; i < neighbors.size;i++ )
	{
		if(neighbors.list[i].status != -1 ) //&& neighbors.list[i].status != 1
		{
			ROS_INFO("Feed map to %s (%s):%d", neighbors.list[i].ip.c_str(),  neighbors.list[i].name.c_str() , neighbors.list[i].port);
			teleport_raw_data( neighbors.list[i].ip.c_str(), neighbors.list[i].port,d);
		}
	}
	if(d.data) free(d.data);
	if(data) delete data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_exchange");
	ros::NodeHandle n("~");
	n.param<std::string>("publish_to",publish_to, "/other_map");
	n.param<std::string>("network_interface",_interface, "wlan0");
	n.param<std::string>("map_update_topic",map_update_, "/map_update");
	//n.param<double>("init_z",_init_z, 0.0);
	//n.param<double>("init_x",_init_x, 0.0);
	//n.param<double>("init_y",_init_y, 0.0);
	n.param<double>("sending_rate",_sending_rate, 1.0);
	neighbors.size = 0;
	//pub = n.advertise<nav_msgs::OccupancyGrid>("other_map", 1000);
	ros::Subscriber sub = n.subscribe<multi_master_bridge::NeighbourList>("/new_robot", 50,&neighbors_discover);
	ros::Subscriber sub1 = n.subscribe<multi_master_bridge::MapData>(map_update_, 100,&send_newmap);
	ros::Rate loop_rate(_sending_rate);
	id = read_inet_id(_interface.c_str());
	ROS_INFO("My address %s on %s",inet_ntoa(id.ip), _interface.c_str());
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
    

	return 0;
    
}
/*
nav_msgs::OccupancyGrid m;
	m.header.seq = 10;
	m.header.stamp = ros::Time::now();
	m.header.frame_id = "feeder";
	m.info.map_load_time = ros::Time::now();
	m.info.resolution = 0.1;
	m.info.width = 480;
	m.info.height = 320;
	m.info.origin.position.x = 1.2;
	m.info.origin.position.y = 1.4;
	m.info.origin.position.z = 1.7;
	m.info.origin.orientation.x = 0.7;
	m.info.origin.orientation.y = 0.45;
	m.info.origin.orientation.z = 40.56;
	m.info.origin.orientation.w = 45.6666;
	int i=0;
	for(i;i<255;i++) m.data.push_back(i+1);
	OccupancyGridHelper hp;
	hp.consume((void*)&m);
    struct portal_data_t d = hp.getPortalDataFor("10.1.16.45");
	d.publish_to = "/food_chain";
	d.hash = OccupancyGridHelper::hash();

	//ROS_INFO("Feed x:%f, st.sec:%d, st.nsec:%d, fid:%s", m.seq, m.stamp.sec,m.stamp.nsec,m.frame_id.c_str());
	pub.publish(m);
	teleport_raw_data(msg->ip.c_str(),msg->port,d);
	
	// free allocated memory
	if(d.data) free(d.data);
*/