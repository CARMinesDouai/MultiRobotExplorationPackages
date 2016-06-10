#include "UDPCom.h"

// System :
#include <iostream>
#include <string.h>

#include <netinet/ip.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

using namespace std;
using namespace kz;

UDPCom :: UDPCom()
{
	a_ownId= 0;

//	a_bufferSize= 1024;
//	a_buffer= new char[1024];
}

UDPCom :: ~UDPCom()
{
	for(int i= 0; i < a_nboRobot; ++i)
	{
		close( a_robotSock[i] );
	}
}

// Engine :
//void UDPCom :: initialize()
void UDPCom :: initialize( int ownId, int nbAddr, const char ** addr )
{
	// Initialize Structure :
	a_nboRobot= nbAddr;
	a_ownId= ownId;

	a_robotAddr= new char*[a_nboRobot];
	a_robotSock= new int[a_nboRobot];

	for(int i= 0; i < a_nboRobot; ++i)
	{
		a_robotAddr[i]= new char[18];
		strcpy( a_robotAddr[i], addr[i] );

		a_robotSock[i]= socket( AF_INET, SOCK_DGRAM, 0 );
		int flags= fcntl(a_robotSock[i], F_GETFL);
		fcntl(a_robotSock[i], F_SETFL, flags | O_NONBLOCK); // unset : udp_socket_flag & ~O_NONBLOCK

		sockaddr_in addr;
		addr.sin_family= AF_INET;
		addr.sin_port= htons(1400);
		inet_aton( a_robotAddr[i], &(addr.sin_addr) );

		bind( a_robotSock[i], (sockaddr*)(&addr), sizeof(sockaddr) );
	}
}

void UDPCom :: process(float)
{
}

// Communication :
void UDPCom :: sendMsg( int ir, const Message &msg )
{
	Data _data;

	_data.nboChar= msg.a_content.size();
	strcpy( _data._char, (const char *)msg.a_content );

	_data.nboInt= msg.a_flag.size();
	for( int i= 0; i < _data.nboInt; ++i )
		_data._int[i]= msg.a_flag[i];

	_data.nboFloat= msg.a_value.size();
	for( int i= 0; i < _data.nboFloat; ++i )
	_data._float[i]= msg.a_value[i];

	sockaddr_in addr;
	addr.sin_family= AF_INET;
	addr.sin_port= htons(1400);
	inet_aton( a_robotAddr[ir], &(addr.sin_addr) );

	if( sendto( a_robotSock[ir], &_data, sizeof(_data), 0, (sockaddr*)(&addr), sizeof(sockaddr) ) == -1 )
		cout << "UDP send error " << errno << " to robot " << ir << endl;
}

bool UDPCom :: receiveMsg(Message & msg)
{
	int size= recv( a_robotSock[a_ownId], &a_buffer, sizeof(a_buffer), 0 );

	if( size < 0 )
	{
	  	if( errno != EAGAIN && errno != EWOULDBLOCK )
			cout << "UDP reception error " << errno << endl;

		return false;
	}

	assert(a_buffer.nboChar <= 128);
	assert(a_buffer.nboInt <= 238);
	assert(a_buffer.nboFloat <= 238);

	msg.a_content= a_buffer._char;
	msg.a_flag.initialize( a_buffer._int, a_buffer.nboInt );
	msg.a_value.initialize( a_buffer._float, a_buffer.nboFloat );

	return true;
}

/*
bool UDPCom :: receiveMsg(Message & msg)
{
	int size= recv( a_robotSock[a_ownId], &_char, 127, 0 );

	if( size < 0 )
	{
	  	if( errno != EAGAIN && errno != EWOULDBLOCK )
			cout << "UDP reception error " << errno << endl;

		return false;
	}

	_char[size]= '\0';
	msg.a_content= _char;

	return true;
}
*/
