#ifndef _UDPCOM_HPP_
#define _UDPCOM_HPP_

#include "MAS/Message.h"

class UDPCom
{
	struct Data
	{
		int nboChar;
		int nboInt;
		int nboFloat;

		char _char[128];
		int _int[32];
		float _float[32];
	};

protected :
	int a_nboRobot, a_ownId;
	char ** a_robotAddr;
	int * a_robotSock;

//	char a_charbuf[128];
	Data a_buffer;

public :
	UDPCom();
	~UDPCom();

	// Engine :
	void initialize(int ownId, int nbAddr, const char ** addr);
	void process(float deltaT);

	// getter :
	int getOwnIdentifier()const{ return a_ownId; }

	// communication :
	void sendMsg( int ir, const kz::Message &msg );
	bool receiveMsg( kz::Message &msg);
};

#endif

