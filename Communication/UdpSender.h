/*
* UdpSender.h
*
*  Created on: May 29, 2016
*      Author: laptopyellow
*/

#ifndef UDPSENDER_H_
#define UDPSENDER_H_

#include "UdpUtils.h"
#include <iostream>
using namespace std;

class UdpSender {
public:
	static UdpSender* GetInstance();
	void Send(const char* bytestream);

private:
	UdpSender();
	~UdpSender();

	enum { PLAYERS_NUM = 6 };
	void Init();
	static UdpSender* m_instance;
	const char* m_send_port;
	int m_socket_fd;
	addrinfo* m_robots_address[PLAYERS_NUM];
	addrinfo* m_referee_address;

};


#endif /* UDPSENDER_H_ */
