/*
* UdpSender.cpp
*
*  Created on: May 29, 2016
*      Author: laptopyellow
*/
#include "UdpSender.h"

UdpSender* UdpSender::m_instance = NULL;

UdpSender* UdpSender::GetInstance() {
	if (m_instance == NULL)
	{
		m_instance = new UdpSender();
		return m_instance;
	}
	else
	{
		return m_instance;
	}
}

UdpSender::UdpSender() :m_send_port("3939") {
	Init();
}

UdpSender::~UdpSender() {
	if (m_instance != NULL) {
		delete m_instance;
	}
	for (int i = 0; i<PLAYERS_NUM; i++) {
		if (m_robots_address[i] != NULL)
			freeaddrinfo(m_robots_address[i]);
	}
}

void UdpSender::Send(const char* bytestream) {
	int numbytes;
	//cout<<"sending message "<<bytestream<<endl;
	for (int i = 0; i<2; ++i) {
		if ((numbytes = sendto(m_socket_fd, bytestream, strlen(bytestream), 0,
			m_robots_address[i]->ai_addr, m_robots_address[i]->ai_addrlen)) == -1) {
			throw CommunicationException("talker: sendto");
		}
	}
}

void UdpSender::Init() {
	m_socket_fd = UdpUtils::CreateUdpSendSocket();
	cout<<"UdpSender::Init()"<<endl;
	UdpUtils::CreateAddressInfo("192.168.0.102", m_send_port, m_robots_address[0]);
	UdpUtils::CreateAddressInfo("192.168.0.103", m_send_port, m_robots_address[1]);

}



