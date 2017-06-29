/*
* UdpUtils.cpp
*
*  Created on: May 29, 2016
*      Author: laptopyellow
*/
#include "UdpUtils.h"
#include <sys/socket.h>
#include <netdb.h>
#include <iostream>
using namespace std;


addrinfo* UdpUtils::GetAddressInfo(const char* address, const char* port, const addrinfo& hints) {
	int rv;
	addrinfo* servinfo;
	//cout<<"GetAddressInfo"<<endl;
	if ((rv = getaddrinfo(address, port, &hints, &servinfo)) != 0) {
		throw CommunicationException(gai_strerror(rv));
	}
	return servinfo;
}

int UdpUtils::CreateSocket(const addrinfo* servinfo) {
	int socket_fd;
	if ((socket_fd = socket(servinfo->ai_family, servinfo->ai_socktype,
		servinfo->ai_protocol)) == -1) {
		throw CommunicationException("listener: socket");
	}
	return socket_fd;
}

void UdpUtils::UdpBind(int socket_fd, addrinfo* servinfo) {
	if (bind(socket_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
		close(socket_fd);
		throw CommunicationException("listener: bind");
	}
}
int UdpUtils::CreateUdpReceiveSocket(const char* address, const char* port) {
	int socket_fd;
	struct addrinfo hints, *servinfo;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; // set to AF_INET to force IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	try {
		servinfo = UdpUtils::GetAddressInfo(address, port, hints);
		socket_fd = UdpUtils::CreateSocket(servinfo);
		UdpUtils::UdpBind(socket_fd, servinfo);
	}
	catch (exception& e) {
		cout << e.what();
	}
	freeaddrinfo(servinfo);
	return socket_fd;
}

void UdpUtils::CreateAddressInfo(const char* address, const char* port, addrinfo*& servinfo) {
	struct addrinfo hints;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
	hints.ai_socktype = SOCK_DGRAM;

	try {
		servinfo = UdpUtils::GetAddressInfo(address, port, hints);
	}
	catch (exception& e) {
		cout << e.what();
	}
}

int UdpUtils::CreateUdpSendSocket() {
	int socket_fd;
	struct addrinfo servinfo;
	memset(&servinfo, 0, sizeof(servinfo));
	servinfo.ai_family = AF_INET; // set to AF_INET to force IPv4
	servinfo.ai_socktype = SOCK_DGRAM;
	servinfo.ai_protocol = 0;

	try {
		socket_fd = UdpUtils::CreateSocket(&servinfo);
	}
	catch (exception& e) {
		cout << e.what();
	}
	return socket_fd;
}



