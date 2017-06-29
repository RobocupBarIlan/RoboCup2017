#ifndef UDP_UTILS_H
#define UDP_UTILS_H
#include "udpincludes.h"
#include "CommUtils.h"

class UdpUtils
{
public:
	static void UdpBind(int socket_fd, addrinfo* servinfo);
	static int CreateSocket(const addrinfo* servinfo);
	static addrinfo* GetAddressInfo(const char* address, const char* port, const addrinfo& hints);
	static int CreateUdpReceiveSocket(const char* address, const char* port);
	static int CreateUdpSendSocket();
	static void CreateAddressInfo(const char* address, const char* port, addrinfo*& servinfo);


};

#endif
