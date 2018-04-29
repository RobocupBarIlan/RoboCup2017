/*
 * SocketCommunication.h
 *
 *  Created on: Apr 12, 2018
 *      Author: root
 */

#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
//#include <string>


using namespace std;


#ifndef COMMUNICATION_SOCKETCOMMUNICATION_H_
#define COMMUNICATION_SOCKETCOMMUNICATION_H_
#define PORT 3000
#define BUFF_SIZE 100

class SocketCommunication {


	//SocketCommunication();

public:

	SocketCommunication();
	class msg_data {

	public:

		int sender;
		int receiver;
		int prd_id;
		int suc_id;
		int ACK[8];

		msg_data();
	};

	class Node{
	public:

		int server_fd, new_socket, valread;
		int sock;
		int opt;

		struct sockaddr_in address;
		struct sockaddr_in serv_addr;
		static socklen_t addrlen;
		static socklen_t serv_addrlen;
//
//	public:

		bool token;
		msg_data mes_data;
		string ip_to_connect, client_ip ,srv_ip;
		int id, successor_id, predecessor_id;
		char buffer[BUFF_SIZE];
		msg_data msg;
		void open_socket (int port);
		void accept_client();
		void set_server_ip(string IP);
		void connect_socket(int port);
		void send_message(msg_data msg, string message);
		int receive_message();
		void receive_first_message();
		void close_socket();
		void clean_buffer();

	};

};

#endif /* COMMUNICATION_SOCKETCOMMUNICATION_H_ */
