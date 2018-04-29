
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#define PORT 3000
#define BUFF_SIZE 100
#include "SocketCommunication.h"


using namespace std;


SocketCommunication :: msg_data :: msg_data()
{
	sender = 0;
	receiver = 0;
	prd_id = 0;
	suc_id = 0;
    for(int i = 0; i < 8; i++)
    	ACK[i] = 0;
}


socklen_t SocketCommunication :: Node :: addrlen = sizeof(address);
socklen_t SocketCommunication :: Node :: serv_addrlen = sizeof(serv_addr);


//Open new socket
void SocketCommunication :: Node :: open_socket (int port)
{
    opt = 1;

    //Creating socket file descriptor
    server_fd = socket (AF_INET, SOCK_STREAM, 0);
    if (!server_fd)
    {
        cout << "Socket failed" << endl;
        return;
    }

    //Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        cout << "Setsockopt failed" << endl;
        return;
    }

    address.sin_family = AF_INET; //IPv4 protocol
    address.sin_addr.s_addr = INADDR_ANY; //Accept any ip address
    address.sin_port = htons(port); //Choose port for the socket

    //Forcefully attaching socket to the port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        cout << "Bind failed" << endl;
        return;
    }

    //Wait for max of 7 clients to connect
    if (listen(server_fd, 7) < 0)
    {
        cout << "Listen failed" << endl;
        return;
    }

    cout << "Socket opening success, waiting for clients" << endl;

}

//Close existing socket
void SocketCommunication :: Node :: close_socket()
{
    //Close socket file descriptor
    close(server_fd);
}


//Wait for a new client
void SocketCommunication :: Node :: accept_client()
{
    //Accept new client
    new_socket = accept(server_fd, (struct sockaddr *) &address, (socklen_t*) &addrlen);
    if (new_socket < 0)
    {
        cout << "Accept failed" << endl;
        return;
    }

    //Get client's ip address
    char* check = inet_ntoa (address.sin_addr);
    client_ip = check;

    cout << "Client's ip: " << client_ip << endl;

    cout << "Client accepted" << endl;
}


//Set server ip to connect
void SocketCommunication :: Node :: set_server_ip(const string IP)
{
	std::string
	strcpy(srv_ip ,IP);
   // srv_ip = IP;
    cout << srv_ip << endl;
    return;
}


//Connect to existing socket
void SocketCommunication :: Node :: connect_socket(int port)
{
    char* ip_p;
    ip_p = &srv_ip[0];

    //Creating socket file descriptor
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        cout << "Socket failed" << endl;
        return;
    }

    serv_addr.sin_family = AF_INET; //IPv4 protocol
    serv_addr.sin_port = htons(port); //Port to connect

    // Convert address from text to binary form
    if(inet_pton(AF_INET, ip_p, &serv_addr.sin_addr) <= 0)
    {
        cout << "Address not supported" << endl;
        return;
    }

    //Connect the server's socket
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        cout << "Connection Failed" << endl;
        return;
    }

    cout << "Connection success" << endl;
}



//Sent a new message
void SocketCommunication :: Node :: send_message(msg_data data, string message)
{
    char* ms_tmp;
    ms_tmp = &message[0];

    //Send message's data to the client connected to the node
    valread = sendto(new_socket , &data , sizeof(msg_data) , 0, (struct sockaddr *) &address, addrlen);

    if (valread < 0)
    {
        cout << "sendto error" << endl;
        return;
    }

    cout << "Message sent" << endl;

    //Send message (string) to the client connected to the node
    valread = send(new_socket, ms_tmp, strlen(ms_tmp), 0);

    if (valread < 0)
    {
        cout << "send error" << endl;
        return;
    }
}


//Receive a new message
int SocketCommunication :: Node :: receive_message()
{
    clean_buffer();

    //Receive message's data from the server the node connected to
    valread = recvfrom(sock , &msg, sizeof(msg_data), 0, (struct sockaddr *)&serv_addr, &serv_addrlen);
    if (valread < 0)
    {
        cout << "recvfrom error" << endl;
        return -1;
    }

    //Receive message from the server the node connected to
    valread = recvfrom(sock , buffer, 256, 0, (struct sockaddr *)&serv_addr, &serv_addrlen);

    if (valread < 0)
    {
        cout << "recvfrom error" << endl;
        return -1;
    }

    //In case that the message sent from this source
    if(msg.sender == id)
    {
        return 3;
    }

    //In case of broadcast message
    else if(msg.receiver == 10)
    {
        return 5;
    }

    //In case that the message is the token
    else if(strcmp(buffer, "token") == 0)
    {
        return 4;
    }

    //In case that the message is destined for itself
    else if(msg.receiver == id)
    {
        return 2;
    }


    //In case that the message is destined for another node
    else if(msg.receiver != id)
    {
    	return 1;
    }

    //Print the message that received to the screen
    cout << buffer << endl;

}

//Receive data of the node's id, successor's id, successor's ip and predecessor's id and set it's variables
void SocketCommunication :: Node :: receive_first_message()
{
    clean_buffer();

    //Receive message's data from the server the active monitor
    valread = recvfrom(sock , &msg, sizeof(msg_data), 0, (struct sockaddr *)&serv_addr, &serv_addrlen);
    if (valread < 0)
    {
        cout << "recvfrom error" << endl;
        return;
    }

    //Initialize id of the node and it's successor's and predecessor's id
    id = msg.receiver;
    predecessor_id = msg.prd_id;
    successor_id = msg.suc_id;

    cout << "Your id is: " << id << endl;
    cout << "Your predecessor id is: " << predecessor_id << endl;
    cout << "Your successor id is: "<< successor_id << endl;

    //Receive ip address of the node's predecessor from the active monitor
    valread = recvfrom(sock , buffer, 256, 0, (struct sockaddr *)&serv_addr, &serv_addrlen);
    if (valread < 0)
    {
        cout << "recvfrom error" << endl;
        return;
    }

    if(strcmp(buffer, "AM") != 0)
    {
        srv_ip = buffer;
    }

    cout << "Your predecessor ip is: " << srv_ip << endl;

}

//Initialize the buffer with NULL's
void SocketCommunication :: Node :: clean_buffer()
{
    for(int i = 0; i < BUFF_SIZE; i++)
    {
        buffer[i] = '\0';
    }
}


SocketCommunication :: SocketCommunication()
{
    /*************************************/
    /****Connection Establishment Phase***/
    /*************************************/

    Node node;
    msg_data data;
    string message;

    cout << "Are you the active monitor? (y/n)" << endl;
    char answer;
    cin >> answer;

    //If you are the active monitor this program runs
    if(answer == 'y')
    {

        //The active monitor chooses quantity of nodes in the token ring
        cout << "Please choose number of nodes to connect" << endl;
        int node_quan;
        cin >> node_quan;

        //Initialize AM data inside the ring
        node.id = 0;
        node.successor_id = 1;
        node.predecessor_id = node_quan - 1;

        //Open socket to connect to all nodes
        node.open_socket(PORT);

        //Connect to each node and send it the data inside the ring
        for(int i = 1; i < node_quan; i++)
        {
            //Wait for node to connect
            node.accept_client();

            if(i == 1)
            {
                //The first node to connect is always the predecessor of the active monitor
                message = "AM";
            }
            else
            {
                //Get the last node that connected ip and sent it to the next one to be its successor's ip
                message = node.ip_to_connect;
            }

            //Enter data to send to the node
            data.sender = 0;
            data.receiver = i;
            data.suc_id = i + 1;
            data.prd_id = i - 1;

            if( i == node_quan - 1)
            {
                data.suc_id = 0;
            }

            //Send to the connected node all the relevant data
            node.send_message(data, message);

            //Next node's ip to connect is the current node in the loop
            node.ip_to_connect = node.client_ip;

        }

        //The AM successor's ip is the last node connected
        node.set_server_ip(node.ip_to_connect);

        //The AM is the firt to hold the token
        node.token = true;

        cout << "Enter char to connect" << endl;
        cin >> answer;

        //Close the active monitor's socket
        node.close_socket();

        //Connect AM to his predecessor and successor
        node.open_socket(PORT + node.id);
        node.connect_socket(PORT + node.predecessor_id);
        node.accept_client();

    }

    //If you are a simple node this program runs
    else if(answer == 'n')
    {
        string am_ip;

        //Enter the active monitor ip in order to connect it
        cout << "Set active monitor ip" << endl;
        cin >> am_ip;
        node.set_server_ip(am_ip);

        //Connect the active monitor
        node.connect_socket(PORT);

        //Receive the node's data and set it to it's variables
        node.receive_first_message();

        //Connect between all nodes
        node.open_socket(PORT + node.id);
        node.accept_client();
        node.connect_socket(PORT + node.predecessor_id);

        //The regular nodes doesn't hold the token first
        node.token = false;
    }

    /*************************************/
    /***********Token Ring Phase**********/
    /*************************************/

    //The first to hold the token cand send a message
    if(node.token)
    {
        cout << "Do you have a message to send? (y/n)" << endl;
        cin >> answer;

        //In case the node have a message to send
        if(answer == 'y')
        {
            cout << "Enter your message" << endl;
            getline(cin, message);
            getline(cin, message);
            cout << "Enter the id of the node you want to send the message (broadcast = 10)" << endl;
            cin >> data.receiver;
            data.sender = node.id;
            node.send_message(data, message);
        }

        //If the node doesn't have any message, he send the token to it's successor
        else if(answer == 'n')
        {
            data.receiver = node.successor_id;
            data.sender = node.id;
            node.send_message(data, "token");
            node.token = false;
        }
    }

    int msg_check = 0;

    while(true)
    {
        //Wait for new message
        msg_check = node.receive_message();

        //Check the message position
        switch(msg_check)
        {
            //Receiving a message which is destined for another node
            case 1:
                node.send_message(node.msg, node.buffer);
                break;

            //Receiving a message which is destined for itself
            case 2:
                cout << "Message received: " << node.buffer << endl;
                node.msg.ACK[node.id] = 1;
                node.send_message(node.msg, node.buffer);
                break;

            //Receiving a message for which it is the source
            case 3:
                //Print the nodes that ACKed the message
                for(int i = 0; i < 8; i++)
                {
                    if(node.msg.ACK[i] == 1)
                    {
                        cout << "node number " << i << " got the message" << endl;
                        node.msg.ACK[i] = 0;
                    }
                }

                //Send the token to the successor
                data.receiver = node.successor_id;
                data.sender = node.id;
                node.send_message(data, "token");
                node.token = false;
                break;

            //Receiving the token
            case 4:
                node.token = true;
                cout << "Do you have a message to send? (y/n)" << endl;
                cin >> answer;

                //In case the node have a message to send
                if(answer == 'y')
                {
                    cout << "Enter your message" << endl;
                    getline(cin, message);
                    getline(cin, message);
                    cout << "Enter the id of the node you want to send the message (broadcast = 10)" << endl;
                    cin >> data.receiver;
                    data.sender = node.id;
                    node.send_message(data, message);
                }

                //If the node doesn't have any message, he send the token to it's successor
                else if(answer == 'n')
                {
                    data.receiver = node.successor_id;
                    data.sender = node.id;
                    node.send_message(data, "token");
                    node.token = false;
                }
                break;

            //A broadcast message received
            case 5:
                node.msg.ACK[node.id] = 1;
                node.send_message(node.msg, node.buffer);
                break;

            case -1:
                break;
        }

    }
}
