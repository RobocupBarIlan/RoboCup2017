

#include <iostream>

#include <string.h>

#include <sys/socket.h>

#include <netinet/in.h>

#include <stdlib.h>

#include <stdio.h>

#include <arpa/inet.h>

#include <unistd.h>



#define PORT 3000

#define BUFF_SIZE 100



using namespace std;



class msg_data{



public:

    int sender;

    int receiver;

    int prd_id;

    int suc_id;

    int ACK[8];


};



//msg_data :: msg_data()
//
//{
//	sender= receiver = prd_id = suc_id = 0;
//
//    for(int i = 0; i < 8; i++)
//
//    {
//
//        ACK[i] = 0;
//
//    }
//
//}



class Node{

private:



    int server_fd, new_socket, valread;

    int sock;

    int opt;

    string srv_ip;

    struct sockaddr_in address;

    struct sockaddr_in serv_addr;

    static socklen_t addrlen;

    static socklen_t serv_addrlen;



public:



    bool token;

    msg_data mes_data;

    string ip_to_connect, client_ip;

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





socklen_t Node :: addrlen = sizeof(address);

socklen_t Node :: serv_addrlen = sizeof(serv_addr);



//Open new socket

void Node :: open_socket (int port)

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

void Node :: close_socket()

{

    //Close socket file descriptor

    close(server_fd);

}





//Wait for a new client

void Node :: accept_client()

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

void Node :: set_server_ip(string IP)

{

    srv_ip = IP;

    cout << srv_ip << endl;

    return;

}





//Connect to existing socket

void Node :: connect_socket(int port)

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
    }
        return;
}

