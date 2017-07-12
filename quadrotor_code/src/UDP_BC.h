#include <iostream>  
#include <stdio.h>  
#include <stdlib.h>
#include <sys/socket.h>  
#include <unistd.h>  
#include <sys/types.h>  
#include <netdb.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <string.h> 
#include <sstream> 

using namespace std;
//#define DEBUG_FLAG 0

#define MAX_UDP_SIZE 1024

class UDP_BC
{
    public:
        UDP_BC(int BC_type_init,string HOST_IP_init,int PORT_init);
        int bind_socket();
        int UDP_BC_send(string msg_type, string payload_data_send);
        int UDP_BC_recv(string &send_addr, int &msg_type, string &payload_data_recv);

        int sock;
        int PORT;
        int BC_type;//0-local BC, 1-global BC
        string network;
        string parity;
        string HOST;

        struct sockaddr_in addrto;  
        int nlen;
        int packet_index_send;
        int packet_index_recv;
};

UDP_BC::UDP_BC(int BC_type_init,string HOST_IP_init,int PORT_init)
{   
    parity = "0110";
    BC_type = BC_type_init;

    /*
    if(BC_type == 0)
        IP_addrto = "255.255.255.255";
    else if(BC_type == 1)
        IP_addrto = "192.168.3.255";
    else  
        IP_addrto = "255.255.255.255";
    */

    HOST = HOST_IP_init;
    PORT = PORT_init;

    setvbuf(stdout, NULL, _IONBF, 0);   
    fflush(stdout);  

    sock = -1;  
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)   
    {     
        cout<<"socket error"<<endl;   
        return;  
    }
    
    const int opt = 1;  
    //set the socket to broadcast type
    int nb = 0;  
    nb = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
    if(nb == -1)  
    {  
        cout<<"set socket error..."<<endl;  
        return;  
    }  

    struct sockaddr_in addrto_init;
    bzero(&addrto_init, sizeof(struct sockaddr_in));  
    addrto_init.sin_family=AF_INET;  
    addrto_init.sin_addr.s_addr=inet_addr("255.255.255.255");//LB
    addrto_init.sin_port=htons(PORT);
    addrto = addrto_init;  
    
    nlen=sizeof(addrto); 

    packet_index_send = 0;//start recording the number of packets
    packet_index_recv = 0;
}

int UDP_BC::bind_socket()
{
    if(bind(sock,(sockaddr*)&(addrto), sizeof(sockaddr_in)) == -1)   
    {     
        cout<<"bind error..."<<endl;  
        return -1;  
    }  
    return 0;
}

int UDP_BC::UDP_BC_send(string msg_type, string payload_data_send)
{
    double time = 0.0;//need a timer in c++ 
    
    stringstream ss;
    //ss << packet_index_send << "-" << time << "-" << msg_type << "-" ;//add network performance record probe
    ss << payload_data_send; 
    //ss << "-" << parity;//add parity bits for middle-ware or application-layer packet parser

    string data_stream = ss.str();
    
    int str_length = data_stream.length();
    char* data;
    data = (char*)malloc((str_length+1)*sizeof(char));
    data_stream.copy(data,str_length,0); 

    int ret=sendto(sock, data, strlen(data), 0, (sockaddr*)&addrto, nlen);

    if(ret<0)  
    {  
        cout<<"send error...."<<ret<<endl;  
    }  
    else  
    {
#ifdef DEBUG_FLAG              
        cout<<data<<endl;
#endif    
    }  
    packet_index_send += 1;
}


int UDP_BC::UDP_BC_recv(string &send_addr, int &msg_type, string &payload_data_recv)
{
    //payload_data_recv[MAX_UDP_SIZE] = {0};
    char recv_data[MAX_UDP_SIZE] = {0};
    stringstream ss;

    while(true)  
    {  
        //receive from the broadcast address 
        int ret=recvfrom(sock, recv_data, MAX_UDP_SIZE, 0, (struct sockaddr*)&addrto,(socklen_t*)&nlen);  
        if(ret<=0)  
        {  
            cout<<"read error...."<<sock<<endl;  
            return -1;
        }  
        else  
        {  
#ifdef DEBUG_FLAG       
            cout<<recv_data<<endl;  
#endif   
            payload_data_recv = recv_data;
            packet_index_recv += 1;
            return 0;
        }  
    }  
}
