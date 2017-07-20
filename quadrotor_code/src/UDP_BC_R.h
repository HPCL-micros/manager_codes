/* -*- c++ -*- */
/*
* Copyright 2017 micROS Group - FlyNet Project
* This file is part of micROS
* Dr.Bo Zhang, State Key Laboratory of High Performance Computing, National University of Defense Technology
* 
*/

/*
* Just include the "UDP_BC.h" and have fun with the UDP_BC class.
* in this version, it provides:
* 1. BC_type = 0: 255.255.255.255 based local broadcast
* 2. BC_type = 1: AAA.BBB.CCC.255 based global broadcast within a single subnet AAA.BBB.CCC
* 3. BC_type = 2: P2P (multihop) communication, need routing-table support from external program, e.g. olsrd 
* 
* Demo:The UDP_BC_Server.cpp and UDP_BC_Client.cpp are the demos of easy-to-use UDP_BC object and methods.

* !!!!remember: Because the Linux Kernel disables all-255 broadcast, we should add it to the route table for enabling reception.
* In the command window or your shell file for network configuration
* add the following cmd, it should be fine:

* $sudo route add 255.255.255.255 dev wlan0

* Then type in $route to check the result, you should find something like this:
* Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
* default         192.168.253.1   0.0.0.0         UG    0      0        0 wlan0
* 255.255.255.255 *               255.255.255.255 UH    0      0        0 wlan0

* Next Version Preview:
* 1. Pure 255.255.255.255 based flooding
*/

/*
forget "\0" from stream to char: 1.0
malloc without freespace: 1.0 
cannot handle space in payloadstream: 1.0
header file reinclude: 0.5
forget return: 0.5
*/

#ifndef UDP_BC_H_
#define UDP_BC_H_

#include <iostream>  
#include <fstream>
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
#include <sys/time.h>
#include "IP_ToolBox.h"

using namespace std;

//* DEBUG_FLAG
//* IF Defined: Print Monitoring information on the screen
//#define DEBUG_FLAG 0  

//* DONT_Listen_to_other_subnet:
//* IF Defined: Only Process and Push to String
//* if NOT Defined, may be interfered by Broadcast from other Subnet
#define DONT_Listen_to_other_subnet 0

//* NET_FULL_RECORD:
//* IF Defined: Record the following information for analyzing Delay & PER
//* Source_node_IP Send_packet_index Send_Timestamp Recv_Timestamp Msg_Type
#define NET_FULL_RECORD 0

#define MAX_UDP_SIZE 10000//Maximum UDP_Size is 65536 - Overhead, So MAX_UDP_SIZE<60000 is Safe

class UDP_BC
{
    public:
        UDP_BC(int BC_type_init,string HOST_IP_init,int PORT_init);
        int bind_socket();
        int UDP_BC_send(string msg_type, string payload_data_send);
        int UDP_BC_recv(string &send_addr, int &msg_type, string &payload_data_recv);

        int sock;
        int PORT;
        int BC_type;
        //0: local BC
        //1: global BC in a subnet
        //2: P2P (multihop) communication, need routing-table support from external program, e.g. olsrd 
        string network;
        string parity;
        string HOST;

        struct sockaddr_in addrto;  
        int nlen;
        int packet_index_send;
        int packet_index_recv;

#ifdef NET_FULL_RECORD
        string log_filename_str;
#endif
        
};

UDP_BC::UDP_BC(int BC_type_init,string HOST_IP_init,int PORT_init)
{   
    parity = "001100";
    BC_type = BC_type_init;

    HOST = HOST_IP_init;
    PORT = PORT_init;

    //set up sendto address
    string IP_addrto;
    
    if(BC_type == 0)
        IP_addrto = "255.255.255.255";
    else if(BC_type == 1)
    {
        //find the global BC address
        stringstream IP_addrto_ss;
        size_t found_last_dot = HOST.find_last_of(".");
        IP_addrto_ss << HOST.substr(0,found_last_dot);
        IP_addrto_ss << ".255";
        IP_addrto_ss >> IP_addrto;
        //cout<<IP_addrto<<endl;
    }     
    else 
        IP_addrto = "255.255.255.255";

    const char* IP_addrto_char = IP_addrto.data();

#ifdef NET_FULL_RECORD
    ofstream outfile;
    stringstream file_name_ss;
    string filename_str;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    int time_file_record = tv.tv_sec;//sec

    file_name_ss << "net_full_record_at_" << HOST_IP_init << "_"<< PORT_init<<"_"<<time_file_record<<".log";
    file_name_ss >> log_filename_str;
#endif

    //
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
    addrto_init.sin_addr.s_addr=inet_addr(IP_addrto_char);//LB
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
    stringstream ss;
    ss << packet_index_send << " ";
    ss << parity << " ";//add parity bits for middle-ware or application-layer packet parser

#ifdef NET_FULL_RECORD
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long time_send = tv.tv_sec*1000 + tv.tv_usec/1000;//ms
    ss << time_send << " " << msg_type << " ";//add network performance record probe
#endif
    ss << payload_data_send; //add the payload data to the end of the string

    string data_stream = ss.str();
    
    int str_length = data_stream.length();
    
    const char* data = data_stream.data();
    //correct this part using string to char* conversion
    /*
    char * data;
    data = (char*)malloc((str_length+1)*sizeof(char));
    data_stream.copy(data,str_length,0); 
    data[str_length] = '\0';//
    */

    int ret=sendto(sock, data, strlen(data), 0, (sockaddr*)&addrto, nlen);

    if(ret<0)  
    {  
        cout<<"send error...."<<ret<<endl;  
        return -1;
    }  
    else  
    {  
#ifdef DEBUG_FLAG         
        cout<<data<<endl;   
#endif
    }  
    packet_index_send += 1;
    return 0;
}


int UDP_BC::UDP_BC_recv(string &send_addr, int &msg_type, string &payload_data_recv)
{

#ifdef DEBUG_FLAG   
    clock_t tic0 = clock();
#endif
    //payload_data_recv[MAX_UDP_SIZE] = {0};
    char recv_data[MAX_UDP_SIZE] = {0};
    stringstream ss;

    struct sockaddr_in from;  
    bzero(&from, sizeof(struct sockaddr_in));  
    from.sin_family = AF_INET;  
    from.sin_addr.s_addr = htonl(INADDR_ANY);  
    from.sin_port = htons(PORT); 

    while(true)  
    {  
        //receive from the broadcast address 
        int ret=recvfrom(sock, recv_data, MAX_UDP_SIZE, 0, (struct sockaddr*)&from,(socklen_t*)&nlen);  
        if(ret<=0)  
        {  
#ifdef DEBUG_FLAG 
            cout<<"read error...."<<sock<<endl;
#endif
            return -1;  
        }  
        else  
        {  
            stringstream recv_data_full;
            recv_data_full << recv_data;

            //read Source_IP
            char *Source_IP = inet_ntoa(from.sin_addr);
            stringstream Source_IP_strstr;
            string Source_IP_str;
            Source_IP_strstr << Source_IP;
            Source_IP_strstr >> Source_IP_str;

            send_addr = Source_IP_str;

#ifdef DEBUG_FLAG 
            cout<<"Source_IP = "<<Source_IP_str;
            cout<<", Self_IP = "<<HOST<<endl;
#endif
            string payload_data_recv_temp;//if parity is not correct, discard.
            string parity_record;
            int packet_index_send_record = 0;
            int success_flag = 0;

#ifdef NET_FULL_RECORD                        
     
            long long send_time_record;

            recv_data_full >> packet_index_send_record;
            recv_data_full >> parity_record; 
            recv_data_full >> send_time_record;
            recv_data_full >> msg_type;  
            //recv_data_full >> payload_data_recv_temp;//output payload_data_recv anyway, not knowing its correct or not
            getline(recv_data_full, payload_data_recv_temp);
            payload_data_recv_temp = payload_data_recv_temp.substr(1, payload_data_recv_temp.length());
            
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long receive_time_record = tv.tv_sec*1000 + tv.tv_usec/1000;
           // cout<<"rcv_string="<<recv_data_full.str()<<endl;
/*
            cout<<"From Node: "<<inet_ntoa(from.sin_addr)<<";send_packet_index = ";
            cout<<packet_index_send_record<<";send_time = "<<send_time_record;
           // cout<<";recv_time = "<<double(clock());*/
            //cout<<"parity_record = "<<parity_record<<";payload = "<<payload_data_recv<<endl;

            //write to log file    
            const char* net_full_record_file = log_filename_str.data();
            ofstream outfile;
            outfile.open(net_full_record_file, std::ofstream::out | std::ofstream::app);  
            //cout<<"File_name = "<<net_full_record_file<<endl;
            outfile<<inet_ntoa(from.sin_addr)<<" ";
            outfile<<packet_index_send_record<<" "<<send_time_record<<" "<<receive_time_record<<" ";
            outfile<<msg_type<<endl;
            outfile.close();

#else
            //cout<<recv_data_full<<endl;
            recv_data_full >> packet_index_send_record;
            recv_data_full >> parity_record;

            //output the rest of the stringstream as payload
            getline(recv_data_full, payload_data_recv_temp);
            payload_data_recv_temp = payload_data_recv_temp.substr(1, payload_data_recv_temp.length());
            //payload_data_recv_temp = recv_data_full.str();
            //cout<<parity_record<<endl;
#endif

            int check_subnet_flag = 1;

#ifdef DONT_Listen_to_other_subnet
            //compare the Source_IP and Self_IP to check whether in the same subnet
            check_subnet_flag = Check_Subnet(Source_IP_str, HOST);
#endif       
            if(parity == parity_record && check_subnet_flag == 1)
            {
                payload_data_recv = payload_data_recv_temp;
            
#ifdef DEBUG_FLAG    
                clock_t tic1 = clock();
                cout<<payload_data_recv<<endl; 
                //cout<<packet_index_send_record<<" "<<packet_index_recv<<endl;
                //cout<<"processing_delay = "<<tic1-tic0<<endl;//processing delay measure in ms 
#endif  
                packet_index_recv += 1;
                return 0;
            }
            else if (parity == parity_record && check_subnet_flag == 0)
            {
#ifdef DEBUG_FLAG  
                cout<<"packet correct but not from the same subnet"<<endl;
#endif  
                return -1;
            }
            
            else
            {
#ifdef DEBUG_FLAG  
                cout<<parity_record<<endl;
                cout<<"packet error"<<endl;
#endif  
                return -2;
            }
        }  
    }  
}

#endif
