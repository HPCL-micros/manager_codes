/* -*- c++ -*- */
/*
* Copyright 2017 micROS Group - FlyNet Project
* This file is part of micROS
* Dr.Bo Zhang, State Key Laboratory of High Performance Computing, National University of Defense Technology
* 
*/

/*
* This is toolbox is made for handling those tedious string-type IP and whatever.
* Let us save the DAY.
*/

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
#include <time.h>
#include <vector>

#define IP_Version 4

using namespace std;


int IPaddr_to_Vec(string IP_addr, vector<int> &IP_sub_addr_vec);
//Convert string-type IP-address IP_addr to int-vector as IP_sub_addr_vec
int Check_Subnet(string IP_addr_A, string IP_addr_B);
//Check whether IP_addr_A and IP_addr_B are in the same subnet
//return: 1:same 0:not the same


int IPaddr_to_Vec(string IP_addr, vector<int> &IP_sub_addr_vec)
//Convert string-type IP-address IP_addr to int-vector as IP_sub_addr_vec
{
    string IP_addr_temp = IP_addr;
    size_t find_dot = IP_addr_temp.find_first_of(".");

    if(find_dot == IP_addr_temp.npos)
        return -1;

    while (find_dot != IP_addr_temp.npos)
    {
        IP_addr_temp[find_dot] = ' ';
        find_dot = IP_addr_temp.find_first_of(".",find_dot+1);
    }

    stringstream ss;
    ss << IP_addr_temp;
    
    //cout << IP_addr_temp <<endl;

    int temp_int = 0;
    vector<int>::iterator iter = IP_sub_addr_vec.begin();
    int counter = 0;

    for(counter = 0; counter < IP_Version; counter++)
    {
        ss >> temp_int;
        IP_sub_addr_vec.push_back(temp_int);
        //iter++;
        //cout<<*iter<<endl;
    }

    return 0;
}

int Check_Subnet(string IP_addr_A, string IP_addr_B)
//Check whether IP_addr_A and IP_addr_B are in the same subnet
//return: 1:same 0:not the same
{
    int length_addr = IP_Version;
    vector<int> IP_int_vec_A;
    vector<int> IP_int_vec_B;

    IPaddr_to_Vec(IP_addr_A, IP_int_vec_A);
    IPaddr_to_Vec(IP_addr_B, IP_int_vec_B);

    for(int counter = 0; counter < length_addr - 1; counter++)//check all subnet except the last one
    {   
        //cout<<IP_int_vec_A[counter]<<" vs "<<IP_int_vec_B[counter]<<endl;
        if(IP_int_vec_A[counter] != IP_int_vec_B[counter])
            return 0;// not the same subnet
    }
    return 1;
}