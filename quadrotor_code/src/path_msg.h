#ifndef PATH_MESSAGE_H_
#define PATH_MESSAGE_H_

#include <iostream>
#include <time.h>
#include <fstream>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>
struct PathMessage{
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
        
        std::vector<float> ox;
        std::vector<float> oy;
        std::vector<float> oz;
        std::vector<float> ow;
        
        float arrival_time;
        int formation;    
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & px;
            ar & py;
            ar & pz;
            ar & oz;
            ar & oy;
            ar & oz;
            ar & ow;
            ar & arrival_time;
            ar & formation;
        }
            
        PathMessage(){}
        
    };
    

#endif
