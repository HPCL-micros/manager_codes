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
            ar & ox;
            ar & oy;
            ar & oz;
            ar & ow;
            ar & arrival_time;
            ar & formation;
        }
            
        PathMessage(){}
        PathMessage(std::vector<float> vpx,std::vector<float> vpy, std::vector<float> vpz, std::vector<float> vox,std::vector<float>voy,std::vector<float> voz,std::vector<float> vow,float vtime,float vf):px(vpx),py(vpy),pz(vpz),ox(vox),oy(voy),oz(voz),ow(vow),arrival_time(vtime),formation(vf){}
        
    };
    

#endif
