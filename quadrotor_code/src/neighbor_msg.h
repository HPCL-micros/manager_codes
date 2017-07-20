#ifndef NEIGHBOR_MSG
#define NEIGHBOR_MSG
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>
struct NeighborInfo{
    int _r_id;
    float _px;
    float _py;
    float _vx;
    float _vy;
    float _xx;
    float _yy;
    float _zz;
    float _ww;
    
    template<class Archive>
    void serialize(Archive &ar,const unsigned int version)
    {
        ar & _r_id;
        ar & _px;
        ar & _py;
        ar & _vx;
        ar & _vy;
        ar & _xx;
        ar & _yy;
        ar & _zz;
        ar & _ww;
    }
    
    NeighborInfo(int r_id,float px,float py,float vx,float vy,float xx=0,float yy=0,float zz=0,float ww=1 )
    {
        _r_id=r_id;
        _px=px;
        _py=py;
        _vx=vx;
        _vy=vy;
        _xx=xx;
        _yy=yy;
        _zz=zz;
        _ww=ww;
    }
    
    NeighborInfo()
    {
        _r_id=-1;
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _xx=0;
        _yy=0;
        _zz=0;
        _ww=1;
    }
};
#endif
