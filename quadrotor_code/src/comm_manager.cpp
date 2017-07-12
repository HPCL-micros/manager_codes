#define TF_EULER_DEFAULT_ZYX

#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "decide_softbus_msgs/NavigationPoint.h"
#include "decide_softbus_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "decide_softbus_msgs/SetControlling.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"
#include "quadrotor_code/Status.h"
#include "quadrotor_code/Neighbor.h"
#include "std_msgs/Time.h"
#include "std_msgs/Empty.h"
#include <geodesy/utm.h>
#include "sensor_msgs/NavSatFix.h"
#include <tf/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <fstream>
#include <math.h>
#include "action_softbus/neighbor_msg.h"
#include "path_msg.h"
#include <boost/thread/thread.hpp> 
#include "UDP_BC.h"
using namespace std;
#define R 7
#define sendT 1
#define perror 0  //value
#define verror 0 //rate
double PI=acos(-1);
double stopdist = 1.5;

int line_uav_num=4;  //每一行摆放的飞机的数量
double delta_dis=3; //无人机之间的间距
int stand_vx=0, stand_vy=0;//作为标准的无人机的虚拟坐标
double stand_x, stand_y;//作为标准的无人机的实际坐标

int BC_type_init = 0;
string HOST_IP_init = "192.168.21.200";
int PORT_init = 1070;
   
UDP_BC UDP_BC_test(BC_type_init,HOST_IP_init,PORT_init);

/*ofstream vel_out("/home/czx/vel.txt");
bool odom_first=true;
std_msgs::Time odom_start_time;*/

class OdomHandle
{
    public:
    boost::mutex mut;
    ros::Subscriber sub_yaw,sub_vel,sub_fix, sub_odom;
    ros::Publisher pub;
    ros::Publisher neighbor_pub;
    ros::Publisher stop_pub;
    ros::Publisher fix_odom_pub;
    double odomx,odomy,odomz,odomtheta;
    double _px,_py,_vx,_vy;
    double _pz,_vz,_theta;
    int _r_id; 
    bool yawrcv,velrcv,fixrcv;
    
    double start_x,start_y;
    int vx,vy;
    double delta_x,delta_y;
    int count;
    
    double xx,yy,zz,ww;
   
    OdomHandle(int r_id)
    {
        ros::NodeHandle n;
        //double xerror,yerror;
        yawrcv = false;
        velrcv = false;
        fixrcv = false;
        stringstream ss;
        ss<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/AttitudeChanged";
        //sub_yaw = n.subscribe(ss.str(), 1000, &OdomHandle::yawcb,this);
 
        stringstream ss1;
        //ss1<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/SpeedChanged";
        ss1<<"/uav"<<r_id<<"/velctrl/input";
        //sub_vel = n.subscribe(ss1.str(), 1000, &OdomHandle::velcb2,this);

        stringstream ss2;
        ss2<<"/uav"<<r_id<<"/fix";
        ////sub_fix = n.subscribe(ss2.str(), 1000, &OdomHandle::fixcb,this);
        
        stringstream ss3;
        ss3<<"/uav"<<r_id<<"/position";
        pub = n.advertise<quadrotor_code::Status>(ss3.str(),1000);
        
        stringstream ss4;
        ss4<<"/uav"<<r_id<<"/neighbor";
        neighbor_pub = n.advertise<quadrotor_code::Neighbor>(ss4.str(),1000);
        
        stringstream ss5;
        ss5<<"/uav"<<r_id<<"/stop";
        //stop_pub = n.advertise<std_msgs::Int32>(ss5.str(),1000);
        stop_pub = n.advertise<std_msgs::Int32>("/stop",1000);
        
        stringstream ss6;
        ss6<<"/uav"<<r_id<<"/fix_odom";
        fix_odom_pub = n.advertise<nav_msgs::Odometry>(ss6.str(),1000);
        
        stringstream ss7;
        ss7<<"/uav"<<r_id<<"/odom";
        //sub_odom = n.subscribe(ss7.str(), 1000, &OdomHandle::odomcb,this);
        
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _pz=0;
        _vz=0;
        _r_id = r_id;
        vx=r_id/line_uav_num;  //计算摆放位置的虚拟坐标
        vy=r_id%line_uav_num;
        
        odomx = 0;//vx * delta_dis;
        odomy = 0;//-vy * delta_dis;
        odomz = 0;
        
        odomtheta=0;
        count = 0;
        
        ww=1;
        xx=0;yy=0;zz=0;ww=1;
      
    }
    //NEU -> NWU
    void yawcb(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr & msg)
    {
        double msgtheta = msg->yaw;
        _theta = -msgtheta;
        if(count == 0)
        {
            odomtheta = _theta;
        }
        count++;
        /*
        if (msgtheta >= 0)
        {
             _theta = PI/2 - msgtheta;
        }
        else
        {
            if(msgtheta > -PI/2)
            {
                _theta = PI/2 - msgtheta;
            }
            else
            {
                _theta = - 3*PI/2 -msgtheta;
            }
        }*/
        yawrcv = true;
    }

    void velcb(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr & msg)
    {
        _vx = msg->speedX;
        _vy = -msg->speedY;//NEU -> NWU
        velrcv = true;
    }
    
    void velcb2(const geometry_msgs::Twist::ConstPtr & msg)
    {
        _vx = msg->linear.x;
        _vy = -msg->linear.y;//NEU -> NWU
        velrcv = true;
    }
    /*
    void fixcb(const sensor_msgs::NavSatFix::ConstPtr & msg)
    {
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = msg->latitude;
        geo_pt.longitude = msg->longitude;
        geo_pt.altitude = msg->altitude;
        geodesy::UTMPoint utm_pt(geo_pt);
        _py = utm_pt.easting;
        _px = utm_pt.northing;
        
        if(count<10)
        {
            start_x=utm_pt.northing;
            start_y=utm_pt.easting;

            if(vx==stand_vx && vy==stand_vy && count==5)
            {
                stand_x=start_x;
                stand_y=start_y;
            }

            if(count==9)
            {
                delta_x=((vx-stand_vx)*delta_dis+stand_x)-start_x;
                delta_y=((vy-stand_vy)*delta_dis+stand_y)-start_y;
                cout<<"uav"<<_r_id<<" aligned"<<endl;
            }

            count++;
        }
        else
        {
           _py = _py+delta_y - stand_y;
           _px = _px+delta_x - stand_x;
        }
        
        fixrcv = true;
        _py = - _py;//NEU -> NWU
    }
    */
    void odomcb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        /*if(odom_first)
        {
            odom_start_time.data=msg->header.stamp;
            odom_first=false;
        }*/
        
        _px= msg->pose.pose.position.x+ odomx;
        _py= msg->pose.pose.position.y+ odomy;
        //cout<<vx<<' '<<vy<<' '<<_px<<' '<<_py<<endl;
        
        
        xx=msg->pose.pose.orientation.x;
        yy=msg->pose.pose.orientation.y;
        zz=msg->pose.pose.orientation.z;
        ww=msg->pose.pose.orientation.w;
        double tmp_vx=msg->twist.twist.linear.x;
        double tmp_vy=msg->twist.twist.linear.y;
       //vel_out<<msg->header.stamp-odom_start_time.data<<" "<<tmp_vx<<" "<<tmp_vy<<" "<<sqrt(tmp_vx*tmp_vx+tmp_vy*tmp_vy)<<endl;
    }
};

static vector<OdomHandle*> odom_list;
int robotnum=50;
vector<vector<int> > adj_list;

void rcv_thread()
{
    cout<<"0000000000"<<endl;
    int BC_type_init = 0;
    string HOST_IP_init = "192.168.21.200";
    int PORT_init = 1070;
   
    UDP_BC UDP_BC_test(BC_type_init,HOST_IP_init,PORT_init);
    UDP_BC_test.bind_socket();
    string send_address;
    string recv_data;
    int msg_type;
    //socket_init
   ros::Rate loop_rate(10);
   int count = 0;
    while(ros::ok())//ToDo
    {
        //loop_rate.sleep();
        //cout<<"000000"<<endl;
        int rcvok=UDP_BC_test.UDP_BC_recv(send_address,msg_type,recv_data);
        cout<<count++<<endl;
        if(rcvok==-1)
        {
            cout<<"[ERRROR]udp rcv failed"<<endl;
            continue;
        }
        string fullstring=recv_data;//rcv string
        //cout<<"111111111111"<<endl;
        if(fullstring.length()<1 || fullstring[0]!='1')
        {
            //loop_rate.sleep();
            continue;
        }
        //cout<<"2222222222"<<endl;
        string istring=fullstring.substr(1);
        istringstream iarchiveStream(istring);
        boost::archive::text_iarchive iarchive(iarchiveStream);
        NeighborInfo ni_new;
        iarchive>>ni_new;
        int id=ni_new._r_id;
        double px=ni_new._px;
        double py=ni_new._py;
        double vx=ni_new._vx;
        double vy=ni_new._vy;            
            
        geometry_msgs::Quaternion q;
        q.x=ni_new._xx;q.y=ni_new._yy;q.z=ni_new._zz;q.w=ni_new._ww;
        cout<<id<<' '<<px<<' '<<py<<' '<<tf::getYaw(q)<<endl;
        //cout<<"11111111111"<<endl;
        if(id>=0)
        {
                odom_list[id]->mut.lock();
                odom_list[id]->_px=px;
                odom_list[id]->_py=py;
                odom_list[id]->xx=ni_new._xx;
                odom_list[id]->yy=ni_new._yy;
                odom_list[id]->zz=ni_new._zz;
                odom_list[id]->ww=ni_new._ww;
                odom_list[id]->mut.unlock();        
        }
        
            //sleep?
        //cout<<"2222222222"<<endl;
            
            //cout<<"rcv count="<<count++<<endl;
    }
};

void virtual_rcv(const std_msgs::String::ConstPtr & msg)
{
    string fullstring=msg->data;
    if(fullstring[0]!='1')
        return;
    string istring=fullstring.substr(1);
    istringstream iarchiveStream(istring);
    boost::archive::text_iarchive iarchive(iarchiveStream);
    NeighborInfo ni_new;
    iarchive>>ni_new;
    int id=ni_new._r_id;
    double px=ni_new._px;
    double py=ni_new._py;
    double vx=ni_new._vx;
    double vy=ni_new._vy;            
            
    cout<<id<<' '<<px<<' '<<py<<endl;
    if(ni_new._r_id>=0)
    {
        odom_list[id]->mut.lock();
        odom_list[id]->_px=px;
        odom_list[id]->_py=py;
        odom_list[id]->xx=ni_new._xx;
        odom_list[id]->xx=ni_new._yy;
        odom_list[id]->xx=ni_new._zz;
        odom_list[id]->xx=ni_new._ww;
        odom_list[id]->mut.unlock();        
    }

}

double dist(int i,int j)
{
    double re=pow(odom_list[i]->_px-odom_list[j]->_px,2)+pow(odom_list[i]->_py-odom_list[j]->_py,2);
    //if(i==4)
    //cout<<re<<endl;
    return sqrt(re);
    
}

ros::Publisher rviz_goal_pub_;

void rvizGoalCb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    decide_softbus_msgs::NavigationPoint tmp_goal;
    tmp_goal.header.seq=msg->header.seq;
    tmp_goal.header.stamp=msg->header.stamp;
    tmp_goal.header.frame_id=msg->header.frame_id;
            
    tmp_goal.pose.position.x=msg->pose.position.x;
    tmp_goal.pose.position.y=msg->pose.position.y;
    tmp_goal.pose.position.z=msg->pose.position.z;
            
    tmp_goal.pose.orientation.x=msg->pose.orientation.x;
    tmp_goal.pose.orientation.y=msg->pose.orientation.y;
    tmp_goal.pose.orientation.z=msg->pose.orientation.z;
    tmp_goal.pose.orientation.w=msg->pose.orientation.w;
    
    rviz_goal_pub_.publish(tmp_goal);
}

ros::ServiceClient rviz_start_action_client_;

ros::Publisher ActivatePub;
string status="0";//control status,0 for land, 1 for takeoff, 2 for activate
void rvizStartActionCb(const geometry_msgs::PointStamped::ConstPtr & msg)
{
    status="2";
    //std_msgs::String sendstring;
    //sendstring.data="0"+string("2");
    //ActivatePub.publish(sendstring);
    /*
    decide_softbus_msgs::SetControlling srv;
    srv.request.CONTROLLING = 1;
    if (rviz_start_action_client_.call(srv))
    {
        ROS_INFO("start action...");
    }
    else
    {
        ROS_ERROR("Failed to call service: /uav0/move_base/set_controlling");
    }*/
}

void activateCb(const std_msgs::Empty::ConstPtr & msg)
{
    status="2";
}
void takeoffCb(const std_msgs::Empty::ConstPtr & msg)
{
    status="1";
}
void landCb(const std_msgs::Empty::ConstPtr & msg)
{
    status="0";
}
ros::Publisher path_pub_virtual;
void swarmPlanCB(const decide_softbus_msgs::Path::ConstPtr& pre_plan)
{
  
  int path_size=pre_plan->navigationpoints.size();
  PathMessage pmsg;
  for (int i = 0; i < pre_plan->navigationpoints.size(); i++)
  {
      pmsg.px.push_back(pre_plan->navigationpoints[i].pose.position.x);
      pmsg.py.push_back(pre_plan->navigationpoints[i].pose.position.y);
      pmsg.pz.push_back(pre_plan->navigationpoints[i].pose.position.z);
      
      pmsg.ox.push_back(pre_plan->navigationpoints[i].pose.orientation.x);
      pmsg.oy.push_back(pre_plan->navigationpoints[i].pose.orientation.y);
      pmsg.oz.push_back(pre_plan->navigationpoints[i].pose.orientation.z);
      pmsg.ow.push_back(pre_plan->navigationpoints[i].pose.orientation.w);
      
      pmsg.arrival_time = pre_plan->navigationpoints[i].arrival_time.toSec();
      pmsg.formation = pre_plan->navigationpoints[i].formation;
  }
  ostringstream archiveStream;
  boost::archive::text_oarchive archive(archiveStream);
  archive<<pmsg;
  string pmsg_string="2"+archiveStream.str();
  //sendstring here
  std_msgs::String sendmsg;
  sendmsg.data = pmsg_string;
  path_pub_virtual.publish(sendmsg);
  
}

int main(int argc, char** argv)
{
   ros::init(argc,argv,"sim_manager");
   ros::NodeHandle n;
   srand(time(0));
   bool param_ok = ros::param::get ("~robotnum", robotnum);
   for(int i=0;i<robotnum;i++)
   {
      OdomHandle *p=new OdomHandle(i);
      odom_list.push_back(p);
      adj_list.push_back(vector<int>());
   }
   boost::thread* rcvThread;
   rcvThread= new boost::thread(&rcv_thread);
   //ofstream fout("/home/liminglong/czx/traject.txt");
   //ofstream fout2("/home/liminglong/czx/velocity.txt");
   ros::Publisher posepub = n.advertise<geometry_msgs::PoseArray>("/swarm_pose",1000);
   //ActivatePub = n.advertise<std_msgs::String>("/broadcast",1000);
   rviz_goal_pub_ = n.advertise<decide_softbus_msgs::NavigationPoint>("/uav0/move_base_simple/goal", 0 );
   ros::Subscriber rviz_sub = n.subscribe<geometry_msgs::PoseStamped>("/rviz_simple/goal", 1000, rvizGoalCb);
   
   rviz_start_action_client_= n.serviceClient<decide_softbus_msgs::SetControlling>("/uav0/move_base/set_controlling");
   ros::Subscriber rviz_start_action_sub = n.subscribe<geometry_msgs::PointStamped>("/rviz_simple/start_action", 1000, rvizStartActionCb);
   //ros::Subscriber virtual_sub = n.subscribe("/broadcast", 1000, virtual_rcv);
   ros::Subscriber takeoff_sub = n.subscribe("/takeoff", 1000, takeoffCb);
   ros::Subscriber land_sub = n.subscribe("/land", 1000, landCb);
   ros::Subscriber activate_sub = n.subscribe("/activate", 1000, activateCb);
   path_pub_virtual = n.advertise<std_msgs::String>("/broadcast",1000);
   ros::Subscriber swarm_plan_sub_ = n.subscribe<decide_softbus_msgs::Path>("/swarm_plan", 1000,swarmPlanCB);
   
   tf::TransformBroadcaster br;
   
   /*
   for(int i=0;i<robotnum;i++)
   {
       stringstream ss;
       ss<<"~uav"<<i<<"_x";
       bool param_ok = ros::param::get (ss.str(), odom_list[i]->odomx);
       stringstream ssy;
       ssy<<"~uav"<<i<<"_y";
       param_ok = ros::param::get (ssy.str(), odom_list[i]->odomy);
   }*/
   //neighbor_list.push_back(NeighborHandle(1));
   ros::Rate loop_rate(20);
   int count = 1;
   while(ros::ok())
   {
      /*publish pose array
      geometry_msgs::PoseArray sendpose;
       sendpose.header.frame_id="odom";
      for(int i=0;i<robotnum;i++)
      {
          geometry_msgs::Pose p;
         
          
          p.position.x = odom_list[i]-> _px;
          p.position.y = odom_list[i]-> _py;
          p.position.z = odom_list[i]-> _pz;
          
          tf::Quaternion q(odom_list[i]->_theta,0,0);
          p.orientation.x = q.x();
          p.orientation.y = q.y();
          p.orientation.z = q.z();
          p.orientation.w = q.w();
          sendpose.poses.push_back(p);
      }
      posepub.publish(sendpose);*/
      
      ros::spinOnce();
      /*
      for(int i=0;i<robotnum;i++)
      {
           for(int j=i+1;j<robotnum;j++)
           {
                if(dist(i,j)>0&&dist(i,j)<R)//calculate adjacent list
                {
                    adj_list[i].push_back(j);
                    adj_list[j].push_back(i);
                }
                if(dist(i,j) < stopdist)//publish stop msg (too close)
                {
                    std_msgs::Int32 stopmsg;
                    //cout<<"distance too close, trigering hovering"<<endl;
                    //for(int k=0;k<robotnum;k++)
                       // odom_list[k]->stop_pub.publish(stopmsg);
                }
           }
           //publish status msg 
           quadrotor_code::Status sendmsg;
           sendmsg.px = odom_list[i]-> _px;
           sendmsg.py = odom_list[i]-> _py;
           sendmsg.vx = odom_list[i]-> _vx;
           sendmsg.vy = odom_list[i]-> _vy;
           sendmsg.theta =  odom_list[i]-> _theta;
           odom_list[i]->pub.publish(sendmsg);
      }*/
      //publish odom 
      for(int i=0;i<robotnum;i++)
      {
          nav_msgs::Odometry odommsg;
          stringstream ss;
          ss<<"uav"<<i<<"/odom";
          odommsg.header.frame_id="map";
          odom_list[i]->mut.lock();
          odommsg.child_frame_id=ss.str().c_str();
          odommsg.pose.pose.position.x=odom_list[i]-> _px;
          odommsg.pose.pose.position.y=odom_list[i]-> _py;
          odommsg.pose.pose.position.z=odom_list[i]-> _pz;
         
          odommsg.pose.pose.orientation.x=odom_list[i]->xx;
          odommsg.pose.pose.orientation.y=odom_list[i]->yy;
          odommsg.pose.pose.orientation.z=odom_list[i]->zz;
          odommsg.pose.pose.orientation.w=odom_list[i]->ww;
          odom_list[i]->mut.unlock();
          odom_list[i]->fix_odom_pub.publish(odommsg);
      }
      //publish control status,0 for land, 1 for takeoff, 2 for activate
      string sendstring;
      sendstring="0"+status;
      //ActivatePub.publish(sendstring);
      string msg_type = "0";
      int index = 0;
      UDP_BC_test.UDP_BC_send(msg_type,sendstring);
      
      /*publish neighbor
      for(int i=0;i<robotnum;i++)
      {
           quadrotor_code::Neighbor sendmsg;
           sendmsg.data = adj_list[i];
           odom_list[i]->neighbor_pub.publish(sendmsg);
           adj_list[i]=vector<int>();
      }*/
      //publish tf map->base_link
      for(int i=0;i<robotnum;i++)
      {
          tf::Transform transform;
          odom_list[i]->mut.lock();
          transform.setOrigin( tf::Vector3(odom_list[i]->_px,odom_list[i]->_py , odom_list[i]->_pz) );
          tf::Quaternion q(odom_list[i]->xx,odom_list[i]->yy,odom_list[i]->zz,odom_list[i]->ww);
          odom_list[i]->mut.unlock();
          //q.setRPY(0, 0, odom_list[i]->odomtheta);
          //q.setRPY(0, 0, 0);
          transform.setRotation(q);
          
          stringstream ss;
          ss<<"uav"<<i<<"/base_link";
          
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", ss.str().c_str()));
          //cout<<"sent"<<endl;
      }
      count++;
      loop_rate.sleep();
   }
   return 0;
}
