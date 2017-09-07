#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <iostream>

using namespace std;

short AStatus = 0;//0: not opened, 1: failed, 2: opened, 3: finished
short BStatus = 0;

string AName;
string BName;

int Aframes = 0;
int Bframes = 0;
int Arf = 0;
int Brf = 0;
int Apf = 0;
int Bpf = 0;

short OStatus = 0;//0: not opened, 1: failed, 2: opened, 3: finished
string OName;
int Oframes = 0;

void infoCallback(const std_msgs::String::ConstPtr& msg)
{
  string res = msg->data;
  try
  {
    stringstream ss;
    size_t index = res.find(":");
    string name = res.substr(0,index);
    size_t index2 = res.find("#");
    ss = stringstream(res.substr(index+1, index2-index-1));
    int cmdchar;
    ss>>cmdchar;
    size_t index3 = res.find("$");
    ss = stringstream(res.substr(index2+1, index3-index2-1));
    int val1;
    ss>>val1;
    ss = stringstream(res.substr(index3+1));
    int val2;
    ss>>val2;
    if(name == AName)
    {
      if(cmdchar==0) AStatus = 1;
      else if(cmdchar==1) {AStatus = 2;Aframes=val1;}
      else if(cmdchar==2) {AStatus = 2;Arf=val1;Aframes=val2;}
      else if(cmdchar==3) {AStatus = 2;Apf=val1;Aframes=val2;}
      else if(cmdchar==4) AStatus = 3;
      else if(cmdchar==5) {AStatus = 2;Arf=Aframes;}
      else if(cmdchar==6) {AStatus = 2;Apf=Aframes;}
    }
    else if(name == BName)
    {
      if(cmdchar==0) BStatus = 1;
      else if(cmdchar==1) {BStatus = 2;Bframes=val1;}
      else if(cmdchar==2) {BStatus = 2;Brf=val1;Bframes=val2;}
      else if(cmdchar==3) {BStatus = 2;Bpf=val1;Bframes=val2;}
      else if(cmdchar==4) BStatus = 3;
      else if(cmdchar==5) {BStatus = 2;Brf=Bframes;}
      else if(cmdchar==6) {BStatus = 2;Bpf=Bframes;}
    }
  }
  catch (...){}
}

void outcomeCallback(const std_msgs::String::ConstPtr& msg)
{
  string res = msg->data;
  try
  {
    stringstream ss;
    size_t index = res.find(":");
    string name = res.substr(0,index);
    size_t index2 = res.find("#");
    ss = stringstream(res.substr(index+1, index2-index-1));
    int cmdchar;
    ss>>cmdchar;
    ss = stringstream(res.substr(index2+1));
    int val1;
    ss>>val1;
    if(name == OName)
    {
      if(cmdchar==0) OStatus = 1;
      else if(cmdchar==1) {OStatus = 2;}
      else if(cmdchar==2) {OStatus = 2;Oframes=val1;}
      else if(cmdchar==3) OStatus = 3;
    }
  }
  catch (...){}
}

string Status2String(short a, bool isA = true)
{
    if(a==0) return "not opened";
    else if(a==1) return "failed to open";
    else if(a==2) 
    {
        std::stringstream ss;
        if(isA) ss<<"read - "<<(Arf*100/Aframes)<<"\% process - "<<(Apf*100/Aframes)<<"\%";
        else ss<<"read - "<<(Brf*100/Bframes)<<"\% process - "<<(Bpf*100/Bframes)<<"\%";
        return ss.str();
    }
    else if(a==3) return "finished";
}

string Outcome2String(short a)
{
    if(a==0) return "not opened";
    else if(a==1) return "failed to open";
    else if(a==2) 
    {
        std::stringstream ss;
        ss<<"rendered frame(s) - "<<Oframes;
        return ss.str();
    }
    else if(a==3) return "finished";
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "info_display");

  AName = "/home/cooplab/field_trees.avi";
  BName = "/home/cooplab/dynamic_test.mp4";
  OName = "/home/cooplab/output.avi";

  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Subscriber sub = n.subscribe("stitch_two/process_status", 1000, infoCallback);
  ros::Subscriber sub_o = n.subscribe("stitch_two/outcome_status", 1000, outcomeCallback);

  while(ros::ok())
  {
      string res = "Video 1: "+Status2String(AStatus)+"    Video 2: "+Status2String(BStatus, false);
      cout<<res.c_str()<<endl;
      if((AStatus==1||AStatus==3)&&(BStatus==1||BStatus==3)) break;
      ros::spinOnce();
      r.sleep();
  }

  while(ros::ok())
  {
      string res = "Outcome video: "+Outcome2String(OStatus);
      cout<<res.c_str()<<endl;
      if(OStatus==1||OStatus==3) break;
      ros::spinOnce();
      r.sleep();
  }

  return 0;
}