#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "VideoLoader.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stitch");
  ros::NodeHandle n;

  VideoLoader vl("/home/cooplab/field_trees.avi");
  vl.Run();
  vl.Wait();
  
  int temp;
  cin>>temp;

  return 0;
}