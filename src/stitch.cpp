#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "VideoLoader.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stitch");
  ros::NodeHandle n;

  VideoLoader vl("/home/cooplab/field_trees.avi", &n);
  VideoLoader v2("/home/cooplab/dynamic_test.mp4", &n);
  vl.Run();
  v2.Run();
  vl.Wait();
  v2.Wait();

  return 0;
}