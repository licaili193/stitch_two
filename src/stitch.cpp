#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "VideoLoader.h"

using namespace std;
using namespace cv;

string outputName;

ros::Publisher info_pub;
ros::Time pubTime;

void PublishInfo(int cmd, int val)
{
    /* cmd  0       1       2       3     
            fail    opened  render  finish
    */

    if(cmd==2)
    {
        if(ros::Time::now()<pubTime+ros::Duration(0.1)) return;
    }

    std_msgs::String msg;
    
    std::stringstream ss;
    ss<<cmd<<"#"<<val;
    string res = outputName+":"+ss.str();
    msg.data = res;
    info_pub.publish(msg); 
    ros::spinOnce();

    pubTime = ros::Time::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stitch");
  ros::NodeHandle n;

  info_pub = n.advertise<std_msgs::String>("stitch_two/outcome_status", 1000);

  outputName = "/home/cooplab/output.avi";

  VideoLoader v1("/home/cooplab/field_trees.avi", &n);
  VideoLoader v2("/home/cooplab/dynamic_test.mp4", &n);
  v1.Run();
  v2.Run();
  v1.Wait();
  v2.Wait();

  VideoInfo vif1 = v1.GetVideoInfo();
  VideoInfo vif2 = v2.GetVideoInfo();

  double finalFps = min(vif1.fps,vif2.fps);
  int finalHeight = min(vif1.height,vif2.height);
  int finalWidth;
  int resizedWidth;
  if(vif1.height!=finalHeight)
  {
    resizedWidth = (int)((double)finalHeight/(double)vif1.height*(double)vif1.width);
    finalWidth = resizedWidth + vif2.width;
  }
  else
  {
    resizedWidth = (int)((double)finalHeight/(double)vif2.height*(double)vif2.width);
    finalWidth = resizedWidth + vif1.width;
  }

  vector<Mat>mlist1 = v1.GetProcessedFrames();
  vector<Mat>mlist2 = v2.GetProcessedFrames();

  int index1 = 0;
  int index2 = 0;
  int mainIndex = 0;

  VideoWriter outputVideo;
  outputVideo.open(outputName, VideoWriter::fourcc('D','I','V','X'), finalFps, Size(finalWidth,finalHeight), false);
  if (!outputVideo.isOpened())
  {
    ROS_WARN("%s failed to create video", outputName);
    PublishInfo(0,0);
    return -1;
  }

  ROS_INFO("%s start to write", outputName);
  pubTime = ros::Time::now();
  PublishInfo(1,0);
  //namedWindow( "Display window", WINDOW_AUTOSIZE );

  while(index1<mlist1.size() && index2<mlist2.size())
  {
    Mat resultFrame(finalHeight,finalWidth,CV_8UC1);
    if(vif1.height!=finalHeight)
    {
      Mat temp;
      resize(mlist1[index1], temp, Size(resizedWidth, finalHeight), 0, 0, INTER_CUBIC);
      Mat roi = resultFrame(Rect(0,0,resizedWidth,finalHeight));
      temp.copyTo(roi);
      Mat roj = resultFrame(Rect(resizedWidth,0,finalWidth-resizedWidth,finalHeight));
      mlist2[index2].copyTo(roj);
    }
    else
    {
      Mat temp;
      resize(mlist2[index2], temp, Size(resizedWidth, finalHeight), 0, 0, INTER_CUBIC);
      Mat roi = resultFrame(Rect(0,0,finalWidth-resizedWidth,finalHeight));
      mlist1[index1].copyTo(roi);
      Mat roj = resultFrame(Rect(finalWidth-resizedWidth,0,resizedWidth,finalHeight));
      temp.copyTo(roj);
    }
    //imshow( "Display window", resultFrame );
    //waitKey(0);
    outputVideo<<resultFrame;
    PublishInfo(2,mainIndex);
    mainIndex++;
    index1 = (int)(vif1.fps/finalFps*(double)(mainIndex));
    index2 = (int)(vif2.fps/finalFps*(double)(mainIndex));
  }

  PublishInfo(3,0);
  ROS_INFO("%s write finished", outputName);
  return 0;
}