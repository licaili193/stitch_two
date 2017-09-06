#include <thread>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "VideoLoader.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
using namespace cv;

VideoLoader::VideoLoader(string path)
{
    _filePath = path;
}

void VideoLoader::Run()
{
    _threadList.push_back(thread(&VideoLoader::loadVideo, this));
}

void VideoLoader::Wait()
{
    for(int i=0;i<_threadList.size();i++) _threadList[i].join();
}

void VideoLoader::loadVideo()
{
    VideoCapture cap(_filePath);
    if(!cap.isOpened())
    {
        ROS_WARN("%s failed to open", _filePath.c_str());
        return;
    }
    ROS_INFO("%s opened", _filePath.c_str());
}

void VideoLoader::inverseImage()
{

}
