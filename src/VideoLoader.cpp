#include <thread>
#include <string>
#include <vector>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "VideoLoader.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
using namespace cv;

VideoLoader::VideoLoader(string path, ros::NodeHandle* nd)
{
    _filePath = path;
    n = nd;
    info_pub = (*n).advertise<std_msgs::String>("stitch_two/process_status", 1000);
    _isStarted = false;
    _pubTime = ros::Time::now();
}

void VideoLoader::Run()
{
    _isStarted = true;
    _threadList.push_back(thread(&VideoLoader::loadVideo, this));
    _threadList.push_back(thread(&VideoLoader::inverseImage, this));
}

void VideoLoader::Wait()
{
    _threadList[0].join();
    _threadList[1].join();
    PublishInfo(4,0,0);
}

void VideoLoader::loadVideo()
{
    ROS_INFO("%s loading", _filePath.c_str());
    VideoCapture cap(_filePath);
    if(!cap.isOpened())
    {
        ROS_WARN("%s failed to open", _filePath.c_str());
        PublishInfo(0,0,0);
        _isStarted = false;
        return;
    }
    _videoInfo.length = int(cap.get(CV_CAP_PROP_FRAME_COUNT));
    _videoInfo.width  = int(cap.get(CV_CAP_PROP_FRAME_WIDTH));
    _videoInfo.height = int(cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    _videoInfo.fps    = cap.get(CV_CAP_PROP_FPS);
    ROS_INFO("%s opened - length: %d (frames) - width: %d - height: %d - fps: %f",\
         _filePath.c_str(),_videoInfo.length,_videoInfo.width,_videoInfo.height,_videoInfo.fps);

    Mat frame;
    int readSum = 1;
    while(cap.read(frame))
    {
        unique_lock<mutex> ul(mu);
        _buffer.push(frame);
        ul.unlock();
        cv.notify_one();
        readSum++;
        PublishInfo(2,readSum,_videoInfo.length);
    }
    _isStarted = false;
    cv.notify_one();
    ROS_INFO("%s load frames finished", _filePath.c_str());
    PublishInfo(5,0,0);
}

void VideoLoader::inverseImage()
{
    ROS_INFO("%s processing", _filePath.c_str());
    int procSum = 1;
    while(true)
    {
        unique_lock<mutex> ul(mu);
        if(_isStarted) cv.wait(ul);
        if(_buffer.empty())
        {
            if(_isStarted) {ul.unlock(); continue;}
            else break;
        }
        Mat frame = _buffer.front();
        _buffer.pop();
        ul.unlock();

        Mat greyMat;
        if(frame.channels() == 3) cvtColor(frame, greyMat, CV_BGR2GRAY);
        else greyMat = move(frame);

        /*Inverse frame by pixels here!*/
        for(int i=0; i<greyMat.rows; i++)
        {
            for(int j=0; j<greyMat.cols; j++)
            {
                greyMat.at<unsigned char>(i,j) = 255-greyMat.at<unsigned char>(i,j);
            }
        }
        _processedFrames.push_back(greyMat); 
        procSum++;
        PublishInfo(3,procSum,_videoInfo.length);
    }
    ROS_INFO("%s process frames finished", _filePath.c_str());
    PublishInfo(6,0,0);
}

vector<Mat>& VideoLoader::GetProcessedFrames()
{
    return _processedFrames;
}

VideoInfo VideoLoader::GetVideoInfo()
{
    return _videoInfo;
}

void VideoLoader::PublishInfo(int cmd, int val, int frm)
{
    /* cmd  0       1       2       3       4       5           6
            fail    opened  read    proc    finish  read_finish proc_finish
    */

    if(cmd==2||cmd==3)
    {
        if(ros::Time::now()<_pubTime+ros::Duration(0.1)) return;
    }

    std_msgs::String msg;
    
    std::stringstream ss;
    ss<<cmd<<"#"<<val<<"$"<<frm;
    string res = _filePath+":"+ss.str();
    msg.data = res;
    lock_guard<mutex>lck(mu_ros);
    info_pub.publish(msg); 
    ros::spinOnce();

    _pubTime = ros::Time::now();
}
