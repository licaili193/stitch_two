#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef __VIDEOLOADER
#define __VIDEOLOADER

struct VideoInfo
{
    int length;
    int width;
    int height;
    double fps;
};

class VideoLoader
{
    ros::NodeHandle* n;
    ros::Publisher info_pub;
    ros::Time _pubTime;

    string _filePath;
    vector<thread>_threadList;
    VideoInfo _videoInfo;

    mutex mu;
    mutex mu_ros;
    condition_variable cv;

    queue<Mat>_buffer;
    vector<Mat>_processedFrames;
    bool _isStarted;

    void loadVideo();
    void inverseImage();

    void PublishInfo(int cmd, int val, int frm);
public:
    VideoLoader(string path, ros::NodeHandle* nd);

    void Run();
    void Wait();

    vector<Mat>& GetProcessedFrames();
    VideoInfo GetVideoInfo();
};

#endif
