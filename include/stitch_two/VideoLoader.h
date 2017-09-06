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
    string _filePath;
    vector<thread>_threadList;
    VideoInfo _videoInfo;

    mutex mu;
    condition_variable cv;

    queue<Mat>_buffer;
    vector<Mat>_processedFrames;
    bool _isStarted;

    void loadVideo();
    void inverseImage();
public:
    VideoLoader(string path);

    void Run();
    void Wait();

    vector<Mat>& GetProcessedFrames();
    VideoInfo GetVideoInfo();
};

#endif
