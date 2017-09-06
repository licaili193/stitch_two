#include <thread>
#include <string>
#include <vector>

using namespace std;

class VideoLoader
{
    string _filePath;
    vector<thread>_threadList;

    void loadVideo();
    void inverseImage();
public:
    VideoLoader(string path);

    void Run();
    void Wait();
};