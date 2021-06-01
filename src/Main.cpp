#include "LaneDetector.hpp"
#include "ShowRoadLane.hpp"



int main(int argc, char *argv[]) 
{
        
   cv::String keys =
        "{v video |<none>           | input video path}"                               
        "{d delay |30                 | delay between two frames}"                
        "{help h usage ?    |      | show help message}";     
  
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Road lane detection");
    if (parser.has("help")) 
    {
        parser.printMessage();
        return 0;
    }
    cv::String videoPath = parser.get<cv::String>("video");
    int frameDelay = parser.get<int>("delay"); 
    std::cout << videoPath<<"\n";
    if (!parser.check()) 
    {
        parser.printErrors();
        return -1;
    }

 const char *source= videoPath.c_str();
    cv::VideoCapture cap(source);
    if (!cap.isOpened())return -1;
    cv::Mat frame;  

    // Main algorithm starts. Iterate through every frame of the video
    while (true) 
    {
      // Capture frame
      if (!cap.read(frame)) break;
      showRoadLane(frame);
      if(cv::waitKey(frameDelay) >= 0) break; 
    }
   return 0;
}
