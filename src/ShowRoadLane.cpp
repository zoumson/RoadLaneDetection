#include "ShowRoadLane.hpp"

int showRoadLane(cv::Mat frame) 
{
        
 
    // Create the class object
    LaneDetector lanedetector;  
    
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
  

      // Denoise the image using a Gaussian filter
      img_denoise = lanedetector.deNoise(frame);

      // Detect edges in the image
      img_edges = lanedetector.edgeDetector(img_denoise);

      // Mask the image so that we only get the ROI
      img_mask = lanedetector.mask(img_edges);

      // Obtain Hough lines in the cropped image
      lines = lanedetector.houghLines(img_mask);

      // Separate lines into left and right lines
      left_right_lines = lanedetector.lineSeparation(lines, img_edges);

      lane = lanedetector.regression(left_right_lines, frame);
      // Apply regression to obtain only one line for each side of the lane     
      lanedetector.plotLane(frame, lane);

   return 0;
}
