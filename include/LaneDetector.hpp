
/*
* License plate detection
* See COPYRIGHT file at the top of the source tree.
*
* This product includes software developed by the
* STARGUE Project (http://www.stargue.org/).
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the STARGUE License Statement and
* the GNU General Public License along with this program. If not,
* see <http://www.lsstcorp.org/LegalNotices/>.
*/

/**
 * @file LaneDetector.h
 *
 * @brief Take RGB images as inputs and output the same RGB image with the plot of the detected lanes.
 *
 * @author Adama Zouma
 * 
 * @Contact: stargue49@gmail.com
 *
 */

#pragma once // same as  include guards
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

/**
 *@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
 *@brief Activity diagram and UML Class diagram.
 *@brief It detects the lanes in an image if a highway and outputs the
 *@brief same image with the plotted lane.
 */
class LaneDetector 
{
 private:
  double img_size;
  double img_center;
  bool left_flag = false;  // Tells us if there's left boundary of lane detected
  bool right_flag = false;  // Tells us if there's right boundary of lane detected
  cv::Point right_b;  // Members of both line equations of the lane boundaries:
  double right_m;  // y = m*x + b
  cv::Point left_b;  //
  double left_m;  //

 public:
  cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
  cv::Mat edgeDetector(cv::Mat img_noise);  // Filter the image to obtain only edges
  cv::Mat mask(cv::Mat img_edges);  // Mask the edges image to only care about ROI
  std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);  // Detect Hough lines in masked edges image
  std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);  // Sprt detected lines by their slope into right and left lines
  std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);  // Get only one line for each side of the lane
  //std::string predictTurn();  // Determine if the lane is turning or not by calculating the position of the vanishing point
  //int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);  // Plot the resultant lane and turn prediction in the frame.
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane);  // Plot the resultant lane and turn prediction in the frame.

};
