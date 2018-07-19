
/*  
    imgHelper.h
  ---------------------------------------------------
  | Contains some utility structures and functions  |
  |  for imageHandler.cpp and compVision.cpp files. |
  ---------------------------------------------------

*/

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

struct Box {
    float left;
    float right;
    float top;
    float bottom;
};


struct CirclesMessage {
    std::vector<cv::RotatedRect> circles;
    Box box;
    std::vector<bool> inTheBox;
};

extern void processImage(cv::Mat&, CirclesMessage&);


// Main structure of imageHandler.cpp, that contains 
//  some image information, flags and publishers.

struct ImageHandler
{
    float imgRows;
    float imgCols;

    bool cv_enabled = false;

    ros::Publisher imgPublisher;
    ros::Publisher circlePublisher;
    ros::Publisher boxSendler;

    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};



