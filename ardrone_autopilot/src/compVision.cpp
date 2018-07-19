
/*  
    compVision.cpp

  -------------------------------------------------------------------
  | Provides the computer vision algorithm that processes           |     
  |  the image received from the imageHandler.cpp.                  |
  |                                                                 |
  | Sends back extracted information about the circles and the box. |
  |                                                                 |
  | Algorithm realisation is based on OpenCV tutorial               |
  |  from the documentation - goo.gl/rGCvhv.                        |
  -------------------------------------------------------------------

*/

#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>
#include "imgHelper.h"
using namespace cv;


// Image processing function.

void processImage(cv::Mat& src, CirclesMessage& msg) {

  // Initialisation.

  int thresh = 100;
  int max_thresh = 255;
  RNG rng(12345);
  Point2f camCenter(src.cols / 2, src.rows / 2);
  
  Mat threshold_output;
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  // Convert the captured frame from BGR to HSV.

  Mat imgHSV;
  cvtColor(src, imgHSV, COLOR_BGR2HSV);  

  Mat red_hue_range;
  Mat green_hue_range;
  Mat blue_hue_range;
  
  // Filter out only pixels in target's colors range.
  
  // First 3 numbers - lower thresholds, last 3 - higher tresholds.
  // Numbers depends on the color of the target and can be selected experimentally
  //  using laptop webcam.

  cv::inRange(imgHSV, cv::Scalar(30, 50, 40), cv::Scalar(79, 200, 255), green_hue_range);
  
  // It's also possible to use several ranges, summed with addWeighted() function.
  // Read OpenCV documentation to get more information.  

  cv::Mat summ = green_hue_range;

  // Remove unwanted noise.
 
  // Morphological opening (remove small objects from the foreground).
  
  erode(summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)) );
  dilate( summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 

  // Morphological closing (fill small holes in the foreground).

  dilate( summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
  erode(summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

  // Detect edges using threshold function.

  threshold( summ, threshold_output, thresh, 255, THRESH_BINARY );

  // Find contours on the image.

  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  // Find the rotated rectangles and ellipses for each contour.

  std::vector<RotatedRect> minRect( contours.size() );
  std::vector<RotatedRect> minEllipse( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

  // Preparing the message

  msg.circles = minEllipse;
  msg.inTheBox = std::vector<bool>(minEllipse.size(), false);

  Mat drawing = src;



  // Calculate central lines coordinates.

  Point2f leftCenter(0, drawing.rows / 2), rightCenter(drawing.cols, drawing.rows / 2);
  Point2f topCenter(drawing.cols / 2, 0), bottomCenter(drawing.cols / 2, drawing.rows);
       
  // Here is a dynamic box implementation.
  // It changes own coords with the size of the circles.

    // Get circles size.
    
    float circleWidth = 50, circleHeight = 50;
    
    if (msg.circles.size() != 0) {
        circleWidth = msg.circles[0].size.width;
        circleHeight = msg.circles[0].size.height;
    }
    
    // Calculation of the box coordinates using linear function of circles size.
    
       float boxTop = std::max((float)1, drawing.rows * (-9 * circleHeight + 1900) / 4600),
	         boxBottom = drawing.rows - boxTop,
	         boxLeft = std::max((float)1, drawing.cols * (-9 * circleWidth + 2160) / 4600),
	         boxRight = drawing.cols - boxLeft;
       
       Point2f boxTopLeft(boxLeft, boxTop), boxTopRight(boxRight, boxTop);
       Point2f boxBottomLeft(boxLeft, boxBottom), boxBottomRight(boxRight, boxBottom);
    
    // Preparing the message.

       msg.box.left = boxLeft;
       msg.box.right = boxRight;
       msg.box.top = boxTop;
       msg.box.bottom = boxBottom;

      //Draw central lines and the box.

       line( drawing, leftCenter, rightCenter, Scalar(255, 0, 0), 3, 8 );
       line( drawing, bottomCenter, topCenter, Scalar(255, 0, 0), 3, 8 );
       rectangle( drawing, boxBottomLeft, boxTopRight, Scalar(100, 0, 255, 0.1), 2, CV_AA);


  // Drawing of ellipses, rectangles and arrowed lines.

  for( int i = 0; i< contours.size(); i++ ) {
	
       // Get centers and sizes of the ellipses.
       Point2f center = minEllipse[i].center;
       Size2f size = minEllipse[i].size;

       // RGB color.
       Scalar color = Scalar( 0, 255, 0 );

       // Contour.
       drawContours( drawing, contours, i, color, 1, 8, std::vector<Vec4i>(), 0, Point() );

       // Ellipse.
       ellipse( drawing, minEllipse[i], color, 2, 8 );


	   // Center.
       cv::circle(drawing, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );

	   // Print coordinates of the centers
         cv::Scalar color2(1, 1, 1);
         Point2f y_p(center.x, center.y + 20);
         std::string x_coord = std::to_string((int)center.x), 
                     y_coord = std::to_string((int)center.y);
         cv::putText(drawing, "x: " + x_coord, center, cv::FONT_HERSHEY_DUPLEX, 0.8, color2);
         cv::putText(drawing, "y: " + y_coord, y_p, cv::FONT_HERSHEY_DUPLEX, 0.8, color2);
      
       // Draw a line to the circle if it is out the box.
       if (center.x > boxRight ||
           center.x < boxLeft ||
           center.y < boxTop ||
           center.y > boxBottom)
       {
           arrowedLine(drawing, camCenter, center, Scalar(255, 0, 0), 2, CV_AA);
        
           // Write this information to the message.
           msg.inTheBox[i] = true;
       }
    
       // Rotated rectangles.
       Point2f rect_points[4]; 
       minRect[i].points( rect_points );
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
     }
}

