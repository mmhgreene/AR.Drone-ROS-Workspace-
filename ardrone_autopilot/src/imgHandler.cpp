
/*  
    imgHandler.cpp
  ----------------------------------------------------------------------------
  | Receives the image from the drone and sends it to the compVision.cpp,    |
  | that processes it and extracts information about the circles and the box.|
  |                                                                          |
  | Then receives processed information and sends it to the controller.      |
  ----------------------------------------------------------------------------

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "imgHelper.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

// Initialize global objects to have an easy access to them.

struct ImageHandler imgHandler;
static CirclesMessage cMessage;



// Convert CircleMessage to Float32MultiArray (only information about box).
// It provides the possibility of using a static box.

void cmsg2BoxMultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& boxToSend) {
        std::vector<float> vec1 = {
                                    cmsg.box.left, 
                                    cmsg.box.right, 
                                    cmsg.box.top, 
                                    cmsg.box.bottom,
                                    imgHandler.imgRows, imgHandler.imgCols 
                                  };

        boxToSend.data.insert(boxToSend.data.end(), vec1.begin(), vec1.end());
}


// Convert CircleMessage to Float32MultiArray (information about circles).

void cmsg2MultiArray(CirclesMessage& cmsg, std_msgs::Float32MultiArray& msg) {
        std::vector<float> vec1;
        for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
            vec1.clear();
            vec1 = {
                    cmsg.circles[i].center.x,
                    cmsg.circles[i].center.y,
                    cmsg.circles[i].size.width,
                    cmsg.circles[i].size.height,
                    cmsg.inTheBox[i]
                    };
            
            msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
        }
} 



// Get and process the image from camera.
// Runs when the image has been received.

void onImage(const sensor_msgs::Image::ConstPtr& image)
{

    // Converts the image to the OpenCV format.
    // Copied from ROS wiki's cv_bridge tutorial - goo.gl/o4zV41 (4).

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch	 (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    // Image processing

   if (imgHandler.cv_enabled) {
        
        // Process the image and get circle information to the cMessage.
        // The image sends to the compVision.cpp.

        processImage(cv_ptr->image, cMessage);
        
        // Convert information from compVision.cpp.
        
        // It's optional to recieve box information only 1 time
        //  using a bool flag to get static box.

        std_msgs::Float32MultiArray msg;
        std_msgs::Float32MultiArray sendBox;

        cmsg2BoxMultiArray(cMessage, sendBox);
        cmsg2MultiArray(cMessage, msg);

        imgHandler.imgRows = cv_ptr->image.rows;
        imgHandler.imgCols = cv_ptr->image.cols;


        // Publish messages to the ROS topics.

        imgHandler.circlePublisher.publish(msg);
        imgHandler.boxSendler.publish(sendBox);
    }

    // Publish the image to the ROS topic.
    // It needs to show the picture in the GUI window.

    imgHandler.imgPublisher.publish(cv_ptr->toImageMsg());

}


// Get camera information.

void onCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    imgHandler.cameraDistortion = cam_info->D;
    imgHandler.cameraMatrix = cam_info->K;
}



// Enable/disable image processing by button 'M'.

void onEnable(const std_msgs::Empty& toggle_msg) {
    imgHandler.cv_enabled = !imgHandler.cv_enabled;
    if (imgHandler.cv_enabled) {
        std::cout << "Image processing enabled.\n";
    } else {
        std::cout << "Image processing disabled.\n";
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;
    
    // Button signal subscriber

    ros::Subscriber enableSub = 
            node.subscribe("cv/enable", 1, onEnable);
    
    // Information publishers

    imgHandler.imgPublisher = 
            node.advertise<sensor_msgs::Image>("/out/image", 5);
    
    imgHandler.boxSendler = 
            node.advertise<std_msgs::Float32MultiArray>("box", 1);
    
    imgHandler.circlePublisher = 
            node.advertise<std_msgs::Float32MultiArray>("target", 5);
    

    // Camera subscribers

    ros::Subscriber sub1 = 
            node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 = 
            node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 = 
            node.subscribe("/ardrone/camera_info", 5, onCameraInfo);

    ros::spin();

    return 0;
}
