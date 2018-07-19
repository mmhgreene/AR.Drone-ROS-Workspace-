
/*  
    controller.cpp
  ----------------------------------------------------------------------------
  | Receives the image from the drone and sends it to the compVision.cpp,    |
  | that processes it and extracts information about the circles and the box.|
  |                                                                          |
  | Then receives processed information and sends it to the controller.      |
  | Some utility classes and functions can be found in the controlHelper.h.  |
  ----------------------------------------------------------------------------

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "controlHelper.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>



// Global control object to have an easy access to some parameters.

struct ControlCenter control;


// Enable/disable controller by button 'N'.

void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n"; 
}


// Extract information from incoming message to the control object. 

void parseArray(const std_msgs::Float32MultiArray& msg) {
    if (msg.data.size() != 0) {
        size_t i = 0;
        Circle circle;
        while (i < msg.data.size()) {
            circle.x = msg.data[i++];
            circle.y = msg.data[i++];
            circle.width = msg.data[i++];
            circle.height = msg.data[i++];
            circle.inTheBox = !msg.data[i++];
            control.targ.push_back(Circle(circle));
        }   
    }   
}


// Drone autopilot control.

void controller(geometry_msgs::Twist& msg) {

    std::vector<float> errorsX;
    std::vector<float> errorsY;
    float middleX = (control.box.right+control.box.left) / 2;
    float middleY = (control.box.top+control.box.bottom) / 2;
 
    // Calculate X and Y errors of the circles that are not in the box.

    for (const auto& circle : control.targ) {

        if (!circle.inTheBox) {
		    
		float Xerr = circle.x - middleX;
		errorsX.push_back(Xerr);

		float Yerr = circle.y - middleY;
		errorsY.push_back(Yerr);
		
		std::cout << "YERR : " << Yerr << " XERR: " << Xerr << "\n"; 
        }
    }

    // Controlling.
    // X part.

    if (errorsX.size() != 0) {

	// Calculate average X error and normalize it to the [-1, 1] by dividing to the max error.

        float summError = 0, avError;
        size_t numError = errorsX.size();
        for (const auto& error : errorsX) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.left;
	
/*      It's also possible to change the PID coefficient with 
         the size of the target to make drone more accurate.

        Not used, optionally. Also should be added in the Y part.
	control.pid.kP = std::max((float)0.05, (320 - control.targ[0].width) / 1800);
*/


	// Calculate necessary acceleration (velocity indeed, but it requires double PID).

        float acc = control.pid.calculate(avError, true);

	// Send the command to the drone (the command changes drone's tilt).

        msg.linear.y = -acc;
    }

    // The Y part is similar to the X part.

    if (errorsY.size() != 0) {
        float summError = 0, avError;
        size_t numError = errorsY.size();
        for (const auto& error : errorsY) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.top;
        float vel = control.pid.calculate(avError, false);
        msg.linear.x = -vel;
    }

    // Output sended values to the console.

    std::cout << "Command sended!\n";
    std::cout << "Vx -> " << msg.linear.y << '\n';
    std::cout << "Vy -> " << msg.linear.x << '\n';
}

// Recieves information and runs the control function the drone.
// Runs when the target information has been received.

void onTarget(const std_msgs::Float32MultiArray& msg) {
        if (control.enabled) {

            geometry_msgs::Twist message;

	// Runs every 0.06 ms that approximately corresponds to every 2 frame.
        // Can be changed.

	    if ((ros::Time::now() - control.lastLoop).toSec() >= 0.06) {

		    control.targ.clear();
		// Handle received information.
		    parseArray(msg);
		
		// Calculate delta time.
		    control.pid.dt = (ros::Time::now() - control.lastLoop).toSec();
		    control.lastLoop = ros::Time::now();

		// Run the controller.
		    controller(message);
		    
		// Circle and box information output		    
		    std::cout << "-----------------------\n";
		    std::cout << "BOX LEFT: " << control.box.left << '\n';
		    std::cout << "BOX RIGHT: " << control.box.right << '\n';
		    std::cout << "BOX TOP: " << control.box.top << '\n';
		    std::cout << "BOX BOTTOM: " << control.box.bottom << '\n' << '\n';
		    std::cout << "-----------------------\n";
		    for (auto& circle : control.targ) {
			std::cout << circle << '\n';
		    }
		    std::cout << "-----------------------\n";
			
                // Send the message.
		    control.cmdPublisher.publish(message);
	    }
	    
	} else {
	// Reset saved information if the controller was turned off.
        control.pid.integralX = 0;
        control.pid.integralY = 0;
        control.pid.prevErrorX = 0;
        control.pid.prevErrorY = 0;
    }
}

// Extract box information from incoming message.

void onBox(const std_msgs::Float32MultiArray& msg) {
        control.box.left = msg.data[0];
        control.box.right = msg.data[1];
        control.box.top = msg.data[2];
        control.box.bottom = msg.data[3];
        control.imgRows = msg.data[4];
        control.imgCols = msg.data[5];
}
        

// Changing PID coefficients by buttons.

// Reducing.

void onPidD(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP += 0.005; // 'F1'
                  break;
        case 'i': control.pid.kI += 0.001; // 'F3'
                  break;
        case 'd': control.pid.kD += 0.005; // 'F5'
                  break;
    }
}

// Increasing.

void onPidI(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP -= 0.0005; // 'F2'
                  break;
        case 'i': control.pid.kI -= 0.0005; // 'F4'
                  break;
        case 'd': control.pid.kD -= 0.0005; // 'F6'
                  break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle node;
    
    ros::Subscriber boxSub = 
            node.subscribe("box", 1, onBox);

    ros::Subscriber enableSub = 
            node.subscribe("controller/enable", 5, onEnableCtrl);


    ros::Subscriber targetSub = 
            node.subscribe("target", 5, onTarget);

    ros::Subscriber pidDecreaseSub = 
            node.subscribe("pid/decrease", 5, onPidD);

    ros::Subscriber pidIncreaseSub = 
            node.subscribe("pid/increase", 5, onPidI);

    control.cmdPublisher =
            node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::spin();

    return 0;
}
