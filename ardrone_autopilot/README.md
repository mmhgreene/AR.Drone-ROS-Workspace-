# AR.Drone 2.0 autopilot.

## What the project is about
### General information about the drone
Parrot AR.Drone is a remote controlled flying quadcopter helicopter built by the French company Parrot. The drone is designed to be controlled by mobile or tablet operating systems such as the supported iOS or Android within their respective apps or the unofficial software available for Windows Phone, Samsung BADA and Symbian devices.

The airframe of the AR.Drone, constructed of nylon and carbon fiber parts, measures 57 cm (22 in) across. Two interchangeable hulls were supplied with the airframe, one designed for indoor and one for external flight. The indoor hull is made from EPP foam, and encases the circumference of the blades for protection. The outdoors use hull is made from lightweight plastic, and allows for increased maneuvrability. In total, the AR.Drone has six degrees of freedom, with a miniaturized inertial measurement unit tracking the pitch, roll and yaw for use in stabilisation.

Inside the airframe, a range of sensors assist flight, enabling the interface used by pilots to be simpler, and making advanced flight easier. The onboard computer runs a Linux operating system, and communicates with the pilot through a self-generated Wi-Fi hotspot. The onboard sensors include an ultrasonic altimeter, which is used to provide vertical stabilisation up to 6 m (19 ft 8 in). The rotors are powered by 15 watt, brushless motors powered by an 11.1 Volt lithium polymer battery. This provides approximately 12 minutes of flight time at a speed of 5 m/s (11 mph). Coupled with software on the piloting device, the forward-facing camera allows the drone to build a 3D environment, track objects and drones, and validate shots in augmented reality games.

### Goals and project's realisation

Our task was the implementation of navigation system for drones, to do it we should have done:
* 	Track a marked object using OpenCV
* 	Build a controller that uses navigation data to set the drones navigation parameters: vertical velocity, and rotation about 3 axis.

Our project based on ROS library and, like any ROS project, it consists of separated files and each of them can be written either in Python or in C++. These files call nodes and every node is respond for the part of the work: 

* Controller - makes the drone fly above the circles and follow them, if the target isn't static.
* Interface - provides an keyboard control of the drone and shows an image from the camera.
* Image Processing - processes the image from the camera and extracts information about the target.

Every node can exchange information with each other, using ROS interfase (publishers and subscribers).<br>

<b> Important note! </b> <br>
There are two versions of programs. The most number of differences are in the controller's realisation, other parts are generally similar. <br>
They can be found in different branches in the repository:
* go - Georgiy Kozhevnikov's project variation
* ju - Julia Reneva's project variation

#### What we have done
* Bright colored target detection
* Controller that makes the drone to follow recognized target

## How to use
### Getting started
All the libraries and soft work only in Ubuntu or Debian. So, firsly install Linux if you don't use it, and than all to let our project compile.

1. Catkin - library to build the ROS project. <br>Documentation - http://wiki.ros.org/catkin. <br>To install execute in command line:

```
sudo apt-get install ros-kinetic-catkin
```

2. ROS - open python/c++  library for robotics. We wrote all the code for the "kinetic" version. All the instructions - http://wiki.ros.org/kinetic/Installation/. It's recommended to perform full instalation.

3. Ardrone_autonomy - library from the developers of the drone, gives the basic commands to control it - take off, land, hover and so on. <br> Documentation - https://ardrone-autonomy.readthedocs.io/en/latest/installation.html <br> To install just execute in command line:

```
sudo apt-get install ros-kinetic-ardrone-autonomy
```

4. OpenCV – open с++/python library to recognising images, video, objets and stuff like that.<br>
Download - https://sourceforge.net/projects/opencvlibrary/<br>
Documentation - http://docs.opencv.org/3.3.0/<br>
Installation - http://docs.opencv.org/3.3.0/d7/d9f/tutorial_linux_install.html<br>

5. Qt library - download the installer on button «Get your open source package» on website https://info.qt.io/download-qt-for-application-development

6. Optional. You can use Gazebo simulator to work with a virtual drone. If you have performed a full ROS installation, the Gazebo is already installed. All you need - to install tum_simulator package, that simulates the AR.Drone in the simulator.<br>
If you use the kinetic ROS version, you should install ported version of tum_simulator: https://github.com/angelsantamaria/tum_simulator
In other case it's possible to use ROS wiki tutorial about tum_simulator: http://wiki.ros.org/tum_simulator

After that, you need to build the project. Change the directory for that one, where you have "src" folder. In command line execute "catkin_make" in this directory. Now all the changes are saved and we can run the project. After that, you should edit your .bashrc file in your home directory:

```
cd ~
sudo gedit .bashrc
```
You should go to the end of the file and add these strings there:
```
source ~/catkin_ws(or another workspace name)/devel/setup.bash
source /opt/ros/kinetic/setup.bash
```
It will allow you to run your project from the home directory.

### Running:
#### With real drone:
To run the project we need to run launch files. It is in the folder named "launch". Now there are 2 files: ardrone_autopilot.launch and enviroment.launch. You need to execute in 2 different windows:

```
roslaunch ardrone_autopilot enviroment.launch
```
It will connect your computer to the drone using Wi-Fi signal. You should be already connected to the drone's hotspot.<br>
To launch the main program, run this:

```
roslaunch ardrone_autopilot autopilot.launch
```

You'll see the window with the image from camera and console windows with the debug information.<br>

#### In Gazebo simulator:
If you want to run the autopilot using Gazebo, you should run these commands from the home directory:

```
roscore
```
It will lauch a main ROS node that allows other nodes to exchange information. (it works instead of environment.launch)
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
It will launch the Gazebo simulator with a default map.
```
roslaunch ardrone_autopilot autopilot.launch
```
It will launch the autopilot program as it was described above.

### Controlling the drone:
#### Keyboard:

|Buttons: |Info|
|-----|------|
|W, A, S, D | tilt forward/left/backward/right
|Q, E| rotate left/right
|T | take off
|L | land
|[ or ] | up or down
|C | change camera
|M |to turn on the image processing
|N | turn on the autopilot
|  | <b>Only in go branch:</b>|
|F1| Decrease P coefficient
|F3| Decrease I coefficient
|F5| Decrease D coefficient
|F2| Increase P coefficient
|F4| Increase I coefficient
|F6| Increase D coefficient

#### Autopilot:
It's recommended to firstly turn on an image processing using 'M' button. Then you should find the target with your bottom camera. Thereafter, you can enable an autopilot by pressing 'N'.<br>
<b>Be careful! You cannot control the drone via keyboard while the autopilot is enabled!</b>


## Short excursion
The project consists of 4 main code files.
* **imgHandler.cpp** - recieves the picture from the drone and processes it using compVision.cpp. When necessary information was extracted, it sends it to the controller.cpp.
* **compVision.cpp** - here is a CV algorithm implementation. It handle the picture, extracts only pixels of the target's color using HSV ranging. Then, the algorithm determines the shape of result objects and draws a circles around them.<br>
Moreover, all the drawings that you can see on the screen are performed in this file.
#### About the target's color
By default it is tuned up to a bright green target. Parameters that are responsible for the color detection are in the cv::inRange() function. You can read more about it in documentation. Also there are some good tutorials on YouTube.<br>
**To tune the program to detect your color, you can use this code:** https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/object-tracking-tut/objectTrackingTut.cpp <br>
It's possible to buid it using cmake and then run as usual binary file. The program uses your computer camera as input. Tuning process is described here: https://www.youtube.com/watch?v=bSeFrPrqZ2A
* **interface.py** - here is the keyboard control and the camera window are implemented.
* **drone.py** - used by interface.py to control the drone.


## Some future recomendations

We've found some interesting libraries, which can make the drone control easier:

* Nodecopter - nodejs library, based on ardrone_autonomy, but with some useful functions, which are nit existing in ardrone_autonomy, easier to use, because it doesn't need many other libraries to work.
<br>Documentation and installation - https://github.com/felixge/node-ar-drone

* tum_ardrone - ROS package that has an already implemented GUI, keyboard controller, camera-based autopilot and PID controller. Also there is a lot of interesting things to work with.<br>
<b>Here is a aproblem with this package:</b> it doesn't work with kinetic-ROS version. But its possible to use it with elder versions. You can read more in GitHub documentation or in ROS wiki: 
<br>ROS wiki - http://wiki.ros.org/tum_ardrone
<br>GitHub - https://github.com/tum-vision/tum_ardrone
