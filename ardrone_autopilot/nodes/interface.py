#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""UI node for visualizing the drone state

This node provides the QT user interface and keyboard input.


Inputs
------

* /ardrone/navdata -- information about the drone state.

* /ui/message -- messages stream.

  All messages published to this stream will be displayed.

  Those messages formed like `name::messsage` will update information
  on the left side of the screen (you should add all names to the `grid` list
  in order to get those messages displayed).

  Messages which doesn't match the above pattern will be shown on the right
  side of screen. They will be displayed for `message_display_time` time
  (which is 5sec. by default) and then removed from screen.

* /in/image -- main picture stream.


Outputs
-------

We use `DroneController` class to emit user control information.

* /ardrone/land -- single `land` command
* /ardrone/takeoff -- single `takeoff` command
* /ardrone/reset -- single `reset` command
* /cmd_vel -- velocity control commands (send on each keypress)


Parameters
----------

* ~message_display_time = 5000 [uint] -- time after which
  anonymous messages will be hidden away from screan (in milliseconds).
* ~connection_check_period = 500 [uint] -- consider dorne is offline if we had
  no messages for more than this time (in milliseconds).
* ~fps = 50 [uint] -- interface update rate.
* ~swap_red_blue = False [bool] -- set this to `True` if you need to swap
  red and blue channels (if you have to enable this, check that other nodes
  work fine with this stream; it's better to swap image color before
  passing it to the system, not after).

"""

import re
from collections import deque
from threading import Lock

from PySide import QtCore, QtGui

import rospy
import math
import time
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata

from utils.drone import DroneController


# Message groups
grid = [
    'drone.status',
    None,
    'drone.battery',
    None,
    ['tgt.x',
     'tgt.y',
     'tgt.z'],
]

""" Postition of the marker """
marker_x = 0
marker_y = 0
""" Most current position of the drone """
drone_x = 0
drone_y = 0
""" Most current position of the drone at the time of doing CheckHypo to assure no changes in position mid-calculation """
xdrone = 0
ydrone = 0
""" Distances in x and y direction between the marker and the drone """
xdistance = 0
ydistance = 0
""" The Euclidean distance between the marker and the drone's current position """
euclidDistance = 0
""" Assures that a function that a key press is calling only goes off once. A keypress usually calls the function twice. Can be applied to all key functions"""
keypressChecker = 0

""" Subscriber callback function to return marker coordinates """
def markerCallback(data):
    print("Callback is called")
    marker_x = data.pose.position.x
    marker_y = data.pose.position.y

""" Subscriber callback function to return drone coordinates """
def droneCallback(data):
    global drone_x
    global drone_y
    time.sleep(.3)
    drone_x = data.pose.position.x
    drone_y = data.pose.position.y

""" Using marker and drone coordinates, get the x/y distance between the two and calculate and return the hypotenuse or Euclidian distance """
def CheckHypo():
        #get drone most recent X,Y
	global drone_x
	global drone_y
	global xdrone
	global ydrone
	global xdistance
	global ydistance
	global euclidDistance
     	xdrone = drone_x
	ydrone = drone_y
	xdistance = (marker_x - xdrone)
	ydistance = (marker_y - ydrone)
        euclidDistance = math.hypot(xdistance,ydistance)
	time.sleep(.5)
	print(euclidDistance)
        return euclidDistance

class Messages(object):
    def __init__(self, message_display_time, *args):
        self.message_structure = args
        self.messages_named = {}
        self.messages_queue = deque()
        self.message_display_time = message_display_time
        self.lock = Lock()

    def messages_put(self, messages):
        """Add new messages to UI"""
        with self.lock:
            for message, name in messages:
                if name is None:
                    self.messages_queue.append((message, rospy.get_time()))
                else:
                    self.messages_named[name] = message

    def message_put(self, message, name=None):
        """Add one new message to UI"""
        self.messages_put([(message, name), ])

    def messages_flush(self):
        """Remove all messages"""
        with self.lock:
            messages = {}

    def clean_queue(self):
        """Remove all outdated messages from the queue"""
        with self.lock:
            while (self.messages_queue and
                   rospy.get_time() - self.messages_queue[0][1] >
                   self.message_display_time / 1000):
                self.messages_queue.popleft()

    def render(self, image):
        """Print all messages onto the given image"""
        self.clean_queue()

        painter = QtGui.QPainter()
        painter.begin(image)
        painter.setPen(QtGui.QColor(255, 255, 255))

        width, height = image.width(), image.height()
        x, y = int(width * .05), int(height * .05)
        column_width = 250

        with self.lock:
            for entry in self.message_structure:
                if entry is None:
                    entry = []
                if not isinstance(entry, (list, tuple)):
                    entry = [entry]
                for name in entry:
                    value = self.messages_named.get(name, '-')
                    if value is None:
                        value = '-'
                    painter.drawText(x, y, '%s: %s' % (name, value))
                    y += 15
                y += 5
                if y > int(height * .9):
                    y = int(height * .05)
                    x += column_width

            x, y = int(width - column_width), int(height * .05)
            for text, time in self.messages_queue:
                painter.drawText(x, y, text)
                y += 15
                if y > int(height * .9):
                    y = int(height * .05)
                    x -= column_width

        painter.end()


class UInode(QtGui.QMainWindow):
    keypressChecker = 0;
    def __init__(self):
        super(UInode, self).__init__()

        self._ap_topic = rospy.Publisher('/apctrl', Empty, queue_size=5)
        self.webcam_shutdown_pub = rospy.Publisher('/webcam/shutdown', Empty, queue_size=5)

        self.swap_red_blue = rospy.get_param('~swap_red_blue', False)

        self.controller = DroneController(
            offline_timeout=rospy.get_param('~connection_check_period', 500))

        self.keymap = self.gen_keymap()
	# Without release action
	self.keymap2 = self.gen_keymap2()

        self.messages = Messages(
            rospy.get_param('~message_display_time', 5000), *grid)

        self.messages_named_template = re.compile(
            r'((?P<name>[a-zA-Z0-9_-]+)::)?(?P<message>.*)')

        self.setWindowTitle('ARdrone camera')
        self.image_box = QtGui.QLabel(self)
        self.setCentralWidget(self.image_box)

        self.image = None
        self.image_lock = Lock()

        fps = rospy.get_param('~fps', 50)

        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.on_redraw)
        self.redraw_timer.start(1000 / fps)

        rospy.Subscriber('/ui/message', String, self.on_ui_request)
        rospy.Subscriber('/out/image', Image, self.on_video_update)

    def on_ui_request(self, message):
        """Process the message show request

        We have spetial `ui/message` topic where any node can send
        any message and that message will be displayed.

        By default, messages are displayed for a while and than hidden.

        Messages which match the mask `([a-zA-Z0-9_-])::(.*)` will be displayed
        permanently. Newer messages will overwrite older messages
        with the same name.

        """
        match = self.messages_named_template.match(message.data)
        self.messages.message_put(**match.groupdict())

    def on_video_update(self, data):
        """On each frame we save new picture for future rendering"""
        self.communication_since_timer = True

        image = QtGui.QImage(data.data,
                             data.width,
                             data.height,
                             QtGui.QImage.Format_RGB888)
        if self.swap_red_blue:
            image = QtGui.QImage.rgbSwapped(image)

        with self.image_lock:
            self.image = image

    def on_redraw(self):
        """Redraw interface"""
        image = None
        with self.image_lock:
            if self.image is not None:
                image = QtGui.QPixmap.fromImage(self.image)
            else:
                image = QtGui.QPixmap(640, 360)
                image.fill(QtGui.QColor(50, 50, 50))

        self.messages.messages_put((
            (self.controller.status.readable(), 'drone.status'),
            (self.controller.battery, 'drone.battery'),
        ))

        self.messages.render(image)

        self.resize(image.width(), image.height())
        self.image_box.setPixmap(image)
 
  


   
    """Homing function that checks continuously if the Euclidiean distance of marker to drone is greater or less than the setError. If greater, moves the drone closer to the marker position"""
    def Homing(self):
	"""keypressChecker is used to assure that the function only happens once on a keypress. A keypress always yields two calls normally."""
	global keypressChecker
        keypressChecker = keypressChecker + 1 
	if(keypressChecker == 2):
       	    global xdistance
            global ydistance
	    """ setError is the maximum distance the drone can be away from the marker to be considered at the marker """
            setError = 0.05
            while (CheckHypo() > setError):
		if (xdistance > 0 and ydistance > 0):
		    time.sleep(.3)
		    self.controller.send_vel(-.0005,-.0005,0,0)
		    print("GO RIGHT FORWARD")
		if (xdistance > 0 and ydistance < 0):
		    time.sleep(.3)
		    self.controller.send_vel(-.0005,.0005,0,0)
		    print("GO RIGHT BACKWARD")
		if (xdistance < 0 and ydistance > 0):
		    time.sleep(.3)
		    self.controller.send_vel(.0005,-.0005,0,0)
		    print("GO LEFT FORWARD")
		if (xdistance < 0 and ydistance < 0):
		    time.sleep(.3)
		    self.controller.send_vel(.0005,.0005,0,0)
		    print("GO LEFT BACKWARD")
	    keypressChecker = 0
	    self.controller.send_vel(0,0,0,0)
	    self.controller.land()

    def gen_keymap2(self):
        return {
            QtCore.Qt.Key.Key_M: lambda ax, e: self.controller.enable_cv(),
            QtCore.Qt.Key.Key_N: lambda ax, e: self.controller.enable_controller(),
        }



    def gen_keymap(self):
        return {
            QtCore.Qt.Key.Key_R: lambda ax, e: self.controller.reset(),
            QtCore.Qt.Key.Key_T: lambda ax, e: self.controller.takeoff(),
            QtCore.Qt.Key.Key_L: lambda ax, e: self.controller.land(),
            QtCore.Qt.Key.Key_C: lambda ax, e: self.controller.change_camera(),
            QtCore.Qt.Key.Key_M: lambda ax, e: self.controller.enable_cv(),
            QtCore.Qt.Key.Key_N: lambda ax, e: self.controller.enable_controller(),

            QtCore.Qt.Key.Key_F1: lambda ax, e: self.controller.increaseP(),
            QtCore.Qt.Key.Key_F2: lambda ax, e: self.controller.decreaseP(),

            QtCore.Qt.Key.Key_F3: lambda ax, e: self.controller.increaseI(),
            QtCore.Qt.Key.Key_F4: lambda ax, e: self.controller.decreaseI(),

            QtCore.Qt.Key.Key_F5: lambda ax, e: self.controller.increaseD(),
            QtCore.Qt.Key.Key_F6: lambda ax, e: self.controller.decreaseD(),

            QtCore.Qt.Key.Key_H: lambda ax, e: self.controller.hover(),
            QtCore.Qt.Key.Key_A: lambda ax, e: self.controller.send_vel(y=ax),
            QtCore.Qt.Key.Key_D: lambda ax, e: self.controller.send_vel(y=-ax),
            QtCore.Qt.Key.Key_W: lambda ax, e: self.controller.send_vel(x=ax),
            QtCore.Qt.Key.Key_S: lambda ax, e: self.controller.send_vel(x=-ax),
            QtCore.Qt.Key.Key_Q: lambda ax, e: self.controller.send_vel(yaw=ax),
            QtCore.Qt.Key.Key_E: lambda ax, e: self.controller.send_vel(yaw=-ax),
            QtCore.Qt.Key.Key_BracketRight: lambda ax, e: self.controller.send_vel(z=ax),
            QtCore.Qt.Key.Key_BracketLeft: lambda ax, e: self.controller.send_vel(z=-ax),
            QtCore.Qt.Key.Key_Y: lambda ax, e: self._ap_topic.publish(Empty()) if ax != 0 else None,
	    QtCore.Qt.Key.Key_J: lambda ax, e: self.Homing(),
        }

     
    def markerListener(self):
    	print("This is going on:")
    	rospy.Subscriber("visualization_marker", Marker, markerCallback)

    def droneListener(self):
    	print("This is going on too!:")
    	rospy.Subscriber("/orb/pose_scaled", PoseStamped, droneCallback)
        
	

    def keyPressEvent(self, event):
        key = event.key()

        if event.isAutoRepeat() or self.controller is None:
            return

        if key in self.keymap:
            self.keymap[key](1, event)
	

    def keyReleaseEvent(self, event):
        key = event.key()

        if event.isAutoRepeat() or self.controller is None:
            return

        if key in self.keymap and key not in self.keymap2:
            self.keymap[key](0, event)


if __name__ == '__main__':
    import sys
    import interface
    rospy.init_node('ui_node')

    rospy.loginfo('Starting user interface')

    app = QtGui.QApplication(sys.argv)
    ui = UInode()
    ui.markerListener()
    ui.droneListener()
    ui.show()
    status = app.exec_()
    ui.webcam_shutdown_pub.publish(Empty())
    sys.exit(status)
