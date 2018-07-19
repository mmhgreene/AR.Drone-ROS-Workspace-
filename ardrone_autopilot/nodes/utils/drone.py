#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""Helper classes for controlling the drone.

The `DroneController` class provides basic api for controlling the drone,
dispatching events etc.


Usage
-----

controller = DroneController()

@controller.on_status_landed.subscribe
def on_status_landed(controller):
    print('drone is landed')
    controller.takeoff()

@controller.on_status_hovering.subscribe
on_status_hovering(controller):
    print('drone is hovering')
    controller.land()

controller.takeoff()

"""

from subprocess import call
from multiprocessing import RLock
from copy import copy
from datetime import datetime, timedelta

import rospy

from std_msgs.msg import Empty
from std_msgs.msg import String
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist

from .events import Event

changing = True

class DroneStatus(object):
    statuses = {
        # After the `reset` command.
        # This state is also set by the driver when it things that
        # connection is lost. I'd not perform any actions on this state.
        0: 'emergency',

        # A wired one. I've never seen this state actually set.
        1: 'initialized',

        # Drone is on the ground.
        2: 'landed',

        # Drone is flying. Its velocity is non-zero.
        3: 'flying',

        # Drone is hovering, e.g. its trying to keep its position constant.
        4: 'hovering',

        # No idea what this is.
        5: 'test',

        # Different guides says different thisngs about this state.
        # I've never seen this state actually set. So better don't use it.
        6: 'unconnected',

        # This state is set after the drone receives the `takeoff` command.
        # However, the offitial documentation calls this state
        # 'going_to_hover_mode' without any explanation.
        7: 'taking_off',

        # This state is set after the drone receives the `hover` command.
        # However, the offitial documentation calls this state
        # 'landing' without any explanation.
        8: 'going_to_hover_mode',

        # This state is set when the drone is going to land.
        # It's probably set when the drone executes an animation.
        # Never tested it, though.
        9: 'looping',


        # Additional states
        # -----------------

        # The drone is offline. This state is set
        # by the `DroneController` class. You can'r receive negative values
        # from the drone itself.
        -1: 'unknown',
    }

    codes = dict(zip(statuses.values(), statuses.keys()))

    def __init__(self, status):
        """Class that describels drone status

        :param status: A status code, a status string, or another status object

        """
        self.__status = self._any_to_code(status)

    @classmethod
    def _any_to_code(cls, status):
        """Converts any valid status description to a status code

        For any object which is a valid status code number, a status string,
        or another status object, returns a valid status code number.

        :param status: A status code, a status string, or another status object
        :return: A valid numeric status code
        :throws RuntimeError: passed object is not a valid status code,
          status string, or another status object

        """
        if hasattr(status, 'status'):
            status = status.status

        if status in cls.codes:
            return cls.codes.get(status, -1)
        elif status in cls.statuses:
            return status
        else:
            raise RuntimeError("unknown status %s" % status)

    @property
    def status(self):
        """Read-only variable accessor"""
        return self.__status

    def __eq__(self, other):
        return self.__status == self._any_to_code(other)

    def __ne__(self, other):
        return not self == other

    def __int__(self):
        return self.__status

    def __index__(self):
        return self.__status

    def __str__(self):
        return self.statuses[self.__status]

    def readable(self):
        """Returns the human-readable version of the state string"""
        s = self.statuses[self.__status]
        return s.replace('_', ' ').capitalize()

    def __repr__(self):
        return 'DroneStatus("%s")' % str(self)


class DroneController(object):
    def __init__(self,
                 land_topic_name='/ardrone/land',
                 takeoff_topic_name='/ardrone/takeoff',
                 reset_topic_name='/ardrone/reset',
                 cmd_topic_name='/cmd_vel',
                 navdata_topic_name='/ardrone/navdata',
                 change_camera_topic_name = '/ardrone/togglecam',
                 enable_controller_topic_name = 'controller/enable',
                 enable_cv_topic_name = 'cv/enable',
                 pid_decrease_topic_name = 'pid/decrease',
                 pid_increase_topic_name = 'pid/increase',
                 queue_size=6,
                 land_on_shutdown=True,
                 offline_timeout=500):
        """A simple interface for the ArDrone autonomy driver

        This class provides simple takeoff/land/reset/set_velocity
        functionality.

        On init, this class will subscribe on appropriate topics
        so make sure that you're instancing this class
        after the node was initialized.

        :param land_topic_name: Name of the topic for land commands.
            Default is '/ardrone/land'.
        :param takeoff_topic_name: Name of the topic for takeoff commands.
            Default is '/ardrone/takeoff'.
        :param reset_topic_name: Name of the topic for reset commands.
            Default is '/ardrone/reset'.
        :param cmd_topic_name: Name of the topic for velocity update commands.
            Default is '/cmd_vel'.
        :param navdata_topic_name: Name of the topic for drone status messages.
            Default is '/ardrone/navdata'.
        :param queue_size: Default queue size.
            Default is 5.
        :param land_on_shutdown: Flag indicates whether the landing command
            should be send if the node gets shutdown. Default is True.
        :param offline_timeout: Consider dorne to be offline if we had
            no messages for more than this time (in milliseconds).
            Default is 500.


        Events
        ------

        * on_online -- executed whenever the drone gets online.
          See the `offline_timeout` parameter.
          This event will be emittet just after a new data received
          from the drone. This mean that you won't have any
          usefull data in the `DroneController` instance at the moment.
          The drone state and other variables of the class
          will be populated after.
        * on_offline -- executed whenever the drone gets offline.
            See the `offline_timeout` parameter.
            This eventt is emitted at last, after `on_state_change` and all
            other events.

        * on_status_change -- executed whenever the drone changes its status.

        * on_status_emergency -- executed whenever the drone
          changes its status to `emergency`.
        * on_status_initialized -- executed whenever the drone
          changes its status to `initialized`.
        * on_status_landed -- executed whenever the drone
          changes its status to `landed`.
        * on_status_flying -- executed whenever the drone
          changes its status to `flying`.
        * on_status_hovering -- executed whenever the drone
          changes its status to `hovering`.
        * on_status_test -- executed whenever the drone
          changes its status to `test`.
        * on_status_unconnected -- executed whenever the drone
          changes its status to `unconnected`.
        * on_status_taking_off -- executed whenever the drone
          changes its status to `taking_off`.
        * on_status_going_to_hover_mode -- executed whenever the drone
          changes its status to `going_to_hover_mode`.
        * on_status_looping -- executed whenever the drone
          changes its status to `looping`.
        * on_status_unknown -- executed whenever the drone
          changes its status to `unknown`.

        * before_cmd_takeoff -- executed whenever the `takeoff` command is sent
          to the drone.
        * before_cmd_land -- executed whenever the `land` command is sent
          to the drone.
        * before_cmd_reset -- executed whenever the `reset` command is sent
          to the drone.
        * before_cmd_hover -- executed whenever the `hover` command is sent
          to the drone.
        * before_cmd_vel -- executed whenever the `vel` command is sent
          to the drone.
        * before_cmd -- executed whenever new velocity message is sent
          to the drone.

        """
        self._last_message_time = datetime(1900, 1, 1)
        self._offline_timeout = timedelta(milliseconds=offline_timeout)
        self.__is_online = False

        rospy.Timer(rospy.Duration(offline_timeout / 1000.0),
                    self._check_online)
        self._check_online()

        self.__prev_command = Twist()
        self.__status = DroneStatus('unknown')
        self.__battery = None

        rospy.Subscriber(navdata_topic_name, Navdata, self._on_navdata)

        self._land_topic = rospy.Publisher(land_topic_name, Empty,
                                           queue_size=queue_size)
        self._takeoff_topic = rospy.Publisher(takeoff_topic_name, Empty,
                                              queue_size=queue_size)
        self._reset_topic = rospy.Publisher(reset_topic_name, Empty,
                                            queue_size=queue_size)

        self._cmd_topic = rospy.Publisher(cmd_topic_name, Twist,
                                          queue_size=queue_size)

        self._change_camera_topic = rospy.Publisher(change_camera_topic_name, Empty,
                                                   queue_size=queue_size)
	self._enable_controller_topic = rospy.Publisher(enable_controller_topic_name, Empty,
                                                   queue_size=queue_size)
	self._enable_cv_topic = rospy.Publisher(enable_cv_topic_name, Empty,
                                                   queue_size=1)
	self.pid_increase_topic = rospy.Publisher(pid_increase_topic_name, String,
                                                   queue_size=5)
	self.pid_decrease_topic = rospy.Publisher(pid_decrease_topic_name, String,
                                                   queue_size=5)


        if land_on_shutdown:
            # Land the drone if this node is disabled
            rospy.on_shutdown(self.land)

        self.on_online = Event()
        self.on_offline = Event()

        self.on_status_change = Event()

        self.on_status_emergency = Event()
        self.on_status_initialized = Event()
        self.on_status_landed = Event()
        self.on_status_flying = Event()
        self.on_status_hovering = Event()
        self.on_status_test = Event()
        self.on_status_unconnected = Event()
        self.on_status_taking_off = Event()
        self.on_status_going_to_hover_mode = Event()
        self.on_status_looping = Event()
        self.on_status_unknown = Event()

        self.before_cmd_takeoff = Event()
        self.before_cmd_land = Event()
        self.before_cmd_reset = Event()
        self.before_cmd_hover = Event()
        self.before_cmd_vel = Event()
        self.before_cmd = Event()
        self.before_cmd_change_camera = Event()

        self.before_cmd_enable_cv = Event()
        self.before_cmd_enable_controller = Event()
        self.before_cmd_pid_decrease = Event()
        self.before_cmd_pid_increase = Event()

    @property
    def status(self):
        """Read-only variable accessor"""
        return self.__status

    @property
    def battery(self):
        """Read-only variable accessor"""
        return self.__battery

    @property
    def is_online(self):
        """Read-only variable accessor"""
        return self.__is_online

    def _set_status(self, status):
        if self.__status != status:
            self.__status = status

            self.on_status_change.emit(self, status)
            if status == 'emergency':
                self.on_status_emergency.emit(self)
            if status == 'initialized':
                self.on_status_initialized.emit(self)
            if status == 'landed':
                self.on_status_landed.emit(self)
            if status == 'flying':
                self.on_status_flying.emit(self)
            if status == 'hovering':
                self.on_status_hovering.emit(self)
            if status == 'test':
                self.on_status_test.emit(self)
            if status == 'unconnected':
                self.on_status_unconnected.emit(self)
            if status == 'taking_off':
                self.on_status_taking_off.emit(self)
            if status == 'going_to_hover_mode':
                self.on_status_going_to_hover_mode.emit(self)
            if status == 'looping':
                self.on_status_looping.emit(self)
            if status == 'unknown':
                self.on_status_unknown.emit(self)

    def _check_online(self, *args, **kwargs):
        is_online = (datetime.now() - self._last_message_time
                     < self._offline_timeout)

        if self.__is_online and not is_online:
            self.__battery = None
            self.__is_online = is_online
            self.__prev_command = Twist()
            self._set_status(DroneStatus('unknown'))
            self.on_offline.emit(self)
        elif not self.__is_online and is_online:
            self.__is_online = is_online
            self.on_online.emit(self)

    def _on_navdata(self, navdata):
        """Update drone status on navdata receive"""
        self._last_message_time = datetime.now()
        self._check_online()  # We want to emit the `online` signal first
        self.__battery = navdata.batteryPercent
        self._set_status(DroneStatus(navdata.state))

    def takeoff(self):
        """Send the takeoff signal"""
        self.before_cmd_takeoff.emit(self)
        if self.__status == 'landed':
            self._takeoff_topic.publish(Empty())

    def land(self):
        """Send the land signal"""
        self.before_cmd_land.emit(self)
        self._land_topic.publish(Empty())

    def enable_controller(self):
	"""Enables autopilot controller"""
	self.before_cmd_enable_controller.emit(self)
	self._enable_controller_topic.publish(Empty())

    def enable_cv(self):
	"""Enables target tracking"""
	self.before_cmd_enable_cv.emit(self)
	self._enable_cv_topic.publish(Empty())


    def increaseP(self):
	"""Enables target tracking"""
	self.before_cmd_pid_increase.emit(self)
	self.pid_increase_topic.publish(String('p'))
    def increaseI(self):
	"""Enables target tracking"""
	self.before_cmd_pid_increase.emit(self)
	self.pid_increase_topic.publish(String('i')) 
    def increaseD(self):
	"""Enables target tracking"""
	self.before_cmd_pid_increase.emit(self)
	self.pid_increase_topic.publish(String('d'))
    
    def decreaseP(self):
	"""Enables target tracking"""
	self.before_cmd_pid_decrease.emit(self)
	self.pid_decrease_topic.publish(String('p'))
    def decreaseI(self):
	"""Enables target tracking"""
	self.before_cmd_pid_decrease.emit(self)
	self.pid_decrease_topic.publish(String('i'))
    def decreaseD(self):
	"""Enables target tracking"""
	self.before_cmd_pid_decrease.emit(self)
	self.pid_decrease_topic.publish(String('d'))


    def change_camera(self):
        global changing
        """Send the change camera signal"""
        
        if (changing == True):
                call("rosservice call /ardrone/togglecam", shell=True)
                changing = False
                print("CHANGE CAMERA")
        else:
                changing = True
        self.before_cmd_change_camera.emit(self)
        self._change_camera_topic.publish(Empty())

    def reset(self, force=False):
        """Send the reset signal

        Warning: the reset signal causes immediate engine shutdown.
        Therefore the land sygnal is sent instead of reset while flying.
        Setting `force` flag cancels this behavior.

        """
        if (self.__status in ['flying', 'hovering',
                              'taking_off', 'going_to_hover_mode',
                              'looping'] and
                not force):
            self.land()
        else:
            self.before_cmd_reset.emit(self)
            self._reset_topic.publish(Empty())

    def send_twist(self, twist):
        """Send the passed `Twist` message instance to the drone"""
        self.before_cmd.emit(twist)
        self._cmd_topic.publish(twist)
        self.__prev_command = twist

    def hover(self):
        """Send the hover signal

        This will kill all velocities and enble the drone to autohover.

        """
        self.before_cmd_hover.emit(self)
        self.send_twist(Twist())

    def send_vel(self, x=None, y=None, z=None, yaw=None):
        """Send new velocities to the drone

        This method recovers previously sent command in order to
        keep values that are not provided (e.g. None).
        Such behaviour can cause troubles in multi-threaded environments.
        Thus, is better to set all prameters explicitly
        in multi-threaded environments.

        """
        self.before_cmd_vel.emit(self, x, y, z, yaw)
        twist = copy(self.__prev_command)
        if x is not None:
            twist.linear.x = x
        if y is not None:
            twist.linear.y = y
        if z is not None:
            twist.linear.z = z
        if yaw is not None:
            twist.angular.z = yaw
        self.send_twist(twist)
