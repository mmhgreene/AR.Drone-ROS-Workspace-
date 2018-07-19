#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""Node for logging status changes of the drone

This node will print all events that are registered by the drone controller.


Inputs
------

* /ardrone/navdata -- information about the drone state.

"""

import rospy

from utils.drone import DroneController


if __name__ == '__main__':
    rospy.init_node('drone_commands_logger', anonymous=True)

    controller = DroneController(land_on_shutdown=False)

    @controller.on_online.subscribe
    def on_online(*args, **kwargs):
        rospy.loginfo('on_online')

    @controller.on_offline.subscribe
    def on_offline(*args, **kwargs):
        rospy.loginfo('on_offline')

    @controller.on_status_change.subscribe
    def on_status_change(*args, **kwargs):
        rospy.loginfo('on_status_change')

    @controller.on_status_emergency.subscribe
    def on_status_emergency(*args, **kwargs):
        rospy.loginfo('on_status_emergency')

    @controller.on_status_initialized.subscribe
    def on_status_initialized(*args, **kwargs):
        rospy.loginfo('on_status_initialized')

    @controller.on_status_landed.subscribe
    def on_status_landed(*args, **kwargs):
        rospy.loginfo('on_status_landed')

    @controller.on_status_flying.subscribe
    def on_status_flying(*args, **kwargs):
        rospy.loginfo('on_status_flying')

    @controller.on_status_hovering.subscribe
    def on_status_hovering(*args, **kwargs):
        rospy.loginfo('on_status_hovering')

    @controller.on_status_test.subscribe
    def on_status_test(*args, **kwargs):
        rospy.loginfo('on_status_test')

    @controller.on_status_unconnected.subscribe
    def on_status_unconnected(*args, **kwargs):
        rospy.loginfo('on_status_unconnected')

    @controller.on_status_taking_off.subscribe
    def on_status_taking_off(*args, **kwargs):
        rospy.loginfo('on_status_taking_off')

    @controller.on_status_going_to_hover_mode.subscribe
    def on_status_going_to_hover_mode(*args, **kwargs):
        rospy.loginfo('on_status_going_to_hover_mode')

    @controller.on_status_looping.subscribe
    def on_status_looping(*args, **kwargs):
        rospy.loginfo('on_status_looping')

    @controller.on_status_unknown.subscribe
    def on_status_unknown(*args, **kwargs):
        rospy.loginfo('on_status_unknown')

    @controller.before_cmd_takeoff.subscribe
    def before_cmd_takeoff(*args, **kwargs):
        rospy.loginfo('before_cmd_takeoff')

    @controller.before_cmd_land.subscribe
    def before_cmd_land(*args, **kwargs):
        rospy.loginfo('before_cmd_land')

    @controller.before_cmd_reset.subscribe
    def before_cmd_reset(*args, **kwargs):
        rospy.loginfo('before_cmd_reset')

    @controller.before_cmd_hover.subscribe
    def before_cmd_hover(*args, **kwargs):
        rospy.loginfo('before_cmd_hover')

    @controller.before_cmd_vel.subscribe
    def before_cmd_vel(*args, **kwargs):
        rospy.loginfo('before_cmd_vel')

    @controller.before_cmd.subscribe
    def before_cmd(*args, **kwargs):
        rospy.loginfo('before_cmd')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
