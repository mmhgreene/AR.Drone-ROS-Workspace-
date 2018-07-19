#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""Events dispatcher allows for subscription and execution of callbacks


Usage
-----

event = Event()

@event.subscribe
def on_event(*args, **kwargs):
    print('Event fired with %s, %s' % (args, kwargs))

event.emit('Hello', word='!')
# prints `Event fired with ('Hello',), {'word': '!'}`

event.unsubscribe(on_event)

event.emit('Hello', word='!')
# No console output

"""


class Event(object):
    def __init__(self):
        """Basic event allows for subscription and execution of callbacks"""
        self.__subscribers = set()

    def subscribe(self, callback):
        """Add new callable to the event

        This callable will be executen each time this event emitted.
        The callable should not block execution.
        Each callable will be called once per event emission.
        There are no guarantees of execution order of callbacks.

        :param callback: callable, hashable object.
        :returns: callback. Thus, this method can be used as a decorator.

        """
        self.__subscribers.add(callback)
        return callback

    def unsubscribe(self, callback, fall_silently=True):
        """Removes the callback from the event callbacks list

        Raises `AttributeError` unless `fall_silently` is set to `True`
        (which is the default behavior).

        :param callback: callable, hashable object.
        :param fall_silently:
        :raises: AttributeError: the callback is not registered.

        """
        if callback in self.__subscribers:
            self.__subscribers.remove(callback)
        elif not fall_silently:
            raise AttributeError("no such callback: %s" % callback)

    def emit(self, *args, **kwargs):
        """Calls all registered callbacks"""
        for callback in self.__subscribers:
            callback(*args, **kwargs)
