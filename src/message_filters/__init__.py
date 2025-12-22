# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Message Filter Objects
======================
"""

"""Message Filter Objects."""

from bisect import insort_right
from dataclasses import dataclass
from functools import reduce
import itertools
import threading
import rclpy

from typing import Type, Union

from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time


class SimpleFilter(object):

    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        """
        Register a callback function `cb` to be called when this filter
        has output.
        The filter calls the function ``cb`` with a filter-dependent
        list of arguments,followed by the call-supplied arguments ``args``.
        """

        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))

class Subscriber(SimpleFilter):

    """
    ROS 2 subscription filter, takes identical arguments as :class:`rclpy.Subscriber`.

    This class acts as a highest-level filter, simply passing messages
    from a ROS 2 subscription through to the filters which have connected
    to it.
    """

    def __init__(
        self,
        node: Node,
        msg_type: Type,
        topic: str,
        qos_profile: Union[QoSProfile, int] = QoSProfile(depth=10),
    ) -> None:
        """
        Construct a Subscriber.

        :param node: The node to create a subscriber for.
        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param qos_profile: A QoSProfile or a history depth to apply to the
            subscription. In the case that a history depth is provided, the QoS history is
            set to KEEP_LAST, the QoS history depth is set to the value of the parameter,
            and all other QoS settings are set to their default values.
        """
        SimpleFilter.__init__(self)
        self.node = node
        self.topic = topic
        self.sub = self.node.create_subscription(
            msg_type=msg_type,
            topic=self.topic,
            callback=self.callback,
            qos_profile=qos_profile,
        )

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

    def __getattr__(self, key):
        """Serve same API as rospy.Subscriber"""
        return self.sub.__getattribute__(key)

class Cache(SimpleFilter):

    """
    Stores a time history of messages.

    Given a stream of messages, the most recent ``cache_size`` messages
    are cached in a ring buffer, from which time intervals of the cache
    can then be retrieved by the client. The ``allow_headerless``
    option specifies whether to allow storing headerless messages with
    current ROS time instead of timestamp. You should avoid this as
    much as you can, since the delays are unpredictable.
    """

    def __init__(self, f, cache_size=1, allow_headerless=False):
        SimpleFilter.__init__(self)
        self.connectInput(f)
        self.cache_size = cache_size
        # Array to store messages
        self.cache_msgs = []
        # Array to store msgs times, auxiliary structure to facilitate
        # sorted insertion
        self.cache_times = []
        # Whether to allow storing headerless messages with current ROS
        # time instead of timestamp.
        self.allow_headerless = allow_headerless

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.add)

    def add(self, msg):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_cache')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                msg_filters_logger.warn("can not use message filters messages "
                                        "without timestamp infomation when "
                                        "'allow_headerless' is disabled. "
                                        "auto assign ROSTIME to headerless "
                                        "messages once enabling constructor "
                                        "option of 'allow_headerless'.")

                return

            stamp = ROSClock().now()
        else:
            stamp = msg.header.stamp
            if not hasattr(stamp, 'nanoseconds'):
                stamp = Time.from_msg(stamp)
        # Insert sorted
        self.cache_times.append(stamp)
        self.cache_msgs.append(msg)

        # Implement a ring buffer, discard older if oversized
        if (len(self.cache_msgs) > self.cache_size):
            del self.cache_msgs[0]
            del self.cache_times[0]

        # Signal new input
        self.signalMessage(msg)

    def getInterval(self, from_stamp, to_stamp):
        """Query the current cache content between from_stamp to to_stamp."""
        assert from_stamp <= to_stamp

        return [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                if from_stamp <= time <= to_stamp]

    def getElemAfterTime(self, stamp):
        """Return the oldest element after or equal the passed time stamp."""
        newer = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time >= stamp]
        if not newer:
            return None
        return newer[0]

    def getElemBeforeTime(self, stamp):
        """Return the newest element before or equal the passed time stamp."""
        older = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time <= stamp]
        if not older:
            return None
        return older[-1]

    def getLastestTime(self):
        """Return the newest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[-1]

    def getOldestTime(self):
        """Return the oldest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[0]

    def getLast(self):
        if self.getLastestTime() is None:
            return None
        return self.getElemAfterTime(self.getLastestTime())


class Chain(SimpleFilter):
    """
    Chains a dynamic number of simple filters together.

    Allows retrieval of filters by index after they are added.

    The Chain filter provides a container for simple filters.
    It allows you to store an N-long set of filters inside a single
    structure, making it much easier to manage them.

    Adding filters to the chain is done by adding shared_ptrs of them
    to the filter. They are automatically connected to each other
    and the output of the last filter in the chain is forwarded
    to the callback you've registered with Chain::registerCallback.
    """

    @dataclass
    class FilterInfo:
        message_filter: any
        connection_callback_index: int

    def __init__(self, message_filter=None):
        SimpleFilter.__init__(self)

        self.incoming_connection = None

        if message_filter is not None:
            self.connectInput(message_filter)

        self._message_filters: dict[int, Chain.FilterInfo] = {}

    def connectInput(self, message_filter):
        if self.incoming_connection is not None:
            raise RuntimeError('Already connected')
        self.incoming_connection = message_filter.registerCallback(self.add)

    def add(self, message):
        if self._message_filters:
            self._message_filters[0].message_filter.add(message)
        else:
            self.signalMessage(message)

    def addFilter(self, message_filter):
        new_filter_index = len(self._message_filters)
        last_filter_index = new_filter_index - 1

        self._message_filters[new_filter_index] = Chain.FilterInfo(
            message_filter=message_filter,
            connection_callback_index=message_filter.registerCallback(self.signalMessage),
        )

        if last_filter_index >= 0:
            last_filter = self._message_filters[last_filter_index].message_filter
            callback_index = self._message_filters[last_filter_index].connection_callback_index
            last_filter.callbacks.pop(callback_index)

            self._message_filters[last_filter_index].connection_callback_index = \
                last_filter.registerCallback(message_filter.add)

    def getFilter(self, index: int):
        return self._message_filters[index].message_filter


class TimeSynchronizer(SimpleFilter):

    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer listens
    on multiple input message filters ``fs``, and invokes the callback when
    it has a collection of messages with matching timestamps.

    The signature of the callback function is:

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``fs``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(self, fs, queue_size):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(fs, self.queues))]

    def add(self, msg, my_queue, my_queue_index=None):
        self.lock.acquire()
        stamp = Time.from_msg(msg.header.stamp)
        my_queue[stamp.nanoseconds] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # common is the set of timestamps that occur in all queues
        common = reduce(set.intersection, [set(q) for q in self.queues])
        signaled_time = None
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [q[t] for q in self.queues]
            self.signalMessage(*msgs)
            for q in self.queues:
                del q[t]
            signaled_time = t
            break

        # for consistency with the C++ implementation:
        #     Delete all stored messages with a timestamp
        #     older than the time of the latest signaled time
        if signaled_time is not None:
            for queue in self.queues:
                for stamp in list(queue.keys()):
                    if stamp < signaled_time:
                        del queue[stamp]

        self.lock.release()

class ApproximateTimeSynchronizer(TimeSynchronizer):

    """
    Approximately synchronizes messages by their timestamps.

    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters
    by the timestamps contained in their messages' headers. The API is the same
    as TimeSynchronizer except for an extra `slop` parameter in the constructor
    that defines the delay (in seconds) with which messages can be synchronized.
    The ``allow_headerless`` option specifies whether to allow storing
    headerless messages with current ROS time instead of timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    """

    def __init__(self, fs, queue_size, slop, allow_headerless=False):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = Duration(seconds=slop)
        self.allow_headerless = allow_headerless

    def add(self, msg, my_queue, my_queue_index=None):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_approx')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                msg_filters_logger.warn("can not use message filters for "
                              "messages without timestamp infomation when "
                              "'allow_headerless' is disabled. auto assign "
                              "ROSTIME to headerless messages once enabling "
                              "constructor option of 'allow_headerless'.")
                return

            stamp = ROSClock().now()
        else:
            stamp = msg.header.stamp
            if not hasattr(stamp, 'nanoseconds'):
                stamp = Time.from_msg(stamp)
            # print(stamp)
        self.lock.acquire()
        my_queue[stamp.nanoseconds] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if my_queue_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:my_queue_index] + \
                self.queues[my_queue_index+1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = Duration(nanoseconds=abs(s - stamp.nanoseconds))
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append(((Time(nanoseconds=s,
                                   clock_type=stamp.clock_type)), stamp_delta))
            if not topic_stamps:
                self.lock.release()
                return
            topic_stamps = sorted(topic_stamps, key=lambda x: x[1])
            stamps.append(topic_stamps)
        for vv in itertools.product(*[list(zip(*s))[0] for s in stamps]):
            vv = list(vv)
            # insert the new message
            if my_queue_index is not None:
                vv.insert(my_queue_index, stamp)
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t.nanoseconds not in q]) == 0) ):
                msgs = [q[t.nanoseconds] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t.nanoseconds]
                break  # fast finish after the synchronization
        self.lock.release()
