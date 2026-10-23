# Copyright 2026, Open Source Robotics Foundation, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from functools import reduce
import itertools
import threading
from typing import Callable, Optional

import rclpy
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
from rclpy.type_support import MsgT

from message_filters.message_traits import get_time_from_message_header     # noqa: I100
from message_filters.simple_filter import SimpleFilter


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

    def __init__(
        self,
        fs,
        queue_size,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
    ):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()
        self.time_getter = time_getter

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(fs, self.queues))]

    def add(self, msg, my_queue, my_queue_index=None):
        stamp = self.time_getter(msg)
        if stamp is None:
            raise ValueError(
                'time_getter could not extract a timestamp from the message; '
                'use ApproximateTimeSynchronizer with "allow_headerless=True" '
                'or provide a custom "time_getter"')
        self.lock.acquire()
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
    The ``queue_offset`` option allow to have temporal offset between subscribers
    , define as a list of offset int in nanoseconds.
    The ``allow_headerless`` option specifies whether to allow storing
    headerless messages with current ROS time instead of timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    The ```sync_arrival_time``` option enables synchronizing incoming messages
    with the arrival ROS time instead of the message timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    """

    def __init__(
        self,
        fs,
        queue_size,
        slop,
        queue_offset=False,
        allow_headerless=False,
        sync_arrival_time=False,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
    ):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = Duration(seconds=slop)
        self.allow_headerless = allow_headerless
        self.queue_offset = queue_offset
        self.sync_arrival_time = sync_arrival_time
        self.time_getter = time_getter

    def add(self, msg, my_queue, my_queue_index=None):
        stamp = None if self.sync_arrival_time else self.time_getter(msg)
        if stamp is None:
            if not self.allow_headerless and not self.sync_arrival_time:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_approx')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                msg_filters_logger.warning('can not use message filters messages '
                                           'without timestamp infomation when '
                                           '"allow_headerless" is disabled. '
                                           'auto assign ROSTIME to headerless '
                                           'messages once enabling constructor '
                                           'option of "allow_headerless".')
                return

            stamp = ROSClock().now()
        new_timestamp = stamp.nanoseconds
        if my_queue_index is not None and self.queue_offset:
            new_timestamp -= self.queue_offset[my_queue_index]
        self.lock.acquire()
        my_queue[new_timestamp] = msg
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
                stamp_delta = Duration(nanoseconds=abs(s - new_timestamp))
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append(((Time(nanoseconds=s,
                                           clock_type=stamp.clock_type)),
                                    stamp_delta))
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
            if (((max(vv) - min(vv)) < self.slop) and
               (len([1 for q, t in qt if t.nanoseconds not in q]) == 0)):
                msgs = [q[t.nanoseconds] for q, t in qt]
                self.signalMessage(*msgs)
                for q, t in qt:
                    del q[t.nanoseconds]
                break  # fast finish after the synchronization
        self.lock.release()
