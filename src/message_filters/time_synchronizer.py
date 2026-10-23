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
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
from rclpy.type_support import MsgT

from message_filters.message_traits import get_time_from_message_header     # noqa: I100
from message_filters.simple_filter import SimpleFilter
from message_filters.utilities import generate_deprecation_warning


class TimeSynchronizer(SimpleFilter):
    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer listens
    on multiple input message filters ``input_filters``, and invokes the callback when
    it has a collection of messages with matching timestamps.

    The signature of the callback function is:

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``input_filters``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(
        self,
        input_filters: Optional[Sequence[SimpleFilter]] = None,
        queue_size: Optional[int] = None,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
        *,
        fs: Optional[Sequence[SimpleFilter]] = None,
    ) -> None:
        """
        Construct a TimeSynchronizer instance.

        :param input_filters: A sequence collection containing upstream message filter channels.
        :type input_filters: Optional[Sequence[SimpleFilter]]
        :param queue_size: The capacity of the message queue for each filter pipeline.
        :type queue_size: Optional[int]
        :param time_getter: Callable mechanism used to extract the Time footprint from messages.
        :type time_getter: Callable[[MsgT], Optional[Time]]
        :param fs: Deprecated argument alias for input_filters.
        :type fs: Optional[Sequence[SimpleFilter]]
        :raises TypeError: If neither 'input_filters' nor 'fs' arguments are provided.
        """
        SimpleFilter.__init__(self)

        if fs is not None:
            generate_deprecation_warning(
                old_arg_name='fs',
                new_arg_name='input_filters',
            )

        if not input_filters and not fs:
            raise TypeError(
                'TimeSynchronizer.__init__ '
                'missing required argument: "input_filters"'
            )

        self.connectInput(input_filters if input_filters is not None else fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()
        self.time_getter = time_getter

    def connectInput(
        self,
        input_filters: Optional[Sequence[SimpleFilter]] = None,
        *,
        fs: Optional[Sequence[SimpleFilter]] = None,
    ):
        """
        Establish upstream connections to all targeting input filter pipes.

        Allocates distinct internal message storage blocks and attaches a shared
        callback registry token across all provided filters.

        :param input_filters: A sequence collection containing upstream message filter channels.
        :type input_filters: Optional[Sequence[SimpleFilter]]
        :param fs: Deprecated argument alias for input_filters.
        :type fs: Optional[Sequence[SimpleFilter]]
        :raises TypeError: If neither 'input_filters' nor 'fs' arguments are provided.
        """
        if fs is not None:
            generate_deprecation_warning(
                old_arg_name='fs',
                new_arg_name='input_filters',
            )

        if not input_filters and not fs:
            raise TypeError(
                'TimeSynchronizer.connectInput '
                'missing one of required arguments: "input_filters"'
            )

        self.queues = [{} for f in input_filters]
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(input_filters, self.queues))]

    def add(
        self,
        msg: MsgT,
        msg_buffer: Optional[dict] = None,          # Replaces 'my_queue'
        msg_buffer_index: Optional[int] = None,     # Replaces 'my_queue_index'
        *,
        my_queue: Optional[dict] = None,            # Replaced by 'msg_buffer'
        my_queue_index: int = None,                 # Replaced by 'msg_buffer_index'
    ):
        """
        Add a new incoming message to the internal buffer and attempt synchronization.

        Extracts the timestamp from the message, inserts it into the specified buffer,
        maintains the queue size limit, and triggers callbacks if a set of synchronized
        messages with exact matching timestamps is found across all queues.

        :param msg: The incoming message object to be synchronized.
        :type msg: MsgT
        :param msg_buffer: The target dictionary buffer storing messages for this filter channel.
        :type msg_buffer: Optional[dict]
        :param msg_buffer_index: The index of the current message buffer channel.
        :type msg_buffer_index: Optional[int]
        :param my_queue: Deprecated alias for msg_buffer.
        :type my_queue: Optional[dict]
        :param my_queue_index: Deprecated alias for msg_buffer_index.
        :type my_queue_index: Optional[int]
        :raises TypeError: If both 'msg_buffer' and 'my_queue' are missing.
        :raises ValueError: If the timestamp cannot be extracted from the message header.
        """
        # deprecation checks
        if my_queue is not None:
            generate_deprecation_warning(
                old_arg_name='my_queue',
                new_arg_name='my_buffer'
            )
        if my_queue_index is not None:
            generate_deprecation_warning(
                old_arg_name='my_queue_index',
                new_arg_name='msg_buffer_index',
            )

        if msg_buffer is None and my_queue is None:
            raise TypeError('TimeSynchronizer.add() missing a positional argument "msg_buffer"')

        msg_buffer = msg_buffer if msg_buffer is not None else my_queue
        stamp = self.time_getter(msg)
        if stamp is None:
            raise ValueError(
                'time_getter could not extract a timestamp from the message; '
                'use ApproximateTimeSynchronizer with "allow_headerless=True" '
                'or provide a custom "time_getter"')
        self.lock.acquire()
        msg_buffer[stamp.nanoseconds] = msg
        while len(msg_buffer) > self.queue_size:
            del msg_buffer[min(msg_buffer)]
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
    as TimeSynchronizer except for an extra `tolerance_seconds` parameter in the constructor
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
        input_filters: Optional[Sequence[SimpleFilter]] = None,  # Replaces 'f'.
        queue_size: Optional[int] = None,
        tolerance_seconds: Optional[int] = None,   # Replaces 'slop'
        queue_offset=False,
        allow_headerless=False,
        sync_arrival_time=False,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
        *,
        # These should be removed in future releases
        fs: Optional[Sequence[SimpleFilter]] = None,   # Replaced by 'input_filters'
        slop: Optional[int] = None,    # Replaced by 'tolerance_seconds'
    ):
        # deprecation checks
        if fs is not None:
            generate_deprecation_warning(
                old_arg_name='fs',
                new_arg_name='input_filters',
            )
        if slop is not None:
            generate_deprecation_warning(
                old_arg_name='slop',
                new_arg_name='tolerance_seconds',
            )

        # arguments list completeness check
        if input_filters is None and fs is None:
            raise TypeError(
                'ApproximateTimeSynchronizer.__init__() '
                'misses a required argument: "input_filters"'
            )
        if queue_size is None:
            raise TypeError(
                'ApproximateTimeSynchronizer.__init__() '
                'misses a required argument: "queue_size"'
            )
        if tolerance_seconds is None and slop is None:
            raise TypeError(
                'ApproximateTimeSynchronizer.__init__() '
                'misses a required argument: "tolerance_seconds"'
            )

        input_filters = input_filters if input_filters is not None else fs
        tolerance_seconds = tolerance_seconds if tolerance_seconds is not None else slop

        TimeSynchronizer.__init__(self, input_filters, queue_size)
        self.tolerance_seconds = Duration(seconds=tolerance_seconds)
        self.allow_headerless = allow_headerless
        self.queue_offset = queue_offset
        self.sync_arrival_time = sync_arrival_time
        self.time_getter = time_getter

    def add(
        self,
        msg: MsgT,
        msg_buffer: Optional[dict] = None,          # Replaces 'my_queue'
        msg_buffer_index: Optional[int] = None,     # Replaces 'my_queue_index'
        *,
        my_queue: Optional[dict] = None,            # Replaced by 'msg_buffer'
        my_queue_index: int = None,                 # Replaced by 'msg_buffer_index'
    ):
        """
        Add a new incoming message to the internal buffer and attempt approximate synchronization.

        Evaluates the timestamp context (accounting for headerless fallbacks, arrival times,
        and custom queue offsets), inserts the tracking event into the message pipeline,
        and evaluates candidate tuples from interconnected filters against the defined
        tolerance_seconds threshold.

        :param msg: The incoming message object to be synchronized.
        :type msg: MsgT
        :param msg_buffer: The target dictionary buffer storing messages for this filter channel.
        :type msg_buffer: Optional[dict]
        :param msg_buffer_index: The index of the current message buffer channel.
        :type msg_buffer_index: Optional[int]
        :param my_queue: Deprecated alias for msg_buffer.
        :type my_queue: Optional[dict]
        :param my_queue_index: Deprecated alias for msg_buffer_index.
        :type my_queue_index: Optional[int]
        :raises TypeError: If both 'msg_buffer' and 'my_queue' are missing.
        """
        # deprecation checks
        if my_queue is not None:
            generate_deprecation_warning(
                old_arg_name='my_queue',
                new_arg_name='my_buffer',
            )
        if my_queue_index is not None:
            generate_deprecation_warning(
                old_arg_name='my_queue_index',
                new_arg_name='msg_buffer_index',
            )

        if msg_buffer is None and my_queue is None:
            raise TypeError('TimeSynchronizer.add() missing a positional argument "msg_buffer"')

        msg_buffer = msg_buffer if msg_buffer is not None else my_queue
        msg_buffer_index = msg_buffer_index if msg_buffer_index is not None else my_queue_index

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
        if msg_buffer_index is not None and self.queue_offset:
            new_timestamp -= self.queue_offset[msg_buffer_index]
        self.lock.acquire()
        msg_buffer[new_timestamp] = msg
        while len(msg_buffer) > self.queue_size:
            del msg_buffer[min(msg_buffer)]
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if msg_buffer_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:msg_buffer_index] + \
                self.queues[msg_buffer_index+1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = Duration(nanoseconds=abs(s - new_timestamp))
                if stamp_delta > self.tolerance_seconds:
                    continue  # exceeds tolerance hence dropped
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
            if msg_buffer_index is not None:
                vv.insert(msg_buffer_index, stamp)
            qt = list(zip(self.queues, vv))
            if (((max(vv) - min(vv)) < self.tolerance_seconds) and
               (len([1 for q, t in qt if t.nanoseconds not in q]) == 0)):
                msgs = [q[t.nanoseconds] for q, t in qt]
                self.signalMessage(*msgs)
                for q, t in qt:
                    del q[t.nanoseconds]
                break  # fast finish after the synchronization
        self.lock.release()
