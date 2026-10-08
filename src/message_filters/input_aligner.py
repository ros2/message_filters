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

"""Input aligner for synchronizing messages from multiple sources based on their timestamps."""

from dataclasses import dataclass
import heapq
import threading
import typing as tp

from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.type_support import MsgT

from .simple_filter import SimpleFilter


@dataclass
class QueueStatus:
    """
    Structure representing the current operational status of an event queue.

    Attributes
    ----------
        active (bool): Flag indicating whether the queue is currently active.
        queue_size (int): The current number of elements waiting in the queue.
        msgs_processed (int): Total number of messages successfully processed from this queue.
        msgs_dropped (int): Total number of messages dropped due to outdated timestamps.

    """

    active: bool
    queue_size: int
    msgs_processed: int
    msgs_dropped: int


def _ros_zero_time() -> Time:
    """
    Generate a ROS 2 Time instance initialized to zero.

    :return: A zero-valued Time instance.
    :rtype: Time
    """
    return Time.from_msg(TimeMsg())


def _ros_max_time() -> Time:
    """
    Generate a ROS 2 Time instance representing the maximum possible timestamp value.

    Uses the maximum 64-bit signed integer value for nanoseconds.

    :return: A max-valued Time instance.
    :rtype: Time
    """
    return Time(
        nanoseconds=9223372036854775807,
        clock_type=_ros_zero_time().clock_type,
    )


class InputAligner:
    """
    Align N inputs by timestamp and forward each to its own output signal.

    Unlike a single-output filter, ``InputAligner`` exposes one signal per
    input, so it does not extend :class:`SimpleFilter`. Register downstream
    callbacks via :meth:`registerCallback` with an explicit ``index``.
    """

    class _EventQueue:
        """
        Internal time-ordered priority queue.

        Used for tracking incoming messages of a specific source.
        """

        def __init__(self) -> None:
            """Construct an internal _EventQueue instance."""
            self.events: list[tuple[Time, int, tp.Any]] = []
            self.next_ts: Time = _ros_max_time()
            self.period: Duration = Duration(seconds=0)
            self.active: bool = False
            self.msgs_processed: int = 0
            self.msgs_dropped: int = 0
            self.seq_id: int = 0

        def push(self, stamp: Time, msg: tp.Any) -> None:
            """
            Push a message onto the priority queue ordered by its timestamp.

            seq_id is a strictly-increasing tie-breaker so messages with
            equal timestamps stay orderable (heap items are tuples).

            :param stamp: The source timestamp of the message.
            :param msg: The actual incoming message object.
            """
            heapq.heappush(self.events, (stamp, self.seq_id, msg))
            self.seq_id += 1

        def first_timestamp(self) -> Time:
            """
            Retrieve the timestamp of the earliest message in the queue.

            If the queue contains messages, it predicts
            the next expected timestamp based on the period.
            If the queue is empty but active, it returns
            the previously calculated next expected timestamp.

            :return: The timestamp of the first message or the estimated next timestamp.
            :rtype: Time
            """
            if self.events:
                first_ts = self.events[0][0]
                self.next_ts = first_ts + self.period
                self.active = True
                return first_ts
            if self.active:
                return self.next_ts
            return _ros_max_time()

        def pop_first(self) -> tuple[Time, tp.Any]:
            """
            Pop the earliest message out of the priority queue.

            Increments the counter for processed messages.

            :return: A tuple containing the message timestamp and the message payload itself.
            :rtype: tuple[Time, tp.Any]
            """
            stamp, _, msg = heapq.heappop(self.events)
            self.msgs_processed += 1
            return stamp, msg

        def msg_dropped(self) -> None:
            """Increment the counter for dropped messages."""
            self.msgs_dropped += 1

        def set_period(self, period: Duration) -> None:
            """
            Set the expected arrival period for messages in this queue.

            :param period: The time duration representing the sensor or data period.
            """
            self.period = period

        def set_active(self, active: bool) -> None:
            """
            Update the active operational status of the queue.

            :param active: True to mark the queue as active, False otherwise.
            """
            self.active = active

        def get_status(self) -> QueueStatus:
            """
            Retrieve a copy of the current queue statistics and state.

            :return: A status object containing size, active state, and message counts.
            :rtype: QueueStatus
            """
            return QueueStatus(
                active=self.active,
                queue_size=len(self.events),
                msgs_processed=self.msgs_processed,
                msgs_dropped=self.msgs_dropped
            )

    def __init__(
        self,
        timeout: Duration,
        filters: tp.Sequence[SimpleFilter] | None = None,
    ) -> None:
        """
        Construct an InputAligner instance.

        :param timeout: Maximum duration to wait before an empty channel is considered stalled.
        :param filters: Optional initial sequence of upstream filters to connect.
        """
        self.timeout: Duration = timeout
        zero_time = _ros_zero_time()
        self.last_in_ts: Time = zero_time
        self.last_out_ts: Time = zero_time
        self.name: str = ''
        self.lock: threading.RLock = threading.RLock()
        self.event_queues: list[InputAligner._EventQueue] = []
        self.input_connections: list[tuple[SimpleFilter, int]] = []
        self.signals: list[SimpleFilter] = []
        self.dispatch_timer: tp.Any = None
        if filters is not None:
            self.connectInput(filters=filters)

    def connectInput(
        self,
        filters: tp.Sequence[SimpleFilter],
    ) -> None:
        """
        Connect ``filters`` as inputs, replacing any existing inputs.

        Note: previously-registered downstream callbacks are also dropped,
        since the per-input signals are recreated.

        :param filters: A sequence of upstream filters supplying messages.
        """
        with self.lock:
            self._disconnect_all_locked()
            self.event_queues = [InputAligner._EventQueue() for _ in filters]
            self.signals = [SimpleFilter() for _ in filters]
            # SimpleFilter.registerCallback appends extra args after the message,
            # so registering ``self.add`` with ``idx`` invokes ``add(msg, idx)``.
            self.input_connections = [
                (f, f.registerCallback(self.add, idx)) for idx, f in enumerate(filters)
            ]

    def disconnectAll(self) -> None:
        """
        Disconnect all upstream input filters safely.

        Thread-safe wrapper around the internal disconnection logic.
        """
        with self.lock:
            self._disconnect_all_locked()

    def _disconnect_all_locked(self) -> None:
        """
        Unregister all callback connections from upstream filters.

        Must be called with the internal reentrant lock acquired.
        """
        for input_filter, conn in self.input_connections:
            input_filter.unregisterCallback(conn)
        self.input_connections = []

    def registerCallback(
        self,
        index: int,
        callback: tp.Callable[tp.Concatenate[MsgT, ...], None],
        *args: tp.Any,
    ) -> int:
        """
        Register a downstream callback for a specific output channel index.

        :param index: The positional index of the output channel to listen to.
        :param callback: The callable handler to invoke when a message is dispatched.
        :param args: Extra arguments passed down directly to the callback registry.
        :return: A registration connection index token.
        :rtype: int
        :raises IndexError: If the specified index does not map to a configured channel.
        """
        return self.signals[index].registerCallback(callback, *args)

    def setName(self, name: str) -> None:
        """
        Set a diagnostic identifier string for this aligner instance.

        :param name: The name string.
        """
        self.name = name

    def getName(self) -> str:
        """
        Get the diagnostic identifier string of this aligner instance.

        :return: The configured name.
        :rtype: str
        """
        return self.name

    def add(self, msg: tp.Any, queue_index: int) -> None:
        """
        Process an incoming message from a connected upstream source.

        Drops the message if its header timestamp is older than the last dispatched output.
        Otherwise, queues the message and updates internal timestamp bounds.

        :param msg: The incoming message instance containing a valid header.stamp.
        :param queue_index: The index of the specific queue this message belongs to.
        """
        msg_timestamp = Time.from_msg(msg.header.stamp)
        with self.lock:
            queue = self.event_queues[queue_index]
            if msg_timestamp < self.last_out_ts:
                queue.msg_dropped()
                return
            if msg_timestamp > self.last_in_ts:
                self.last_in_ts = msg_timestamp
            queue.push(msg_timestamp, msg)

    def setInputPeriod(self, index: int, period: Duration) -> None:
        """
        Configure the message rate period for a specific queue index.

        :param index: The position index of the targeted input queue.
        :param period: The expected Duration between successive messages.
        :raises IndexError: If the index is out of bounds for the configured queues.
        """
        self.event_queues[index].set_period(period)

    def getQueueStatus(self, index: int) -> QueueStatus:
        """
        Retrieve state statistics for a specific queue index.

        :param index: The position index of the targeted input queue.
        :return: The operational status metrics of the requested queue.
        :rtype: QueueStatus
        :raises IndexError: If the index is out of bounds for the configured queues.
        """
        return self.event_queues[index].get_status()

    def setupDispatchTimer(self, node: Node, update_rate: Duration) -> None:
        """
        Create a periodic ROS 2 wall timer to drive the message alignment loop.

        :param node: The ROS 2 Node instance used to initialize the timer.
        :param update_rate: Duration specifying how frequently the timer should fire.
        """
        self.dispatch_timer = node.create_timer(
            update_rate.nanoseconds / 1e9, self.dispatchMessages)

    def dispatchMessages(self) -> None:
        """
        Process and flush all currently alignment-ready messages across all channels.

        Acquires the internal lock and processes messages until no more can be dispatched.
        """
        with self.lock:
            if not any(queue.events for queue in self.event_queues):
                return
            while self._dispatch_first_message():
                pass

    def _dispatch_first_message(self) -> bool:
        """
        Evaluate and dispatch the single oldest available message across all streams.

        If a queue's earliest time is stalled beyond the timeout limit relative to the
        latest incoming message, that channel's active state is dropped.

        :return: True if a message was successfully dispatched or a stalled channel state changed,
                 False if no progress could be made.
        :rtype: bool
        """
        timestamps = [queue.first_timestamp() for queue in self.event_queues]
        idx = min(range(len(timestamps)), key=lambda i: timestamps[i].nanoseconds)
        queue = self.event_queues[idx]
        if queue.events:
            stamp, msg = queue.pop_first()
            self.last_out_ts = stamp
            self.signals[idx].signalMessage(msg)
            return True
        if (self.last_in_ts - queue.first_timestamp()) >= self.timeout:
            queue.set_active(False)
            return True
        return False
