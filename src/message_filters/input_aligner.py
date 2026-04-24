# Copyright 2009, Willow Garage, Inc. All rights reserved.
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
from queue import PriorityQueue as Queue
import threading
import typing as tp

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import SimpleFilter
from rclpy.duration import Duration
from rclpy.time import Time


@dataclass
class QueueStatus:
    active: bool
    queue_size: int
    msgs_processed: int
    msgs_dropped: int


class _Signal:
    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))


def _ros_zero_time() -> Time:
    return Time.from_msg(TimeMsg())


def _ros_max_time() -> Time:
    return Time(
        nanoseconds=9223372036854775807,
        clock_type=_ros_zero_time().clock_type,
    )


class _EventQueue:
    def __init__(self) -> None:
        self.events: Queue = Queue()
        self.next_ts: Time = _ros_max_time()
        self.period: Duration = Duration(seconds=0)
        self.active: bool = False
        self.msgs_processed: int = 0
        self.msgs_dropped: int = 0
        self.seq_id: int = 0

    def first_timestamp(self) -> Time:
        if not self.events.empty():
            first_ts = self.events.queue[0][0]
            self.next_ts = first_ts + self.period
            self.active = True
            return first_ts
        if self.active:
            return self.next_ts
        return _ros_max_time()

    def pop_first(self) -> None:
        self.events.get_nowait()
        self.msgs_processed += 1

    def msg_dropped(self) -> None:
        self.msgs_dropped += 1

    def set_period(self, period: Duration) -> None:
        self.period = period

    def set_active(self, active: bool) -> None:
        self.active = active

    def get_status(self) -> QueueStatus:
        return QueueStatus(self.active, self.events.qsize(), self.msgs_processed, self.msgs_dropped)


class InputAligner(SimpleFilter):
    def __init__(
        self,
        timeout: Duration,
        *filters: SimpleFilter
    ) -> None:
        SimpleFilter.__init__(self)
        self.timeout: Duration = timeout
        zero_time = _ros_zero_time()
        self.last_in_ts: Time = zero_time
        self.last_out_ts: Time = zero_time
        self.name: str = ''
        self.lock: threading.Lock = threading.Lock()
        self.event_queues: list[_EventQueue] = []
        self.input_connections: list[tuple[SimpleFilter, int]] = []
        self.signals: list[_Signal] = []
        self.dispatch_timer: tp.Any = None
        if filters:
            self.connectInput(*filters)

    def connectInput(
        self,
        *filters: SimpleFilter
    ) -> None:
        with self.lock:
            self.disconnectAll()
            self.event_queues = [_EventQueue() for _ in filters]
            self.signals = [_Signal() for _ in filters]
            self.input_connections = [(f, f.registerCallback(self.add, idx)) for idx, f in enumerate(filters)]

    def disconnectAll(self) -> None:
        for input_filter, conn in self.input_connections:
            input_filter.unregisterCallback(conn)
        self.input_connections = []

    def registerCallback(
        self,
        index: int,
        callback: tp.Callable[..., tp.Any],
        *args: tp.Any,
    ) -> int:
        return self.signals[index].registerCallback(callback, *args)

    def setName(self, name: str) -> None:
        self.name = name

    def getName(self) -> str:
        return self.name

    def add(self, msg: tp.Any, queue_index: int) -> None:
        msg_timestamp = Time.from_msg(msg.header.stamp)
        with self.lock:
            queue = self.event_queues[queue_index]
            if msg_timestamp < self.last_out_ts:
                queue.msg_dropped()
                return
            if msg_timestamp > self.last_in_ts:
                self.last_in_ts = msg_timestamp
            # Use seq_id as a tie-breaker so duplicate timestamps stay orderable.
            queue.events.put_nowait((msg_timestamp, queue.seq_id, msg))
            queue.seq_id += 1

    def setInputPeriod(self, index: int, period: Duration) -> None:
        self.event_queues[index].set_period(period)

    def getQueueStatus(self, index: int) -> QueueStatus:
        return self.event_queues[index].get_status()

    def setupDispatchTimer(self, node: tp.Any, update_rate: Duration) -> None:
        self.dispatch_timer = node.create_timer(update_rate.nanoseconds / 1e9, self.dispatchMessages)

    def dispatchMessages(self) -> None:
        with self.lock:
            if not any(not queue.events.empty() for queue in self.event_queues):
                return
            input_available = True
            while input_available:
                input_available = self._dispatch_first_message()

    def _dispatch_first_message(self) -> bool:
        timestamps = [queue.first_timestamp() for queue in self.event_queues]
        idx = min(range(len(timestamps)), key=lambda i: timestamps[i].nanoseconds)
        queue = self.event_queues[idx]
        if not queue.events.empty():
            stamp, _, msg = queue.events.queue[0]
            self.last_out_ts = stamp
            self.signals[idx].signalMessage(msg)
            queue.pop_first()
            return True
        if (self.last_in_ts - queue.first_timestamp()) >= self.timeout:
            queue.set_active(False)
            return True
        return False
