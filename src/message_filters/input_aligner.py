from bisect import insort_right
from dataclasses import dataclass
from queue import Queue
import threading

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


def _ros_zero_time():
    return Time.from_msg(TimeMsg())


def _ros_max_time():
    zero = _ros_zero_time()
    return Time(nanoseconds=9223372036854775807, clock_type=zero.clock_type)


class _EventQueue:
    def __init__(self):
        self.events = Queue()
        self.next_ts = _ros_max_time()
        self.period = Duration(seconds=0)
        self.active = False
        self.msgs_processed = 0
        self.msgs_dropped = 0

    def first_timestamp(self):
        if not self.events.empty():
            first_ts = self.events.queue[0][0]
            self.next_ts = first_ts + self.period
            self.active = True
            return first_ts
        if self.active:
            return self.next_ts
        return _ros_max_time()

    def pop_first(self):
        self.events.get_nowait()
        self.msgs_processed += 1

    def msg_dropped(self):
        self.msgs_dropped += 1

    def set_period(self, period):
        self.period = period

    def set_active(self, active):
        self.active = active

    def get_status(self):
        return QueueStatus(self.active, self.events.qsize(), self.msgs_processed, self.msgs_dropped)


class InputAligner(SimpleFilter):
    def __init__(self, timeout, *filters):
        SimpleFilter.__init__(self)
        self.timeout = timeout
        zero_time = _ros_zero_time()
        self.last_in_ts = zero_time
        self.last_out_ts = zero_time
        self.name = ''
        self.lock = threading.Lock()
        self.event_queues = []
        self.input_connections = []
        self.signals = []
        self.dispatch_timer = None
        if filters:
            self.connectInput(*filters)

    def connectInput(self, *filters):
        self.disconnectAll()
        self.event_queues = [_EventQueue() for _ in filters]
        self.signals = [_Signal() for _ in filters]
        self.input_connections = [f.registerCallback(self.add, idx) for idx, f in enumerate(filters)]

    def disconnectAll(self):
        self.input_connections = []

    def registerCallback(self, index, cb, *args):
        return self.signals[index].registerCallback(cb, *args)

    def setName(self, name):
        self.name = name

    def getName(self):
        return self.name

    def add(self, msg, queue_index):
        msg_timestamp = Time.from_msg(msg.header.stamp)
        with self.lock:
            queue = self.event_queues[queue_index]
            if msg_timestamp < self.last_out_ts:
                queue.msg_dropped()
                return
            if msg_timestamp > self.last_in_ts:
                self.last_in_ts = msg_timestamp
            insort_right(queue.events.queue, (msg_timestamp, msg), key=lambda x: x[0].nanoseconds)

    def setInputPeriod(self, index, period):
        self.event_queues[index].set_period(period)

    def getQueueStatus(self, index):
        return self.event_queues[index].get_status()

    def setupDispatchTimer(self, node, update_rate):
        self.dispatch_timer = node.create_timer(update_rate.nanoseconds / 1e9, self.dispatchMessages)

    def dispatchMessages(self):
        with self.lock:
            if not any(not queue.events.empty() for queue in self.event_queues):
                return
            input_available = True
            while input_available:
                input_available = self._dispatch_first_message()

    def _dispatch_first_message(self):
        timestamps = [queue.first_timestamp() for queue in self.event_queues]
        idx = min(range(len(timestamps)), key=lambda i: timestamps[i].nanoseconds)
        queue = self.event_queues[idx]
        if not queue.events.empty():
            stamp, msg = queue.events.queue[0]
            self.last_out_ts = stamp
            self.signals[idx].signalMessage(msg)
            queue.pop_first()
            return True
        if (self.last_in_ts - queue.first_timestamp()) >= self.timeout:
            queue.set_active(False)
            return True
        return False
