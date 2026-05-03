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


import time
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import SimpleFilter
from message_filters.input_aligner import InputAligner
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time


class Header:

    def __init__(self, stamp=None):
        self.stamp = stamp if stamp is not None else TimeMsg()


class Msg1:

    def __init__(self, stamp=None, data=None):
        self.header = Header(stamp)
        self.data = data


class Msg2:

    def __init__(self, stamp=None, data=None):
        self.header = Header(stamp)
        self.data = data


class TestInputAligner(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_input_aligner_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.timeout = Duration(seconds=1.0)
        self.update_rate = Duration(nanoseconds=10000000)
        self.callback_content = []

    def callback(self, msg):
        self.callback_content.append(msg.data)

    def create_msg(self, cls, milliseconds, data):
        return cls(stamp=Time(nanoseconds=int(milliseconds * 1e6)).to_msg(), data=data)

    def test_init(self):
        f0, f1, f2, f3 = SimpleFilter(), SimpleFilter(), SimpleFilter(), SimpleFilter()
        aligner1 = InputAligner(self.timeout, filters=[f0, f1, f2, f3])
        self.assertEqual(len(aligner1.event_queues), 4)
        aligner2 = InputAligner(self.timeout)
        aligner2.connectInput(filters=[f0, f2, f3])
        self.assertEqual(len(aligner2.event_queues), 3)

    def test_set_and_get_name(self):
        aligner = InputAligner(self.timeout)
        self.assertEqual(aligner.getName(), '')
        aligner.setName('camera_aligner')
        self.assertEqual(aligner.getName(), 'camera_aligner')

    def test_dispatch_inputs_in_order(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(
            filters=[SimpleFilter(), SimpleFilter(), SimpleFilter(), SimpleFilter()])
        for i in range(4):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(4e6)))
        aligner.add(self.create_msg(Msg1, 3, 3), 2)
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg1, 7, 7), 2)
        aligner.add(self.create_msg(Msg1, 5, 5), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 3)
        aligner.add(self.create_msg(Msg1, 9, 9), 0)
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.add(self.create_msg(Msg2, 8, 8), 1)
        aligner.add(self.create_msg(Msg2, 6, 6), 3)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, list(range(1, 10)))

    def test_dispatch_inputs_with_duplicate_timestamps(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(
            filters=[SimpleFilter(), SimpleFilter(), SimpleFilter(), SimpleFilter()])
        for i in range(4):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(4e6)))
        aligner.add(self.create_msg(Msg1, 3, 3), 2)
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg1, 7, 7), 2)
        aligner.add(self.create_msg(Msg1, 5, 5), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 3)
        aligner.add(self.create_msg(Msg1, 9, 9), 0)
        # Two messages with identical timestamp on the same queue should
        # dispatch in insertion order via the seq_id tie-breaker.
        aligner.add(self.create_msg(Msg1, 9, 9), 0)
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.add(self.create_msg(Msg2, 8, 8), 1)
        aligner.add(self.create_msg(Msg2, 6, 6), 3)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4, 5, 6, 7, 8, 9, 9])

    def test_reconnect_input_disconnects_old_upstream(self):
        f0, f1, f2 = SimpleFilter(), SimpleFilter(), SimpleFilter()
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[f0, f1, f2])
        aligner.connectInput(filters=[f0, f1])
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        # f2 was dropped on reconnect; messages from it must be ignored.
        f2.signalMessage(self.create_msg(Msg1, 1, 1))
        f0.signalMessage(self.create_msg(Msg1, 2, 2))
        f1.signalMessage(self.create_msg(Msg2, 3, 3))
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [2, 3])

    def test_reconnect_input_drops_old_downstream_callbacks(self):
        # Callbacks registered before reconnect should be discarded along with
        # the per-input signals they were attached to.
        stale = []
        fresh = []
        f0, f1 = SimpleFilter(), SimpleFilter()
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[f0, f1])
        aligner.registerCallback(0, lambda m: stale.append(m.data))
        aligner.registerCallback(1, lambda m: stale.append(m.data))
        aligner.connectInput(filters=[f0, f1])
        aligner.registerCallback(0, lambda m: fresh.append(m.data))
        aligner.registerCallback(1, lambda m: fresh.append(m.data))
        for i in range(2):
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.dispatchMessages()
        self.assertEqual(stale, [])
        self.assertEqual(fresh, [1, 2])

    def test_disconnect_all_stops_upstream_delivery(self):
        f0, f1 = SimpleFilter(), SimpleFilter()
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[f0, f1])
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.disconnectAll()
        f0.signalMessage(self.create_msg(Msg1, 1, 1))
        f1.signalMessage(self.create_msg(Msg2, 2, 2))
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [])

    def test_simple_filter_unregister_callback(self):
        received = []
        f = SimpleFilter()
        conn_a = f.registerCallback(lambda m: received.append(('a', m)))
        f.registerCallback(lambda m: received.append(('b', m)))
        f.signalMessage(1)
        self.assertEqual(received, [('a', 1), ('b', 1)])
        f.unregisterCallback(conn_a)
        f.signalMessage(2)
        self.assertEqual(received, [('a', 1), ('b', 1), ('b', 2)])
        # Unregistering an unknown id must be a no-op.
        f.unregisterCallback(conn_a)
        f.unregisterCallback(9999)
        f.signalMessage(3)
        self.assertEqual(received, [('a', 1), ('b', 1), ('b', 2), ('b', 3)])

    def test_ignores_inactive_inputs(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter(), SimpleFilter()])
        for i in range(3):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg1, 2, 2), 2)
        aligner.add(self.create_msg(Msg2, 1, 1), 1)
        aligner.add(self.create_msg(Msg1, 4, 4), 2)
        aligner.add(self.create_msg(Msg2, 3, 3), 1)
        aligner.add(self.create_msg(Msg2, 5, 5), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4, 5])

    def test_input_timeout(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter()])
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        for i in range(1, 17, 2):
            aligner.add(self.create_msg(Msg1, i, i), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4, 5])
        aligner.add(self.create_msg(Msg1, 17, 17), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4, 5, 7, 9, 11, 13, 15, 17])

    def test_drops_msgs(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter()])
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.add(self.create_msg(Msg1, 3, 3), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [3, 4])
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg1, 5, 5), 0)
        aligner.add(self.create_msg(Msg1, 7, 7), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg2, 6, 6), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [3, 4, 5, 6, 7])

    def test_dispatch_by_timer(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter()])
        aligner.setupDispatchTimer(self.node, self.update_rate)
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        # Spin until the timer fires and both messages flow through, with a
        # generous deadline so this is not flaky on slow CI runners.
        deadline = time.monotonic() + 2.0
        while len(self.callback_content) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(self.callback_content, [1, 2])

    def test_no_period_information(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter(), SimpleFilter()])
        for i in range(3):
            aligner.registerCallback(i, self.callback)
        aligner.add(self.create_msg(Msg1, 6, 6), 0)
        aligner.add(self.create_msg(Msg1, 2, 2), 2)
        aligner.add(self.create_msg(Msg1, 4, 4), 2)
        aligner.add(self.create_msg(Msg2, 1, 1), 1)
        aligner.add(self.create_msg(Msg2, 3, 3), 1)
        aligner.add(self.create_msg(Msg2, 5, 5), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4])
        aligner.add(self.create_msg(Msg1, 16, 16), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.callback_content, [1, 2, 3, 4, 5, 6, 16])

    def test_get_queue_status(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(filters=[SimpleFilter(), SimpleFilter()])
        for i in range(2):
            aligner.registerCallback(i, self.callback)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg1, 3, 3), 0)
        aligner.add(self.create_msg(Msg1, 5, 5), 0)
        status_0 = aligner.getQueueStatus(0)
        self.assertFalse(status_0.active)
        self.assertEqual(status_0.queue_size, 2)
        self.assertEqual(status_0.msgs_processed, 0)
        self.assertEqual(status_0.msgs_dropped, 0)
        status_1 = aligner.getQueueStatus(1)
        self.assertFalse(status_1.active)
        self.assertEqual(status_1.queue_size, 1)
        self.assertEqual(status_1.msgs_processed, 0)
        self.assertEqual(status_1.msgs_dropped, 0)
        aligner.dispatchMessages()
        status_0 = aligner.getQueueStatus(0)
        self.assertTrue(status_0.active)
        self.assertEqual(status_0.queue_size, 1)
        self.assertEqual(status_0.msgs_processed, 1)
        self.assertEqual(status_0.msgs_dropped, 0)
        status_1 = aligner.getQueueStatus(1)
        self.assertTrue(status_1.active)
        self.assertEqual(status_1.queue_size, 0)
        self.assertEqual(status_1.msgs_processed, 1)
        self.assertEqual(status_1.msgs_dropped, 0)
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg1, 17, 17), 0)
        aligner.dispatchMessages()
        status_0 = aligner.getQueueStatus(0)
        self.assertTrue(status_0.active)
        self.assertEqual(status_0.queue_size, 0)
        self.assertEqual(status_0.msgs_processed, 3)
        self.assertEqual(status_0.msgs_dropped, 1)
        status_1 = aligner.getQueueStatus(1)
        self.assertFalse(status_1.active)
        self.assertEqual(status_1.queue_size, 0)
        self.assertEqual(status_1.msgs_processed, 1)
        self.assertEqual(status_1.msgs_dropped, 0)


if __name__ == '__main__':
    unittest.main()
