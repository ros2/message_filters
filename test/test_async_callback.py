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

import asyncio
from concurrent.futures import Future
import functools
import threading
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import (
    ApproximateTimeSynchronizer,
    Cache,
    Chain,
    SimpleFilter,
    Subscriber,
    SynchronizingCallback,
    TimeSynchronizer,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Header:
    """Mock header with a stamp."""

    def __init__(self, sec=0, nanosec=0):
        self.stamp = TimeMsg(sec=sec, nanosec=nanosec)


class MockMsg:
    """Mock ROS message with header and data payload."""

    def __init__(self, sec=0, nanosec=0, data=0):
        self.header = Header(sec=sec, nanosec=nanosec)
        self.data = data


class TestAsyncCallbacks(unittest.TestCase):
    """Test suite verifying SynchronizingCallback with message_filters."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_async_node')
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

    def tearDown(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join(timeout=2.0)
        self.loop.close()
        self.node.destroy_node()

    def _wait_for_completed(
        self,
        sync_cb: SynchronizingCallback,
        expected_count: int = 1,
        timeout: float = 2.0,
    ) -> None:
        """Wait until all active futures finish and completed_futures reaches expected_count."""
        deadline = threading.Event()
        elapsed = 0.0
        step = 0.005
        while elapsed < timeout:
            if (
                len(sync_cb.completed_futures) >= expected_count
                and len(sync_cb.active_futures) == 0
            ):
                return
            deadline.wait(step)
            elapsed += step
        self.fail(
            f'Timed out waiting for {expected_count} completed future(s). '
            f'Active: {len(sync_cb.active_futures)}, '
            f'Completed: {len(sync_cb.completed_futures)}'
        )

    def test_simple_filter_synchronizing_callback_and_result(self):
        called = []

        async def async_cb(msg, multiplier, offset=0):
            called.append((msg.data, multiplier, offset))
            return msg.data * multiplier + offset

        sync_cb = SynchronizingCallback(async_cb, self.loop, 3, offset=5)
        f = SimpleFilter()
        f.registerCallback(sync_cb)
        f.signalMessage(MockMsg(data=10))

        self._wait_for_completed(sync_cb, expected_count=1)
        self.assertEqual(called, [(10, 3, 5)])
        self.assertEqual(len(sync_cb.active_futures), 0)
        self.assertEqual(len(sync_cb.completed_futures), 1)

        future = next(iter(sync_cb.completed_futures))
        self.assertEqual(sync_cb.get_result(future, timeout=1.0), 35)

        sync_cb.pop_future(future)
        self.assertEqual(len(sync_cb.completed_futures), 0)
        self.assertIsNone(sync_cb.get_result(future))

    def test_subscriber_bound_async_callback(self):
        called = []

        class AsyncNode(Node):

            def __init__(self):
                super().__init__('async_node')

            async def on_message(self, msg):
                called.append(msg.data)
                return msg.data

        async_node = AsyncNode()
        sync_cb = SynchronizingCallback(async_node.on_message, self.loop)

        sub = Subscriber(async_node, PointCloud2, 'test_topic_bound')
        sub.registerCallback(sync_cb)

        sub.signalMessage(MockMsg(data=42))
        self._wait_for_completed(sync_cb, expected_count=1)

        self.assertEqual(called, [42])
        async_node.destroy_node()

    def test_subscriber_standalone_async_callback(self):
        called = []

        async def standalone_cb(msg):
            called.append(msg.data)

        sync_cb = SynchronizingCallback(standalone_cb, self.loop)
        sub = Subscriber(self.node, PointCloud2, 'test_topic_standalone')
        sub.registerCallback(sync_cb)

        sub.signalMessage(MockMsg(data=84))
        self._wait_for_completed(sync_cb, expected_count=1)

        self.assertEqual(called, [84])

    def test_subscriber_partial_async_callback(self):
        called = []

        async def partial_cb(msg, tag):
            called.append((msg.data, tag))

        sync_cb = SynchronizingCallback(
            functools.partial(partial_cb, tag='tagged'),
            self.loop,
        )
        sub = Subscriber(self.node, PointCloud2, 'test_topic_partial')
        sub.registerCallback(sync_cb)

        sub.signalMessage(MockMsg(data=99))
        self._wait_for_completed(sync_cb, expected_count=1)

        self.assertEqual(called, [(99, 'tagged')])

    def test_time_synchronizer_async_callback(self):
        called = []

        async def sync_fn(msg1, msg2):
            called.append((msg1.data, msg2.data))
            return msg1.data + msg2.data

        sync_cb = SynchronizingCallback(sync_fn, self.loop)
        sub1 = Subscriber(self.node, PointCloud2, 'topic_sync1')
        sub2 = Subscriber(self.node, PointCloud2, 'topic_sync2')
        ts = TimeSynchronizer([sub1, sub2], 10)
        ts.registerCallback(sync_cb)

        m1 = MockMsg(sec=5, data=1)
        m2 = MockMsg(sec=5, data=2)

        sub1.signalMessage(m1)
        sub2.signalMessage(m2)

        self._wait_for_completed(sync_cb, expected_count=1)
        self.assertEqual(called, [(1, 2)])
        future = next(iter(sync_cb.completed_futures))
        self.assertEqual(sync_cb.get_result(future), 3)

    def test_approximate_time_synchronizer_async_callback(self):
        called = []

        async def approx_fn(msg1, msg2):
            called.append((msg1.data, msg2.data))

        sync_cb = SynchronizingCallback(approx_fn, self.loop)
        sub1 = Subscriber(self.node, PointCloud2, 'topic_approx1')
        sub2 = Subscriber(self.node, PointCloud2, 'topic_approx2')
        ats = ApproximateTimeSynchronizer([sub1, sub2], 10, slop=0.2)
        ats.registerCallback(sync_cb)

        m1 = MockMsg(sec=10, nanosec=0, data=10)
        m2 = MockMsg(sec=10, nanosec=50000000, data=20)

        sub1.signalMessage(m1)
        sub2.signalMessage(m2)

        self._wait_for_completed(sync_cb, expected_count=1)
        self.assertEqual(called, [(10, 20)])

    def test_cache_and_chain_async_callback(self):
        called = []

        async def pipeline_cb(msg):
            called.append(msg.data)

        sync_cb = SynchronizingCallback(pipeline_cb, self.loop)
        sub = Subscriber(self.node, PointCloud2, 'topic_cache_chain')
        cache = Cache(sub, 5)
        chain = Chain(cache)
        chain.registerCallback(sync_cb)

        sub.signalMessage(MockMsg(sec=1, data=77))
        self._wait_for_completed(sync_cb, expected_count=1)

        self.assertEqual(called, [77])

    def test_active_future_raises_invalid_state_error(self):
        gate = threading.Event()

        async def slow_cb(msg):
            await asyncio.to_thread(gate.wait, 2.0)
            return msg.data

        sync_cb = SynchronizingCallback(slow_cb, self.loop)
        future = sync_cb(MockMsg(data=55))

        self.assertIn(future, sync_cb.active_futures)
        self.assertNotIn(future, sync_cb.completed_futures)

        with self.assertRaises(asyncio.InvalidStateError):
            sync_cb.get_result(future)

        gate.set()
        self._wait_for_completed(sync_cb, expected_count=1)
        self.assertEqual(sync_cb.get_result(future), 55)

    def test_properties_return_shallow_copies_and_unknown_future(self):
        async def noop_cb(msg):
            return msg.data

        sync_cb = SynchronizingCallback(noop_cb, self.loop)
        future = sync_cb(MockMsg(data=11))
        self._wait_for_completed(sync_cb, expected_count=1)

        active_copy = sync_cb.active_futures
        completed_copy = sync_cb.completed_futures
        active_copy.add(Future())
        completed_copy.clear()

        self.assertEqual(len(sync_cb.active_futures), 0)
        self.assertEqual(len(sync_cb.completed_futures), 1)
        self.assertIn(future, sync_cb.completed_futures)
        self.assertIsNone(sync_cb.get_result(Future()))

    def test_rapid_concurrent_completions_have_no_race(self):
        async def immediate_cb(msg):
            return msg.data

        sync_cb = SynchronizingCallback(immediate_cb, self.loop)
        f = SimpleFilter()
        f.registerCallback(sync_cb)

        count = 50
        for i in range(count):
            f.signalMessage(MockMsg(data=i))

        self._wait_for_completed(sync_cb, expected_count=count)
        self.assertEqual(len(sync_cb.active_futures), 0)
        self.assertEqual(len(sync_cb.completed_futures), count)

        results = {sync_cb.get_result(fut) for fut in sync_cb.completed_futures}
        self.assertEqual(results, set(range(count)))

    def test_unregister_callback_prevents_scheduling(self):
        called = []

        async def async_cb(msg):
            called.append(msg.data)

        sync_cb = SynchronizingCallback(async_cb, self.loop)
        f = SimpleFilter()
        conn = f.registerCallback(sync_cb)
        f.unregisterCallback(conn)

        f.signalMessage(MockMsg(data=999))
        self.assertEqual(len(sync_cb.active_futures), 0)
        self.assertEqual(len(sync_cb.completed_futures), 0)
        self.assertEqual(called, [])


if __name__ == '__main__':
    unittest.main()
