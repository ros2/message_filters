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
import functools
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import (
    ApproximateTimeSynchronizer,
    Cache,
    Chain,
    SimpleFilter,
    Subscriber,
    TimeSynchronizer,
)
import rclpy
from rclpy.executors import SingleThreadedExecutor
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
    """Test suite verifying native async callback support in message_filters."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_async_node')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_sync_callback_backward_compatibility(self):
        called = []

        def sync_cb(msg, extra):
            called.append((msg.data, extra))
            return 'return_value'

        f = SimpleFilter()
        f.registerCallback(sync_cb, 'extra_arg')
        f.signalMessage(MockMsg(data=100))

        # Synchronous callbacks execute immediately without spinning an executor
        self.assertEqual(called, [(100, 'extra_arg')])

    def test_subscriber_bound_async_callback(self):
        called = []

        class AsyncNode(Node):

            def __init__(self):
                super().__init__('async_node')

            async def on_message(self, msg):
                called.append(msg.data)

        async_node = AsyncNode()
        self.executor.add_node(async_node)

        sub = Subscriber(async_node, PointCloud2, 'test_topic_bound')
        sub.registerCallback(async_node.on_message)

        sub.signalMessage(MockMsg(data=42))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [42])
        async_node.destroy_node()

    def test_subscriber_standalone_async_callback(self):
        called = []

        async def standalone_cb(msg):
            called.append(msg.data)

        sub = Subscriber(self.node, PointCloud2, 'test_topic_standalone')
        sub.registerCallback(standalone_cb)

        sub.signalMessage(MockMsg(data=84))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [84])

    def test_subscriber_partial_async_callback(self):
        called = []

        async def partial_cb(msg, tag):
            called.append((msg.data, tag))

        sub = Subscriber(self.node, PointCloud2, 'test_topic_partial')
        sub.registerCallback(functools.partial(partial_cb, tag='tagged'))

        sub.signalMessage(MockMsg(data=99))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [(99, 'tagged')])

    def test_time_synchronizer_async_callback(self):
        called = []

        async def sync_cb(msg1, msg2):
            called.append((msg1.data, msg2.data))

        sub1 = Subscriber(self.node, PointCloud2, 'topic_sync1')
        sub2 = Subscriber(self.node, PointCloud2, 'topic_sync2')
        ts = TimeSynchronizer([sub1, sub2], 10)
        ts.registerCallback(sync_cb)

        m1 = MockMsg(sec=5, data=1)
        m2 = MockMsg(sec=5, data=2)

        sub1.signalMessage(m1)
        sub2.signalMessage(m2)

        self.executor.spin_once(timeout_sec=0.1)
        self.assertEqual(called, [(1, 2)])

    def test_approximate_time_synchronizer_async_callback(self):
        called = []

        async def approx_cb(msg1, msg2):
            called.append((msg1.data, msg2.data))

        sub1 = Subscriber(self.node, PointCloud2, 'topic_approx1')
        sub2 = Subscriber(self.node, PointCloud2, 'topic_approx2')
        ats = ApproximateTimeSynchronizer([sub1, sub2], 10, slop=0.2)
        ats.registerCallback(approx_cb)

        m1 = MockMsg(sec=10, nanosec=0, data=10)
        m2 = MockMsg(sec=10, nanosec=50000000, data=20)

        sub1.signalMessage(m1)
        sub2.signalMessage(m2)

        self.executor.spin_once(timeout_sec=0.1)
        self.assertEqual(called, [(10, 20)])

    def test_cache_async_callback(self):
        called = []

        async def cache_cb(msg):
            called.append(msg.data)

        sub = Subscriber(self.node, PointCloud2, 'topic_cache')
        cache = Cache(sub, 5)
        cache.registerCallback(cache_cb)

        sub.signalMessage(MockMsg(sec=1, data=77))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [77])

    def test_chain_async_callback(self):
        called = []

        async def chain_cb(msg):
            called.append(msg.data)

        sub = Subscriber(self.node, PointCloud2, 'topic_chain')
        chain = Chain(sub)
        chain.registerCallback(chain_cb)

        sub.signalMessage(MockMsg(data=33))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [33])

    def test_custom_filter_subclass_async_callback(self):
        called = []

        class CustomFilter(SimpleFilter):

            def __init__(self, f):
                super().__init__()
                self.conn = f.registerCallback(self.add)

            def add(self, msg):
                self.signalMessage(msg)

        async def custom_cb(msg):
            called.append(msg.data)

        sub = Subscriber(self.node, PointCloud2, 'topic_custom')
        custom = CustomFilter(sub)
        custom.registerCallback(custom_cb)

        sub.signalMessage(MockMsg(sec=1, data=123))

        self.executor.spin_once(timeout_sec=0.1)
        self.assertEqual(called, [123])

    def test_explicit_executor_registration(self):
        custom_executor = SingleThreadedExecutor()
        called = []

        async def async_cb(msg):
            called.append(msg.data)

        f = SimpleFilter()
        f.registerCallback(async_cb, executor=custom_executor)
        f.signalMessage(MockMsg(data=505))

        custom_executor.spin_once(timeout_sec=0.1)
        self.assertEqual(called, [505])

    def test_asyncio_event_loop_support(self):
        called = []

        async def async_cb(msg):
            called.append(msg.data)

        async def runner():
            f = SimpleFilter()
            f.registerCallback(async_cb)
            f.signalMessage(MockMsg(data=707))
            await asyncio.sleep(0.01)

        asyncio.run(runner())
        self.assertEqual(called, [707])

    def test_missing_executor_raises_runtime_error(self):
        f = SimpleFilter()

        async def unhandled_async_cb(msg):
            pass

        f.registerCallback(unhandled_async_cb)
        with self.assertRaises(RuntimeError) as ctx:
            f.signalMessage(MockMsg(data=0))

        self.assertIn('No rclpy executor or running asyncio event loop found', str(ctx.exception))

    def test_unregister_callback_prevents_scheduling(self):
        called = []

        async def async_cb(msg):
            called.append(msg.data)

        f = SimpleFilter()
        conn = f.registerCallback(async_cb, executor=self.executor)
        f.unregisterCallback(conn)

        f.signalMessage(MockMsg(data=999))
        self.executor.spin_once(timeout_sec=0.1)

        self.assertEqual(called, [])


if __name__ == '__main__':
    unittest.main()
