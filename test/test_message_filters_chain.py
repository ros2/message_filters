# Copyright 2025, Open Source Robotics Foundation, Inc. All rights reserved.
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


import unittest

from message_filters import Cache, Chain, Subscriber
import rclpy
from rclpy.time import Time
from std_msgs.msg import String


class AnonymMsg:
    class AnonymHeader:

        def __init__(self):
            self.stamp = Time()

    def __init__(self):
        self.header = AnonymMsg.AnonymHeader()


class TestChain(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('TestChainNode', namespace='/my_ns')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_add_filter(self):
        chain_filter = Chain()

        first_cache_filter = Cache(
            f=Subscriber(self.node, String, '/empty'),
            cache_size=3,
        )
        second_cache_filter = Cache(
            f=Subscriber(self.node, String, '/empty'),
            cache_size=3,
        )

        chain_filter.addFilter(first_cache_filter)

        assert chain_filter.getFilter(0) == first_cache_filter

        first_msg = AnonymMsg()
        first_msg.header.stamp = Time(seconds=1)
        chain_filter.add(first_msg)

        first_cached_messages = first_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        second_cached_messages = second_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        assert first_cached_messages == [first_msg]
        assert second_cached_messages == []

        chain_filter.addFilter(second_cache_filter)

        second_msg = AnonymMsg()
        second_msg.header.stamp = Time(seconds=2)
        chain_filter.add(second_msg)

        first_cached_messages = first_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        second_cached_messages = second_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        assert first_cached_messages == [first_msg, second_msg]
        assert second_cached_messages == [second_msg]

    def test_get_filter(self):
        chain_filter = Chain()

        with self.assertRaises(KeyError):
            chain_filter.getFilter(0)

        first_cache_filter = Cache(Subscriber(self.node, String, '/empty'))
        second_cache_filter = Cache(Subscriber(self.node, String, '/empty'))

        chain_filter.addFilter(first_cache_filter)
        chain_filter.addFilter(second_cache_filter)

        first_filter_in_chain = chain_filter.getFilter(0)
        second_filter_in_chain = chain_filter.getFilter(1)

        assert first_filter_in_chain is first_cache_filter
        assert second_filter_in_chain is second_cache_filter

    def test_chain_callback(self):
        class Counter:

            def __init__(self):
                self._counter: int = 0

            def counter_callback(self, _):
                self._counter += 1

            @property
            def value(self):
                return self._counter

        counter = Counter()

        assert counter.value == 0

        chain_filter = Chain()
        cache_filter = Cache(Subscriber(self.node, String, '/empty'))
        chain_filter.addFilter(cache_filter)
        chain_filter.registerCallback(counter.counter_callback)

        msg = AnonymMsg()
        msg.header.stamp = Time(seconds=1)
        chain_filter.add(msg)

        assert counter.value == 1

    def test_connect_input(self):
        chain_filter = Chain()
        first_cache_filter = Cache(Subscriber(self.node, String, '/empty'))
        second_cache_filter = Cache(Subscriber(self.node, String, '/empty'))

        chain_filter.connectInput(first_cache_filter)
        chain_filter.addFilter(second_cache_filter)

        msg = AnonymMsg()
        msg.header.stamp = Time(seconds=1)
        first_cache_filter.add(msg)

        first_cached_messages = first_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        second_cached_messages = second_cache_filter.getInterval(
            Time(seconds=0),
            Time(seconds=10)
        )

        assert first_cached_messages == second_cached_messages


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestChain('test_add_filter'))
    suite.addTest(TestChain('test_get_filter'))
    suite.addTest(TestChain('test_chain_callback'))
    suite.addTest(TestChain('test_connect_input'))
    unittest.TextTestRunner(verbosity=2).run(suite)
