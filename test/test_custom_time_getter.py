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

"""Tests for the custom time_getter support of the Python filters."""

import unittest

import message_filters
from message_filters import ApproximateTimeSynchronizer, Cache, TimeSynchronizer

from rclpy.time import Time

from std_msgs.msg import Header


class MockHeaderlessMessage:
    """A message carrying its timestamp in plain fields instead of a header."""

    def __init__(self, sec, nanosec, data):
        self.sec = sec
        self.nanosec = nanosec
        self.data = data


class MockHeaderMessage:

    def __init__(self, sec):
        self.header = Header()
        self.header.stamp.sec = sec


class MockFilter(message_filters.SimpleFilter):
    pass


def field_time_getter(msg):
    return Time(seconds=msg.sec, nanoseconds=msg.nanosec)


class TestCustomTimeGetter(unittest.TestCase):

    def test_cache_with_custom_getter(self):
        f = MockFilter()
        cache = Cache(f, cache_size=5, time_getter=field_time_getter)

        for sec in (1, 2, 3):
            f.signalMessage(MockHeaderlessMessage(sec, 0, sec))

        self.assertEqual(
            [t.nanoseconds for t in cache.cache_times],
            [1 * 10**9, 2 * 10**9, 3 * 10**9])

    def test_approximate_synchronizer_with_custom_getter(self):
        got = []
        f0, f1 = MockFilter(), MockFilter()
        sync = ApproximateTimeSynchronizer(
            [f0, f1], 10, 0.1, time_getter=field_time_getter)
        sync.registerCallback(lambda a, b: got.append((a, b)))

        f0.signalMessage(MockHeaderlessMessage(5, 0, 1))
        f1.signalMessage(MockHeaderlessMessage(5, 10, 2))

        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][0].data, 1)
        self.assertEqual(got[0][1].data, 2)

    def test_time_synchronizer_with_custom_getter(self):
        got = []
        f0, f1 = MockFilter(), MockFilter()
        sync = TimeSynchronizer([f0, f1], 10, time_getter=field_time_getter)
        sync.registerCallback(lambda a, b: got.append((a, b)))

        f0.signalMessage(MockHeaderlessMessage(7, 0, 1))
        f1.signalMessage(MockHeaderlessMessage(7, 0, 2))

        self.assertEqual(len(got), 1)

    def test_default_getter_still_drops_headerless(self):
        f = MockFilter()
        cache = Cache(f, cache_size=5)

        f.signalMessage(MockHeaderlessMessage(1, 0, 1))

        self.assertEqual(cache.cache_times, [])
        self.assertEqual(cache.cache_msgs, [])

    def test_default_getter_headerless_allowed_uses_ros_time(self):
        f = MockFilter()
        cache = Cache(f, cache_size=5, allow_headerless=True)

        f.signalMessage(MockHeaderlessMessage(1, 0, 1))

        self.assertEqual(len(cache.cache_times), 1)
        # Stamped with arrival ROS time, not the (unreadable) message fields.
        self.assertNotEqual(cache.cache_times[0].nanoseconds, 10**9)

    def test_time_synchronizer_raises_without_timestamp(self):
        f0, f1 = MockFilter(), MockFilter()
        TimeSynchronizer([f0, f1], 10)

        with self.assertRaises(ValueError):
            f0.signalMessage(MockHeaderlessMessage(1, 0, 1))

    def test_default_getter_header_message_unchanged(self):
        f = MockFilter()
        cache = Cache(f, cache_size=5)

        f.signalMessage(MockHeaderMessage(7))

        self.assertEqual(len(cache.cache_times), 1)
        self.assertEqual(cache.cache_times[0].nanoseconds, 7 * 10**9)


if __name__ == '__main__':
    unittest.main()
