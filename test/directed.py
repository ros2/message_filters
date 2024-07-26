# Copyright 2010, Willow Garage, Inc. All rights reserved.
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

#  image_transport::SubscriberFilter wide_left;   // "/wide_stereo/left/image_raw"
#  image_transport::SubscriberFilter wide_right;  // "/wide_stereo/right/image_raw"
#  message_filters::Subscriber<CameraInfo> wide_left_info;    // "/wide_stereo/left/camera_info"
#  message_filters::Subscriber<CameraInfo> wide_right_info;   // "/wide_stereo/right/camera_info"
#  message_filters::TimeSynchronizer<Image, CameraInfo, Image, CameraInfo> wide;
#
#  PersonDataRecorder() :
#    wide_left(nh_, "/wide_stereo/left/image_raw", 10),
#    wide_right(nh_, "/wide_stereo/right/image_raw", 10),
#    wide_left_info(nh_, "/wide_stereo/left/camera_info", 10),
#    wide_right_info(nh_, "/wide_stereo/right/camera_info", 10),
#    wide(wide_left, wide_left_info, wide_right, wide_right_info, 4),
#
#    wide.registerCallback(boost::bind(&PersonDataRecorder::wideCB, this, _1, _2, _3, _4));

import random
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import SimpleFilter, TimeSynchronizer


class MockHeader:
    pass


class MockMessage:

    def __init__(self, stamp, data):
        self.header = MockHeader()
        self.header.stamp = TimeMsg(sec=stamp)
        self.data = data


class MockFilter(SimpleFilter):
    pass


class TestDirected(unittest.TestCase):

    def cb_collector_2msg(self, msg1, msg2):
        self.collector.append((msg1, msg2))

    def test_synchronizer(self):
        m0 = MockFilter()
        m1 = MockFilter()
        ts = TimeSynchronizer([m0, m1], 1)
        ts.registerCallback(self.cb_collector_2msg)

        if 0:
            # Simple case, pairs of messages, make sure that they get combined
            for t in range(10):
                self.collector = []
                msg0 = MockMessage(t, 33)
                msg1 = MockMessage(t, 34)
                m0.signalMessage(msg0)
                self.assertEqual(self.collector, [])
                m1.signalMessage(msg1)
                self.assertEqual(self.collector, [(msg0, msg1)])

        # Scramble sequences of length N.
        # Make sure that TimeSequencer recombines them.
        random.seed(0)
        for N in range(1, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(t, random.random()) for t in range(N)]
            seq1 = [MockMessage(t, random.random()) for t in range(N)]
            # random.shuffle(seq0)
            ts = TimeSynchronizer([m0, m1], N)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            for msg in random.sample(seq0, N):
                m0.signalMessage(msg)
            self.assertEqual(self.collector, [])
            for msg in random.sample(seq1, N):
                m1.signalMessage(msg)
            self.assertEqual(set(self.collector), set(zip(seq0, seq1)))


if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestDirected('test_synchronizer'))
    unittest.TextTestRunner(verbosity=2).run(suite)
