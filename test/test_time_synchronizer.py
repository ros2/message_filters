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

import functools
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import SimpleFilter, Subscriber, Cache, TimeSynchronizer


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

    @staticmethod
    def collector_callback(msg1, msg2, collector):
        collector.append((msg1, msg2))

    def test_time_synchronizer_queue_lentgth(self):
        for N in range(1, 10):
            seq0 = [MockMessage(t, 0) for t in range(N)]
            seq1 = [MockMessage(t, 0) for t in range(N)]

            m0 = MockFilter()
            m1 = MockFilter()
            ts = TimeSynchronizer([m0, m1], N)

            collector = []
            ts.registerCallback(
                functools.partial(
                    self.collector_callback,
                    collector=collector,
                )
            )

            for msg in seq0:
                m0.signalMessage(msg)
            self.assertEqual(collector, [])
            for msg in seq1:
                m1.signalMessage(msg)
            self.assertEqual(set(collector), set(zip(seq0, seq1)))

    def test_time_synchronizer_drop_old_messages(self):
        collector = []

        filter_0 = MockFilter()
        filter_1 = MockFilter()
        ts = TimeSynchronizer([filter_0, filter_1], 10)
        ts.registerCallback(
            functools.partial(
                self.collector_callback,
                collector=collector,
            )
        )

        t1 = 0
        t2 = 1

        x0 = MockMessage(t1, 1)
        x1 = MockMessage(t1, 2)

        y0 = MockMessage(t2, 1)
        y1 = MockMessage(t2, 2)

        filter_0.signalMessage(x0)
        assert len(collector) == 0

        filter_1.signalMessage(y1)
        assert len(collector) == 0

        filter_0.signalMessage(y0)
        assert len(collector) == 1
        assert collector[0] == (y0, y1)

        filter_1.signalMessage(x1)
        assert len(collector) == 1
        assert collector[0] == (y0, y1)

    def test_time_synchronizer_shifted_time_signalling(self):
        collector = []

        filter_0 = MockFilter()
        filter_1 = MockFilter()
        ts = TimeSynchronizer([filter_0, filter_1], 10)
        ts.registerCallback(
            functools.partial(
                self.collector_callback,
                collector=collector,
            )
        )

        t1 = 0
        t2 = 1

        x0 = MockMessage(t1, 1)
        x1 = MockMessage(t1, 2)

        y0 = MockMessage(t2, 1)
        y1 = MockMessage(t2, 2)

        filter_0.signalMessage(x0)
        assert len(collector) == 0

        filter_0.signalMessage(y0)
        assert len(collector) == 0

        filter_1.signalMessage(x1)
        assert len(collector) == 1
        assert collector[0] == (x0, x1)

        filter_1.signalMessage(y1)
        assert len(collector) == 2
        assert collector[0] == (x0, x1)
        assert collector[1] == (y0, y1)

if __name__ == '__main__':
    suite = unittest.TestSuite()
    suite.addTest(TestDirected('test_time_synchronizer_queue_lentgth'))
    suite.addTest(TestDirected('test_time_synchronizer_drop_old_messages'))
    suite.addTest(TestDirected('test_time_synchronizer_shifted_time_signalling'))
    unittest.TextTestRunner(verbosity=2).run(suite)
