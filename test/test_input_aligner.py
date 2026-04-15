import time
import unittest

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import InputAligner, SimpleFilter
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
        self.cb_content = []

    def cb(self, msg):
        self.cb_content.append(msg.data)

    def create_msg(self, cls, milliseconds, data):
        return cls(stamp=Time(nanoseconds=int(milliseconds * 1e6)).to_msg(), data=data)

    def test_init(self):
        f0, f1, f2, f3 = SimpleFilter(), SimpleFilter(), SimpleFilter(), SimpleFilter()
        aligner1 = InputAligner(self.timeout, f0, f1, f2, f3)
        self.assertEqual(len(aligner1.event_queues), 4)
        aligner2 = InputAligner(self.timeout)
        aligner2.connectInput(f0, f2, f3)
        self.assertEqual(len(aligner2.event_queues), 3)

    def test_dispatch_inputs_in_order(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter(), SimpleFilter(), SimpleFilter())
        for i in range(4):
            aligner.registerCallback(i, self.cb)
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
        self.assertEqual(self.cb_content, list(range(1, 10)))

    def test_ignores_inactive_inputs(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter(), SimpleFilter())
        for i in range(3):
            aligner.registerCallback(i, self.cb)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg1, 2, 2), 2)
        aligner.add(self.create_msg(Msg2, 1, 1), 1)
        aligner.add(self.create_msg(Msg1, 4, 4), 2)
        aligner.add(self.create_msg(Msg2, 3, 3), 1)
        aligner.add(self.create_msg(Msg2, 5, 5), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [1, 2, 3, 4, 5])

    def test_input_timeout(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter())
        for i in range(2):
            aligner.registerCallback(i, self.cb)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        for i in range(1, 17, 2):
            aligner.add(self.create_msg(Msg1, i, i), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [1, 2, 3, 4, 5])
        aligner.add(self.create_msg(Msg1, 17, 17), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [1, 2, 3, 4, 5, 7, 9, 11, 13, 15, 17])

    def test_drops_msgs(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter())
        for i in range(2):
            aligner.registerCallback(i, self.cb)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg2, 4, 4), 1)
        aligner.add(self.create_msg(Msg1, 3, 3), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [3, 4])
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        aligner.add(self.create_msg(Msg1, 5, 5), 0)
        aligner.add(self.create_msg(Msg1, 7, 7), 0)
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg2, 6, 6), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [3, 4, 5, 6, 7])

    def test_dispatch_by_timer(self):
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter())
        aligner.setupDispatchTimer(self.node, self.update_rate)
        for i in range(2):
            aligner.registerCallback(i, self.cb)
            aligner.setInputPeriod(i, Duration(nanoseconds=int(2e6)))
        aligner.add(self.create_msg(Msg2, 2, 2), 1)
        aligner.add(self.create_msg(Msg1, 1, 1), 0)
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(self.cb_content, [1, 2])

    def test_no_period_information(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter(), SimpleFilter())
        for i in range(3):
            aligner.registerCallback(i, self.cb)
        aligner.add(self.create_msg(Msg1, 6, 6), 0)
        aligner.add(self.create_msg(Msg1, 2, 2), 2)
        aligner.add(self.create_msg(Msg1, 4, 4), 2)
        aligner.add(self.create_msg(Msg2, 1, 1), 1)
        aligner.add(self.create_msg(Msg2, 3, 3), 1)
        aligner.add(self.create_msg(Msg2, 5, 5), 1)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [1, 2, 3, 4])
        aligner.add(self.create_msg(Msg1, 16, 16), 0)
        aligner.dispatchMessages()
        self.assertEqual(self.cb_content, [1, 2, 3, 4, 5, 6, 16])

    def test_get_queue_status(self):
        self.timeout = Duration(nanoseconds=int(1e7))
        aligner = InputAligner(self.timeout)
        aligner.connectInput(SimpleFilter(), SimpleFilter())
        for i in range(2):
            aligner.registerCallback(i, self.cb)
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
