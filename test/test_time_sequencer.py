# Copyright 2024, Open Source Robotics Foundation, Inc. All rights reserved.
# Copyright 2024, Martin Llofriu. All rights reserved.
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

from builtin_interfaces.msg import Time as TimeMsg
from message_filters import SimpleFilter, TimeSequencer
import rclpy
from rclpy.duration import Duration


class MockMsg:
    class MockHeader:

        def __init__(self, stamp=None):
            if stamp is None:
                self.stamp = TimeMsg()
            else:
                self.stamp = stamp

    def __init__(self, stamp=None, data=None):
        self.header = MockMsg.MockHeader(stamp)
        self.data = data


class MockCustomStampMsg:

    def __init__(self, custom_stamp, data):
        self.custom_stamp = custom_stamp
        self.data = data


class MockCustomNestedStampMsg:

    def __init__(self, stamp, data):
        self.custom_nested_stamp = MockCustomStampMsg(stamp, data)
        self.data = data


class MockHeaderlessMessage:

    def __init__(self, data):
        self.data = data


class TestTimeSequencer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_time_sequencer_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_time_sequencer_basic(self):
        """Test basic functionality of TimeSequencer with ordered messages."""
        # Create a SimpleFilter to act as the input to TimeSequencer
        input_filter = SimpleFilter()

        # Create a TimeSequencer with a delay and update rate
        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 10
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
        )

        # Generate messages with increasing timestamps
        current_time = self.node.get_clock().now()
        for i in range(5):
            stamp = current_time + Duration(seconds=i * 0.1)
            msg = MockMsg(stamp=stamp.to_msg(), data=f'Message {i}')
            sequencer._add(msg)

        # Check that the messages were received in order after the delay
        expected_data = [f'Message {i}' for i in range(5)]
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            received_data, expected_data, 'Messages were not received in order'
        )

    def test_time_sequencer_out_of_order(self):
        # Create a SimpleFilter to act as the input to TimeSequencer
        input_filter = SimpleFilter()

        # Create a TimeSequencer with a delay and update rate
        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 10
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
        )

        # Generate messages with increasing timestamps
        current_time = self.node.get_clock().now()
        random_order = [2, 0, 4, 1, 3]
        for i in random_order:
            stamp = current_time + Duration(seconds=i * 0.1)
            msg = MockMsg(stamp=stamp.to_msg(), data=f'Message {i}')
            sequencer._add(msg)

        # Check that the messages were received in order after the delay
        expected_data = [f'Message {i}' for i in range(5)]
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            received_data, expected_data, 'Messages were not received in order'
        )

    def test_time_sequencer_queue_size(self):
        input_filter = SimpleFilter()

        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 3  # Small queue size
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
        )

        current_time = self.node.get_clock().now()
        # Generate more messages than the queue size
        for i in range(5):
            stamp = current_time + Duration(seconds=i * 0.1)
            msg = MockMsg(stamp=stamp.to_msg(), data=f'Message {i}')
            sequencer._add(msg)

        # Only the last 'queue_size' messages should have been received
        expected_data = [f'Message {i}' for i in range(2, 5)]
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            received_data,
            expected_data,
            'Queue size limit not enforced correctly',
        )

    def test_time_sequencer_disallow_headerless(self):
        """Test that TimeSequencer discards headerless messages."""
        # Create a SimpleFilter to act as the input to TimeSequencer
        input_filter = SimpleFilter()

        # Create a TimeSequencer with a delay and update rate
        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 10
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
        )

        for i in range(5):
            msg = MockHeaderlessMessage(data=f'Message {i}')
            sequencer._add(msg)

        # The message should not have been received
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            len(received_data),
            0,
            'Headerless message was incorrectly processed',
        )

    def test_time_sequencer_custom_stamp_attribute(self):
        """Test that TimeSequencer can use a custom attribute for timestamp."""
        # Create a SimpleFilter to act as the input to TimeSequencer
        input_filter = SimpleFilter()

        # Create a TimeSequencer with a delay and update rate
        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 10
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
            msg_stamp_attr='custom_stamp',
        )

        # Generate messages with increasing timestamps
        current_time = self.node.get_clock().now()
        for i in range(5):
            stamp = current_time + Duration(seconds=i * 0.1)
            msg = MockCustomStampMsg(
                custom_stamp=stamp.to_msg(), data=f'Message {i}'
            )
            sequencer._add(msg)

        # Check that the messages were received in order after the delay
        expected_data = [f'Message {i}' for i in range(5)]
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            received_data, expected_data, 'Messages were not received in order'
        )

    def test_time_sequencer_custom_nested_stamp_attribute(self):
        """Test that TimeSequencer can use a custom nested attribute for timestamp."""
        # Create a SimpleFilter to act as the input to TimeSequencer
        input_filter = SimpleFilter()

        delay = Duration(seconds=0.5)
        update_rate = Duration(seconds=0.1)
        queue_size = 10
        sequencer = TimeSequencer(
            input_filter,
            delay=delay,
            update_rate=update_rate,
            queue_size=queue_size,
            node=self.node,
            msg_stamp_attr='custom_nested_stamp.custom_stamp',
        )

        current_time = self.node.get_clock().now()
        for i in range(5):
            stamp = current_time + Duration(seconds=i * 0.1)
            msg = MockCustomNestedStampMsg(
                stamp=stamp.to_msg(), data=f'Message {i}'
            )
            sequencer._add(msg)

        expected_data = [f'Message {i}' for i in range(5)]
        received_data = [msg.data for _, _, msg in sequencer.messages]
        self.assertEqual(
            received_data, expected_data, 'Messages were not received in order'
        )


if __name__ == '__main__':
    suite = unittest.TestSuite()
    unittest.TextTestRunner(verbosity=2).run(suite)
