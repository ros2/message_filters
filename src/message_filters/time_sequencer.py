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

from bisect import insort_right
import threading
from typing import Any, Optional, Union

from builtin_interfaces.msg import Time as TimeMsg

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.type_support import MsgT

from message_filters.simple_filter import SimpleFilter     # noqa: I100


class TimeSequencer(SimpleFilter):
    """
    Sequences messages based on the timestamp of their header.

    At construction, the TimeSequencer takes a duration 'delay' which specifies
    how long to queue up messages to provide a time sequencing over them.
    As messages arrive, they are sorted according to their timestamps.
    A callback for a message is never invoked until the messages' timestamp is
    out of date by at least the delay. However, for all messages which are out of
    date by at least delay, their callbacks are invoked in temporal order.
    If a message arrives from a time prior to a message which has already had its
    callback invoked, it is thrown away.
    """

    def __init__(
        self,
        input_filter: SimpleFilter,
        delay: Union[Duration, float],
        update_rate: Union[Duration, float],
        queue_size: int,
        node: Node,
        msg_stamp_attr: str = 'header.stamp',
    ) -> None:
        """
        Construct a TimeSequencer filter for a subscriber.

        :param input_filter: The input filter to connect to. Typically a Subscriber.
        :param delay: The delay (in seconds) to wait for messages to arrive before
            dispatching them.
        :param update_rate: The rate at which to check for messages that are ready to be
            dispatched.
        :param queue_size: The maximum number of messages to store. Set 0 for no limit.
        :param node: The node to create the timer on.
        :param msg_stamp_attr: The attribute to use for retrieving the timestamp from the
            message. Should point to a builtin_interfaces.msg.Time field.
            Defaults to ``'header.stamp'``.
        """
        super().__init__()
        if not isinstance(delay, Duration):
            delay = Duration(seconds=delay)
        if not isinstance(update_rate, Duration):
            update_rate = Duration(seconds=update_rate)
        self.delay: float = delay
        self.update_rate: float = update_rate
        self.queue_size: int = queue_size
        self.lock = threading.Lock()
        self.messages = []
        self.last_time: Time = Time()
        self.node: Node = node
        self.msg_stamp_attrs = msg_stamp_attr.split('.')
        self.update_timer = self.node.create_timer(
            self.update_rate.nanoseconds / 1e9, self._dispatch
        )
        self.incoming_connection = None
        if input_filter is not None:
            self.connectInput(input_filter)

    def _getMsgStampAttr(self, msg: MsgT) -> Optional[Any]:
        """
        Dynamically resolve and retrieve the configured timestamp attribute from the message.

        Recursively traverses the nested attributes specified during construction
        to find the target timestamp field.

        :param msg: The incoming message object to inspect.
        :return: The resolved attribute value if found, or None if the path does not exist.
        :rtype: Any or None
        """
        obj = msg
        for attr in self.msg_stamp_attrs:
            if not hasattr(obj, attr):
                return None
            obj = getattr(obj, attr)
        return obj

    def connectInput(self, input_filter: SimpleFilter) -> None:
        """
        Connect the sequencer to an upstream input filter pipeline.

        Registers an internal callback to intercept incoming messages from the
        specified filter. Only one source input filter can be connected at a time.

        :param input_filter: The upstream simple filter pipeline to listen to.
        :type input_filter: SimpleFilter
        :raises RuntimeError: If this sequencer is already connected to an input filter.
        """
        if self.incoming_connection is not None:
            raise RuntimeError('Already connected to an input filter.')
        self.incoming_connection = input_filter.registerCallback(self._add)

    def _add(self, msg: MsgT) -> None:
        """
        Process and insert a new incoming message into the temporal sequencing queue.

        Extracts the timestamp from the message, filters out messages that arrive out of
        chronological order (older than the last dispatched timestamp), and stores the valid
        messages sorted by time. If the internal queue limit is exceeded, the oldest stored
        message is dropped.

        :param msg: The incoming message object forwarded from the input filter.
        """
        with self.lock:
            stamp = self._getStamp(msg)
            if stamp is None:
                return
            if stamp.nanoseconds < self.last_time.nanoseconds:
                return
            # Insert msg into messages in sorted order
            insort_right(self.messages, (stamp, msg))
            # If queue_size is exceeded, remove the earliest message
            if self.queue_size != 0 and len(self.messages) > self.queue_size:
                del self.messages[0]

    def _getStamp(self, msg: MsgT) -> Optional[Time]:
        """
        Extract and validate the ROS 2 Time representation from the message metadata.

        Uses the resolved attribute path to locate the timestamp, ensures it matches the
        expected ROS 2 message Time type, and converts it into a `rclpy.time.Time` object.

        :param msg: The message object to extract the timestamp from.
        :return: The validated Time instance, or None if the attribute is missing.
        :rtype: rclpy.time.Time or None
        :raises TypeError: If the extracted attribute does not match the builtin TimeMsg type.
        """
        stamp = self._getMsgStampAttr(msg)
        if stamp is not None:
            if not isinstance(stamp, TimeMsg):
                raise TypeError(
                    f'Expected {TimeMsg}, got {type(stamp)} in msg attribute '
                    f"{'.'.join(self.msg_stamp_attrs)}"
                )
            stamp = Time.from_msg(stamp)
            return stamp
        else:
            self.node.get_logger().warning(
                'Cannot use message without timestamp; discarding message.'
            )
            return None

    def _dispatch(self) -> None:
        """
        Periodically evaluate the queue and broadcast ready messages to downstream handlers.

        Iterates through the sorted queue to find messages whose timestamps are older
        than the current time minus the configured delay duration. Those messages are
        popped out and signaled in order to all registered callbacks.
        """
        to_call = []
        with self.lock:
            while self.messages:
                stamp, msg = self.messages[0]
                if stamp + self.delay <= self.node.get_clock().now():
                    self.last_time = stamp
                    # Remove message from messages
                    self.messages.pop(0)
                    to_call.append(msg)
                else:
                    break
        for msg in to_call:
            self.signalMessage(msg)

    def shutdown(self) -> None:
        """
        Clean up the TimeSequencer.

        Cancels the active internal dispatch timer and safely destroys its reference
        within the associated ROS 2 node context.
        """
        self.update_timer.cancel()
        self.node.destroy_timer(self.update_timer)
