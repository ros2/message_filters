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

from typing import Callable, List, Optional

from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.type_support import MsgT

from message_filters.message_traits import get_time_from_message_header     # noqa: I100
from message_filters.simple_filter import SimpleFilter      # noqa: I100
from message_filters.utilities import generate_deprecation_warning


class Cache(SimpleFilter):
    """
    Stores a time history of messages.

    Given a stream of messages, the most recent ``cache_size`` messages
    are cached in a ring buffer, from which time intervals of the cache
    can then be retrieved by the client. The ``allow_headerless``
    option specifies whether to allow storing headerless messages with
    current ROS time instead of timestamp. You should avoid this as
    much as you can, since the delays are unpredictable.
    """

    def __init__(
        self,
        message_filter: Optional[SimpleFilter] = None,      # Replaces 'f'
        cache_size: int = 1,
        allow_headerless: bool = False,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
        *,
        f: Optional[SimpleFilter] = None,       # Replaced my 'message_filter'
    ):
        """
        Construct a Cache filter.

        :param message_filter: The input filter to connect to and monitor
            for incoming messages.
        :param cache_size: The maximum number of messages to keep in
            the ring buffer.
        :param allow_headerless: If True, allow storing messages
            without a header timestamp by assigning the current ROS time.
            If False, raises a RuntimeError for headerless messages.
        :param time_getter: A callable function to extract the timestamp
            from a message.
            Defaults to extracting from the standard message header.
        :param f: Deprecated alias for ``message_filter``.
        """
        SimpleFilter.__init__(self)

        if f is not None:
            generate_deprecation_warning(
                old_arg_name='f',
                new_arg_name='message_filter',
            )
        if not message_filter and not f:
            raise TypeError(
                'Cache.__init__ '
                'missing required argument: "input_filters"'
            )
        message_filter = message_filter if message_filter is not None else f

        self.connectInput(message_filter)
        self.cache_size = cache_size
        # Array to store messages
        self.cache_msgs = []
        # Array to store msgs times, auxiliary structure to facilitate
        # sorted insertion
        self.cache_times = []
        # Whether to allow storing headerless messages with current ROS
        # time instead of timestamp.
        self.allow_headerless = allow_headerless

        self.time_getter = time_getter

    def connectInput(
        self,
        message_filter: Optional[SimpleFilter] = None,      # Replaces 'f'
        *,
        f: Optional[SimpleFilter] = None,       # Replaced by 'message_filter'
    ):
        """
        Connect the cache to an input message filter stream.

        Registers the cache's ``add`` method as a callback
        to the provided filter.

        :param message_filter: The input filter to connect to.
        :param f: Deprecated alias for ``message_filter``.
        """
        if f is not None:
            generate_deprecation_warning(
                old_arg_name='f',
                new_arg_name='message_filter',
            )
        if not message_filter and not f:
            raise TypeError(
                'Cache.connectInput '
                'missing required argument: "input_filters"'
            )
        message_filter = message_filter if message_filter is not None else f
        self.incoming_connection = message_filter.registerCallback(self.add)

    def add(self, msg: MsgT):
        """
        Process and add a new message to the cache.

        Extracts the timestamp from the message.
        If the buffer size exceeds ``cache_size``,
        the oldest message is discarded. Finally,
        triggers notifications for down-stream filters.

        :param msg: The incoming message to add to the cache ring buffer.
        :raises RuntimeError: If the message lacks a timestamp
            and ``allow_headerless`` is False.
        """
        stamp = self.time_getter(msg)
        if stamp is None:
            if not self.allow_headerless:
                # Same behavior as a C++ implementation
                raise RuntimeError(
                    'Can not use message filters messages '
                    'without timestamp information when '
                    '"allow_headerless" is disabled. '
                    'Auto assign ROSTIME to headerless '
                    'messages once enabling constructor '
                    'option of "allow_headerless".'
                )

            stamp = ROSClock().now()
        # Insert sorted
        self.cache_times.append(stamp)
        self.cache_msgs.append(msg)

        # Implement a ring buffer, discard older if oversized
        if (len(self.cache_msgs) > self.cache_size):
            del self.cache_msgs[0]
            del self.cache_times[0]

        # Signal new input
        self.signalMessage(msg)

    def getInterval(self, from_stamp: Time, to_stamp: Time) -> List[MsgT]:
        """
        Query the current cache content between from_stamp to to_stamp.

        :param from_stamp: The start timestamp of the desired interval (inclusive).
        :param to_stamp: The end timestamp of the desired interval (inclusive).
        :return: A list of messages that fall within the specified time interval.
        :rtype: list[MsgT]
        """
        assert from_stamp <= to_stamp

        return [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                if from_stamp <= time <= to_stamp]

    def getElemAfterTime(self, stamp: Time) -> Optional[MsgT]:
        """
        Return the oldest element after or equal the passed time stamp.

        :param stamp: The reference timestamp threshold.
        :return: The oldest message with a timestamp >= ``stamp``,
            or None if no such message exists.
        :rtype: Optional[MsgT]
        """
        newer = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time >= stamp]
        if not newer:
            return None
        return newer[0]

    def getElemBeforeTime(self, stamp: Time) -> Optional[MsgT]:
        """
        Return the newest element before or equal the passed time stamp.

        :param stamp: The reference timestamp threshold.
        :return: The newest message with a timestamp <= ``stamp``,
            or None if no such message exists.
        :rtype: Optional[MsgT]
        """
        older = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time <= stamp]
        if not older:
            return None
        return older[-1]

    def getLatestTime(self) -> Optional[Time]:
        """
        Return the newest recorded timestamp.

        :return: The timestamp of the most recently added message,
            or None if the cache is empty.
        :rtype: Optional[Time]
        """
        if not self.cache_times:
            return None
        return self.cache_times[-1]

    def getOldestTime(self) -> Optional[Time]:
        """
        Return the oldest recorded timestamp.

        :return: The timestamp of the oldest message still in the cache,
            or None if the cache is empty.
        :rtype: Optional[Time]
        """
        if not self.cache_times:
            return None
        return self.cache_times[0]

    def getLast(self) -> Optional[MsgT]:
        """
        Return the newest message in the cache.

        :return: The most recently recorded message object, or None if the cache is empty.
        :rtype: Optional[MsgT]
        """
        if self.getLatestTime() is None:
            return None
        return self.getElemAfterTime(self.getLatestTime())
