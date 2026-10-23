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

from typing import Callable, Optional

import rclpy

from rclpy.clock import ROSClock
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
from rclpy.type_support import MsgT

from message_filters.message_traits import get_time_from_message_header     # noqa: I100
from message_filters.simple_filter import SimpleFilter


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
        f,
        cache_size=1,
        allow_headerless=False,
        time_getter: Callable[[MsgT], Optional[Time]] = get_time_from_message_header,
    ):
        SimpleFilter.__init__(self)
        self.connectInput(f)
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

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.add)

    def add(self, msg):
        stamp = self.time_getter(msg)
        if stamp is None:
            if not self.allow_headerless:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_cache')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                # TODO: This should be an exception!
                msg_filters_logger.warning('can not use message filters messages '
                                           'without timestamp infomation when '
                                           '"allow_headerless" is disabled. '
                                           'auto assign ROSTIME to headerless '
                                           'messages once enabling constructor '
                                           'option of "allow_headerless".')

                return

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

    def getInterval(self, from_stamp, to_stamp):
        """Query the current cache content between from_stamp to to_stamp."""
        assert from_stamp <= to_stamp

        return [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                if from_stamp <= time <= to_stamp]

    def getElemAfterTime(self, stamp):
        """Return the oldest element after or equal the passed time stamp."""
        newer = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time >= stamp]
        if not newer:
            return None
        return newer[0]

    def getElemBeforeTime(self, stamp):
        """Return the newest element before or equal the passed time stamp."""
        older = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time <= stamp]
        if not older:
            return None
        return older[-1]

    def getLatestTime(self):
        """Return the newest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[-1]

    def getOldestTime(self):
        """Return the oldest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[0]

    def getLast(self):
        if self.getLatestTime() is None:
            return None
        return self.getElemAfterTime(self.getLatestTime())
