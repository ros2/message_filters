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


from typing import Optional, Type, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.subscription_content_filter_options import ContentFilterOptions
from rclpy.type_support import MsgT

from .simple_filter import SimpleFilter


class Subscriber(SimpleFilter):
    """
    ROS 2 subscription filter, takes identical arguments as :class:`rclpy.Subscriber`.

    This class acts as a highest-level filter, simply passing messages
    from a ROS 2 subscription through to the filters which have connected
    to it.
    """

    def __init__(
        self,
        node: Node,
        msg_type: Type[MsgT],
        topic: str,
        qos_profile: Union[QoSProfile, int] = QoSProfile(depth=10),
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False,
        content_filter_options: Optional[ContentFilterOptions] = None
    ) -> None:
        """
        Construct a Subscriber.

        :param node: The node to create a subscriber for.
        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param qos_profile: A QoSProfile or a history depth to apply to the
            subscription. In the case that a history depth is provided, the QoS history is
            set to KEEP_LAST, the QoS history depth is set to the value of the parameter,
            and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            default callback group for the node is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        :param content_filter_options: The filter expression and parameters for content filtering.
        """
        SimpleFilter.__init__(self)
        self.node = node
        self.topic = topic
        self.sub = self.node.create_subscription(
            msg_type=msg_type,
            topic=self.topic,
            callback=self.callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            raw=raw,
            content_filter_options=content_filter_options,
        )

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

    def __getattr__(self, key):
        """Serve same API as rospy.Subscriber."""
        return self.sub.__getattribute__(key)
