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


from abc import ABC
from functools import partial
from typing import Any, Callable

from message_filters.simple_filter import SimpleFilter
from rclpy.type_support import MsgT


class ComparisonHandler(ABC):
    """Abstract base class for defining message comparison logic."""

    def message_fits(self, message: MsgT) -> bool:
        """
        Evaluate if the incoming message satisfies the filtering conditions.

        :param message: The incoming message to evaluate.
        :return: True if the message fits the comparison criteria, False otherwise.
        :rtype: bool
        :raises NotImplementedError: If the method is not overridden in a subclass.
        """
        raise NotImplementedError()

    def do_fields_fit(self, field_a: Any, field_b: Any) -> bool:
        """
        Compare two individual message fields.

        :param field_a: The first field value to compare (usually from a cached message).
        :param field_b: The second field value to compare (usually from the current message).
        :return: True if the fields satisfy the comparison match, False otherwise.
        :rtype: bool
        :raises NotImplementedError: If the method is not overridden in a subclass.
        """
        raise NotImplementedError()


class CachedComparisonHandler(ComparisonHandler):
    """
    Implements cached messages comparison field by field.

    CachedComparisonHandler implements messages comparison field by field.
    A successor to this class should implement ``do_fields_fit`` method.
    If any of the fields provided to the ``do_fields_fit`` method do satisfy
    a comparison conditions the message is accepted. That means that the message
    is stored in the ``_message_cache`` and the ``message_fits`` method returns ``True``.
    """

    def __init__(self):
        """Construct a CachedComparisonHandler."""
        super().__init__()

        self._message_cache: MsgT | None = None

    def message_fits(self, message: MsgT) -> bool:
        """
        Evaluate the message against the cached message field by field.

        If the cache is empty, the message is automatically accepted and cached.
        Otherwise, it applies ``do_fields_fit`` across all registered field getters.
        If a match is found, the internal cache is updated with the new message.

        :param message: The incoming message to evaluate.
        :return: True if the cache is empty or if any field comparison passes, False otherwise.
        :rtype: bool
        """
        if self._message_cache is None:
            self._message_cache = message
            return True

        any_field_fits = any(
            self.do_fields_fit(
                get_field(self._message_cache),
                get_field(message),
            )
            for get_field in self._field_getters
        )

        if any_field_fits:
            self._message_cache = message
        return any_field_fits

    def do_fields_fit(self, field_a: Any, field_b: Any) -> bool:
        """
        Compare two individual fields to determine if they fit criteria.

        :param field_a: The field value from the cached message.
        :param field_b: The field value from the current message.
        :return: True if the fields match the condition.
        :rtype: bool
        :raises NotImplementedError: Subclasses must implement this method.
        """
        raise NotImplementedError(
            f'Field comparison not implemented for class: {self.__class__.__name__}. '
            f'Consider implementing do_fields_fit'
        )


class DeltaCompare(CachedComparisonHandler):
    """
    Implements delta comparison.

    If any of the fields, acquired by any of the ``field_getters`` do differ
    between previously cached and current message, the current message is accepted.
    """

    _basic_types = (
        bool,
        int,
        float,
        str,
        # list requires a specific comparison function
    )

    def __init__(
        self,
        field_getters: Callable[[MsgT], Any],
    ):
        """
        Construct DeltaCompare.

        :param field_getters: A list of callable objects that are expected
            to retrieve a value of a basic type from a message field for comparison.
            Any of the ``field_getters`` is applied to the cached and to the current
            message. The returned values are compared. If any of these values differ
            between cached and current message, the current message is accepted.
        """
        super().__init__()

        self._field_getters = field_getters

    def do_fields_fit(self, val_a: Any, val_b: Any) -> bool:
        """
        Check if the values of two fields are different (delta check).

        Ensures both values are of the supported basic types before performing the comparison.

        :param val_a: The field value from the cached message.
        :param val_b: The field value from the incoming message.
        :return: True if the field values are different, False if they are equal.
        :rtype: bool
        :raises TypeError: If either of the values is not one of the supported basic types.
        """
        if any(not isinstance(val, self._basic_types) for val in [val_a, val_b]):
            raise TypeError(f'Unable to compare fields of types {type(val_a)} and {type(val_b)}')
        return val_a != val_b


class ComparisonFilter(SimpleFilter):
    """
    ROS 2 Comparison filter.

    Given a stream of messages, the message is passed down to the next filter
    if ``comparison_handler`` ``message_fits`` method returns ``True`` for that message.
    """

    def __init__(
        self,
        comparison_handler: CachedComparisonHandler,
        message_filter: SimpleFilter | None = None,
    ):
        """
        Construct ComparisonFilter.

        :param comparison_handler: An instance of the CachedComparisonHandler class.
            Is expected implement a ``message_fits`` method. If ``message_fits`` returns ``True``
            for a provided message, that message is considered valid and is passed
            to a next filter if any.
        :param message_filter: An instance of the ``SimpleFilter``. The input filter to connect to
        """
        super().__init__()

        self._comparison_handler = comparison_handler

        self.incoming_connection = None
        self.connectInput(message_filter)

    def add(self, message: MsgT):
        """
        Process an incoming message.

        If the message satisfies the internal comparison handler conditions,
        it is propagated forward to the registered callbacks.

        :param message: The incoming message to process.
        """
        if self.message_fits(message):
            self.signalMessage(message)

    def message_fits(self, message: MsgT):
        """
        Check if the message satisfies the comparison filter logic.

        :param message: The incoming message to evaluate.
        :return: True if the message fits the criteria, False otherwise.
        :rtype: bool
        """
        return self._comparison_handler.message_fits(message)

    def connectInput(self, message_filter: SimpleFilter):
        """
        Connect this filter to an upstream input filter.

        :param message_filter: The upstream filter providing incoming messages.
        :raises RuntimeError: If this filter is already connected to an incoming source.
        """
        if self.incoming_connection is not None:
            raise RuntimeError(
                'This instance of ComparisonFilter already has an incoming connection'
            )
        if message_filter is not None:
            self.incoming_connection = message_filter.registerCallback(self.add)


class DeltaFilter(ComparisonFilter):
    """
    ROS 2 Delta filter.

    Given a stream of messages, the message is passed down to the next filter
    if any of the message fields, that may be acquired by ``field_getters``
    have changed compared to the previously accepted message.
    """

    def __init__(
        self,
        field_getters: list[Callable[[MsgT], Any]],
        message_filter: SimpleFilter | None = None,
    ):
        """
        Construct a DeltaFilter.

        :param field_getters: A list of callable objects each of which returns a field value.
            When message is processed, every field getter is applied to it and to the previously
            accepted message. If any of the fields have changed between currently processed message
            and previous value, the message is accepted, stored to the ``message_cache`` and passed
            to a next message filter if any.
        :param message_filter: A message filter to connect this filter to as a follow up.
        """
        super().__init__(
            comparison_handler=DeltaCompare(field_getters=field_getters),
            message_filter=message_filter,
        )


class PathDeltaFilter(DeltaFilter):
    """
    ROS 2 Path Delta Filter.

    Given a stream of messages, the message is passed down to the next filter
    if any of the message fields, that may be found at any ``field_path``
    of the ``field_path_list`` have changed compared to the previously accepted message.
    """

    def __init__(
        self,
        field_path_list: list[str],
        message_filter: SimpleFilter | None = None
    ):
        """
        Construct a DeltaFilter.

        :param field_path_list: A list of field paths to access fields by.
            A field path is expected to be a dot-separated string. Eg:
            ``a.b.c.d`` where ``d`` is a field of ``c``, ``c`` is a field of ``b`` e.t.c.
        :param message_filter: A message filter to connect this filter to as a follow up.
        """
        super().__init__(
            field_getters=[
                partial(
                    self.get_field_by_path,
                    field_path=field_path,
                )
                for field_path in field_path_list
            ],
            message_filter=message_filter,
        )

    def get_field_by_path(self, message: MsgT, field_path: str):
        """
        Extract a nested field value from a message using a dot-separated string path.

        Dynamically navigates through the attributes of the message structure.

        :param message: The message object containing the desired field.
        :param field_path: A dot-separated string path pointing to the target nested field.
        :return: The extracted field value.
        :rtype: Any
        :raises RuntimeError: If any attribute along the path evaluates to None or doesn't exist.
        """
        res = message

        for field_name in field_path.split('.'):
            prev = res
            res = getattr(res, field_name, None)
            if res is None:
                raise RuntimeError(
                     f'Unable to access field {field_name!r} of '
                     f'{prev.__class__.__name__} (full path: {field_path!r})'
                )
        return res
