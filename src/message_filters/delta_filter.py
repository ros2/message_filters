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

from message_filters import SimpleFilter
from rclpy.type_support import MsgT


class ComparisonHandler(ABC):

    def message_fits(self, message: MsgT) -> bool:
        raise NotImplementedError()

    def do_fields_fit(self, field_a: Any, field_b: Any) -> bool:
        raise NotImplementedError()


class CachedComparisonHandler(ComparisonHandler):
    """
    Implements cached messages comparison field by field.

    :class: CachedComparisonHandler implements messages comparison field by field.
    A successor to this class should implement ``do_fields_fit`` method.
    If any of the fields provided to the ``do_fields_fit`` method do satisfy
    a comparison conditions the message is accepted. That means that the message
    is stored in the ``_message_cache`` and the ``message_fits`` method returns ``True``.
    """

    def __init__(self):
        super().__init__()

        self._message_cache: MsgT | None = None

    def message_fits(self, message: MsgT) -> bool:
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

    def do_fields_fit(self, val_a, val_b) -> bool:
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
        if self.message_fits(message):
            self.signalMessage(message)

    def message_fits(self, message: MsgT):
        return self._comparison_handler.message_fits(message)

    def connectInput(self, message_filter):
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
