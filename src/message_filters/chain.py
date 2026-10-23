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

from dataclasses import dataclass

from .simple_filter import SimpleFilter


class Chain(SimpleFilter):
    """
    Chains a dynamic number of simple filters together.

    Allows retrieval of filters by index after they are added.

    The Chain filter provides a container for simple filters.
    It allows you to store an N-long set of filters inside a single
    structure, making it much easier to manage them.

    Adding filters to the chain is done by adding shared_ptrs of them
    to the filter. They are automatically connected to each other
    and the output of the last filter in the chain is forwarded
    to the callback you've registered with Chain::registerCallback.
    """

    @dataclass
    class FilterInfo:
        message_filter: any
        connection_callback_index: int

    def __init__(self, message_filter=None):
        SimpleFilter.__init__(self)

        self.incoming_connection = None

        if message_filter is not None:
            self.connectInput(message_filter)

        self._message_filters: dict[int, Chain.FilterInfo] = {}

    def connectInput(self, message_filter):
        if self.incoming_connection is not None:
            raise RuntimeError('Already connected')
        self.incoming_connection = message_filter.registerCallback(self.add)

    def add(self, message):
        if self._message_filters:
            self._message_filters[0].message_filter.add(message)
        else:
            self.signalMessage(message)

    def addFilter(self, message_filter):
        new_filter_index = len(self._message_filters)
        last_filter_index = new_filter_index - 1

        self._message_filters[new_filter_index] = Chain.FilterInfo(
            message_filter=message_filter,
            connection_callback_index=message_filter.registerCallback(self.signalMessage),
        )

        if last_filter_index >= 0:
            last_filter = self._message_filters[last_filter_index].message_filter
            callback_index = self._message_filters[last_filter_index].connection_callback_index
            last_filter.callbacks.pop(callback_index)

            self._message_filters[last_filter_index].connection_callback_index = \
                last_filter.registerCallback(message_filter.add)

    def getFilter(self, index: int):
        return self._message_filters[index].message_filter
