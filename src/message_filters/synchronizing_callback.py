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

"""Callable wrapper for scheduling asynchronous coroutine callbacks from message filters."""

import asyncio
from concurrent.futures import Future
import copy
import threading
import typing as tp

from rclpy.type_support import MsgT


class SynchronizingCallback:
    """
    Callable wrapper that dispatches asynchronous coroutine callbacks to an asyncio event loop.

    Matches the standard synchronous callback contract expected by message filters
    while scheduling coroutine execution onto the provided ``async_loop`` and
    tracking active and completed futures in a thread-safe manner.
    """

    def __init__(
        self,
        callback: tp.Callable[tp.Concatenate[MsgT, ...], tp.Coroutine[tp.Any, tp.Any, tp.Any]],
        async_loop: asyncio.AbstractEventLoop,
        *args: tp.Any,
        **kwargs: tp.Any,
    ) -> None:
        """
        Construct a SynchronizingCallback instance.

        :param callback: Asynchronous coroutine function to invoke with incoming message(s).
        :param async_loop: Target asyncio event loop on which to schedule the coroutine.
        :param args: Additional positional arguments passed to ``callback`` alongside message(s).
        :param kwargs: Additional keyword arguments passed to ``callback`` alongside message(s).
        """
        self._callback = callback
        self._async_loop = async_loop
        self._args = args
        self._kwargs = kwargs

        self._lock = threading.Lock()
        self._active_futures: set[Future] = set()
        self._completed_futures: set[Future] = set()

    def __call__(self, *messages: MsgT) -> Future:
        """
        Schedule the wrapped coroutine on the event loop with the provided message(s).

        :param messages: Incoming message(s) forwarded by a message filter.
        :return: Concurrent Future representing the scheduled coroutine execution.
        :rtype: Future
        """
        coroutine = self._callback(
            *messages,
            *self._args,
            **self._kwargs,
        )
        future = asyncio.run_coroutine_threadsafe(
            coroutine,
            self._async_loop,
        )

        with self._lock:
            self._active_futures.add(future)
        future.add_done_callback(self._on_future_complete)

        return future

    def _on_future_complete(self, future: Future) -> None:
        """
        Move a completed future from the active set to the completed set.

        :param future: The Future instance that has finished execution.
        """
        with self._lock:
            self._active_futures.discard(future)
            self._completed_futures.add(future)

    @property
    def active_futures(self) -> set[Future]:
        """
        Retrieve a shallow copy of the set of currently active futures.

        :return: Set of in-progress Future instances.
        :rtype: set[Future]
        """
        with self._lock:
            return copy.copy(self._active_futures)

    @property
    def completed_futures(self) -> set[Future]:
        """
        Retrieve a shallow copy of the set of completed futures.

        :return: Set of finished Future instances.
        :rtype: set[Future]
        """
        with self._lock:
            return copy.copy(self._completed_futures)

    def get_result(
        self,
        future: Future,
        timeout: tp.Optional[float] = None,
    ) -> tp.Any:
        """
        Retrieve the result of a completed future managed by this callback wrapper.

        :param future: The Future instance to query.
        :param timeout: Maximum number of seconds to wait for the result, or None.
        :return: The result returned by the coroutine, or None if the future is not tracked.
        :rtype: tp.Any
        :raises asyncio.InvalidStateError: If the future is still active and not yet completed.
        """
        with self._lock:
            is_completed = future in self._completed_futures
            is_active = future in self._active_futures

        if is_completed:
            return future.result(timeout=timeout)
        if is_active:
            raise asyncio.InvalidStateError('Unable to get result. Future is not in done state')
        return None

    def pop_future(self, future: Future) -> None:
        """
        Remove a completed future from the completed futures set.

        :param future: The completed Future instance to remove.
        """
        with self._lock:
            self._completed_futures.discard(future)
