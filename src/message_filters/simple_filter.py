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
import asyncio
import inspect
from itertools import count
import types
import typing as tp

import rclpy.executors
from rclpy.node import Node
import rclpy.task
from rclpy.type_support import MsgT


class SimpleFilter:
    """
    Base class for simple message filters in ROS 2.

    Provides core mechanisms for managing downstream callback functions and
    dispatching incoming or filtered messages to all registered hooks.
    Supports both synchronous and asynchronous coroutine callbacks.
    """

    def __init__(self) -> None:
        """Construct a SimpleFilter instance."""
        self.callbacks: tp.Dict[
            int,
            tp.Tuple[
                tp.Callable[tp.Concatenate[MsgT, ...], tp.Any],
                tp.Tuple[tp.Any, ...],
                tp.Optional[rclpy.executors.Executor],
            ],
        ] = {}
        self.callbacks_counter = count()
        self._node: tp.Optional[Node] = None
        self.input_filters: tp.List['SimpleFilter'] = []

    @property
    def node(self) -> tp.Optional[Node]:
        """
        Retrieve the ROS 2 Node associated with this filter, if any.

        :return: The associated Node instance, or None if not set.
        """
        return self._node

    @node.setter
    def node(self, value: tp.Optional[Node]) -> None:
        """
        Set the ROS 2 Node associated with this filter.

        :param value: The Node instance to associate with this filter.
        """
        self._node = value

    def registerCallback(
        self,
        callback: tp.Callable[tp.Concatenate[MsgT, ...], tp.Any],
        *args: tp.Any,
        executor: tp.Optional[rclpy.executors.Executor] = None,
    ) -> int:
        """
        Register a callback `callback` to be called when this filter has output.

        The filter calls the function ``callback`` with the incoming message(s),
        followed by the static registration-supplied arguments ``args``. If ``callback``
        returns a coroutine, it is scheduled for asynchronous execution on an rclpy
        executor or running asyncio event loop.

        :param callback: The callable handler to register for incoming data events.
        :param args: Additional static arguments to pass to the callback upon invocation.
        :param executor: Optional explicit rclpy Executor to schedule coroutine callbacks.
        :return: A unique connection index token representing this registration.
        :rtype: int
        """
        conn = next(self.callbacks_counter)
        self.callbacks[conn] = (callback, args, executor)
        if (
            isinstance(callback, types.MethodType)
            and isinstance(callback.__self__, SimpleFilter)
            and self not in callback.__self__.input_filters
        ):
            callback.__self__.input_filters.append(self)
        return conn

    def signalMessage(self, *msg: tp.Any) -> None:
        """
        Broadcast a message (or multiple messages) to all registered callbacks.

        Executes all registered callbacks with the msg followed by the static
        arguments provided at the time of each callback's registration.
        If a callback returns a coroutine, it is scheduled for asynchronous execution.

        :param msg: Variadic argument representing the payload to forward.
        """
        for (callback, args, explicit_executor) in list(self.callbacks.values()):
            result = callback(*(msg + args))
            if inspect.iscoroutine(result):
                self._schedule_coroutine(result, explicit_executor)

    def unregisterCallback(self, conn: int) -> None:
        """
        Remove a previously registered callback from the filter pipeline.

        If the connection token does not exist, the removal request is silently ignored.

        :param conn: The connection index token returned during registration.
        :type conn: int
        """
        entry = self.callbacks.pop(conn, None)
        if entry is not None:
            callback = entry[0]
            if (
                isinstance(callback, types.MethodType)
                and isinstance(callback.__self__, SimpleFilter)
                and self in callback.__self__.input_filters
            ):
                callback.__self__.input_filters.remove(self)

    def _find_executor_from_upstream(
        self, visited: tp.Optional[tp.Set[int]] = None
    ) -> tp.Optional[rclpy.executors.Executor]:
        """
        Recursively traverse upstream filters to locate an associated rclpy Executor.

        :param visited: Set of already inspected filter object IDs to prevent cycles.
        :return: An rclpy Executor instance if found, None otherwise.
        """
        if visited is None:
            visited = set()
        if id(self) in visited:
            return None
        visited.add(id(self))

        if self.node is not None and self.node.executor is not None:
            return self.node.executor

        for input_filter in self.input_filters:
            executor = input_filter._find_executor_from_upstream(visited)
            if executor is not None:
                return executor

        return None

    def _find_executor(
        self,
        explicit_executor: tp.Optional[rclpy.executors.Executor] = None,
    ) -> tp.Optional[tp.Union[rclpy.executors.Executor, asyncio.AbstractEventLoop]]:
        """
        Locate the executor or event loop for dispatching an async callback.

        :param explicit_executor: Explicit executor passed at registration time.
        :return: An rclpy Executor or asyncio event loop instance, or None if not found.
        """
        if explicit_executor is not None:
            return explicit_executor

        executor = self._find_executor_from_upstream()
        if executor is not None:
            return executor

        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            return None

    def _schedule_coroutine(
        self,
        coroutine: tp.Coroutine[tp.Any, tp.Any, tp.Any],
        explicit_executor: tp.Optional[rclpy.executors.Executor] = None,
    ) -> tp.Union[rclpy.task.Task, asyncio.Task, asyncio.Future]:
        """
        Schedule an asynchronous coroutine for execution on an executor or event loop.

        :param coroutine: The coroutine object returned by the callback invocation.
        :param explicit_executor: Optional explicit executor passed during registration.
        :return: The scheduled Task or Future instance.
        :raises RuntimeError: If no executor or running asyncio loop can be resolved.
        """
        executor = self._find_executor(explicit_executor)
        if executor is not None:
            return executor.create_task(coroutine)

        coroutine.close()
        raise RuntimeError(
            'Cannot schedule async callback: '
            'No rclpy executor or running asyncio event loop found.'
        )
