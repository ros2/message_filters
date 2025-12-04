SimpleFilter (Python):
----------------------

Overview
~~~~~~~~

This tutorial demonstrates how to create a custom filter that is going to be a successor to the ``SimpleFilter`` class.
The ``SimpleFilter`` is a base class for almost all of message filters implemented in ``Python``.
It provides the basic functionality for building filters.
To demonstrate the functionality of this filter we are going to create a ``CounterWithCallback`` filter class.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node
~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for Python.
The next step is to create a new Python file inside your package, e.g. ``simple_filter_tutorial.py``, and write an example code:

.. code-block:: python

    import typing as tp

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import SimpleFilter, Subscriber
    from std_msgs.msg import String
    from rclpy.type_support import MsgT


    TUTORIAL_TOPIC = "/example/topic"
    QOS_PROFILE = QoSProfile(depth=10)
    TIMER_PERIOD_SECONDS = 1


    class CounterWithCallback(SimpleFilter):

        def __init__(
            self,
            message_filter = None,
        ):
            super().__init__()

            self.count = 0

            self.incoming_connection = None

            self.connectInput(message_filter=message_filter)
            self.registerCallback(self.counter_callback)

        def connectInput(self, message_filter):
            if self.incoming_connection is not None:
                raise RuntimeError('Already connected')
            self.incoming_connection = message_filter.registerCallback(self.add)

        def registerCallback(self, callback, *args):
            super().registerCallback(callback, *args)

        def add(self, message: MsgT):
            # Some other work may be done here with a message
            super().signalMessage(message)

        def counter_callback(self, _: MsgT):
            self.count += 1

        def print_counter_value(self):
            print(f"Filter counter value: {self.count}")


    class LastMessageCache:
        def __init__(self):
            self.last_message = None

        def new_message_callback(self, message: MsgT):
            self.last_message = message

        def print_last_msg_data(self):
            if self.last_message:
                print(f"Last cached message: {self.last_message.data}")
            else:
                print(f"Cache is empty")


    class SimpleFilterExampleNode(Node):

        def __init__(self):
            super().__init__("simple_filter_example_node")

            self.pub_count = 0
            self.publisher = self.create_publisher(
                String,
                TUTORIAL_TOPIC,
                QOS_PROFILE,
            )

            self.subscriber_filter = Subscriber(
                node=self,
                msg_type=String,
                topic=TUTORIAL_TOPIC,
                qos_profile=QOS_PROFILE,
            )

            self.cache = LastMessageCache()

            self.filter = CounterWithCallback(
                message_filter=self.subscriber_filter,
            )
            self.filter.registerCallback(
                self.cache.new_message_callback
            )

            self.publisher_timer = self.create_timer(
                TIMER_PERIOD_SECONDS,
                self.publisher_timer_callback,
            )

            self.query_timer = self.create_timer(
                TIMER_PERIOD_SECONDS,
                self.counter_query_timer_callback,
            )

        def publisher_timer_callback(self):
            msg = String()
            msg.data = f"Example string data № {self.pub_count}"
            self.publisher.publish(msg)
            self.pub_count += 1

        def counter_query_timer_callback(self):
            self.filter.print_counter_value()
            self.cache.print_last_msg_data()
            print("")


    def main():
        rclpy.init()

        example_node = SimpleFilterExampleNode()
        rclpy.spin(example_node)

        example_node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's break down this code and examine the details.

.. code-block:: python

    import typing as tp

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import SimpleFilter, Subscriber
    from std_msgs.msg import String
    from rclpy.type_support import MsgT


    TUTORIAL_TOPIC = "/example/topic"
    QOS_PROFILE = QoSProfile(depth=10)
    TIMER_PERIOD_SECONDS = 1

We start with importing ``rclpy``, ``Node`` and ``QoSProfile`` classes
that are required for constructing node and it's subscriptions and publishers,
and for running the node itself.
After that we import ``SimpleFilter`` and ``Subscriber`` filter classes.
The first one will serve as a base class for the example filter.
The second will serve as an entry point for messages from an example topic.
Next the ``String`` message class is imported.
We are giong to create a publisher and subscriber for this message type.
And finally the message type ``MsgT`` that will be used for type hints.
In the end of this section the ``TUTORIAL_TOPIC``, ``QOS_PROFILE`` and ``TIMER_PERIOD_SECONDS`` constants are declared.
These will be used for configuring publisher and subscriber as well as for publisher automation later on.

Next let's take a look at the ``CounterWithCallback`` class.
It implements a synthetic scenario to demonstrate all the main features of the ``SimpleFilter`` class in a clearer manner.

.. code-block:: python

        class CounterWithCallback(SimpleFilter):

        def __init__(
            self,
            message_filter = None,
        ):
            super().__init__()

            self.count = 0

            self.incoming_connection = None

            self.connectInput(message_filter=message_filter)
            self.registerCallback(self.counter_callback)

        def connectInput(self, message_filter):
            if self.incoming_connection is not None:
                raise RuntimeError('Already connected')
            self.incoming_connection = message_filter.registerCallback(self.add)

        def registerCallback(self, callback, *args):
            super().registerCallback(callback, *args)

        def add(self, message: MsgT):
            # Some other work may be done here with message
            super().signalMessage(message)

        def counter_callback(self, _: MsgT):
            self.count += 1

        def print_counter_value(self):
            print(f"Filter counter value: {self.count}")

This filter accepts other filter as an argument of the ``__init__`` method.
If any is provided, this filter registers it's own ``add`` method as a callback with that other filter.
Thus, when a message arrives to this other filter and that filter calls the ``signalMessage``,
a message is passed to this ``CounterWithCallback`` filter's ``add`` method and from there, down the chain of filters.
The ``signalMessage`` method is of the base ``SimpleFilter`` class.

Every instance of the ``SimpleFilter`` class may have a collection of callbacks.
A callback may be added to this collection via ``registerCallback`` call.

.. code-block:: python

    def callback(message: MsgT):
        # Some work done here

    ...

    def callback_with_args(message: MsgT, *args):
        # Some work done here

    ...

    # register callback *without* arguments
    simple_filter.registerCallback(callback)

    # register callback *with* arguments
    simple_filter.registerCallback(callback_with_args, arguments_list)

When the ``signalMessage`` method of a ``SimpleFilter`` class is called,
all callbacks from this collection are executed with the corresponding arguments if any.

So it is important to note that for your filter to be able pass messages to other filters,
it has to call the ``signalMessage`` method at some point in it's workflow.

The example ``CounterWithCallback`` class, as the name suggests counts the messages coming through.
Also it gives an option to register a callback to be called for every message.
To be as precise as possible, evetry subclass of the ``SimpleFilter`` class provides this option expressly prohibited.
To demonstrate this, there is the ``registerCallback`` method in the class interface,
that does nothing except calling ``super().registerCallback``.

Next up is the ``LastMessageCache`` class.

.. code-block:: python

    class LastMessageCache:
        def __init__(self):
            self.last_message = None

        def new_message_callback(self, message: MsgT):
            self.last_message = message

        def print_last_msg_data(self):
            if self.last_message:
                print(f"Last cached message: {self.last_message.data}")
            else:
                print(f"Cache is empty")

This class, as the name suggests, stores the last message, and provides a callback for the ``CounterWithCallback`` class.

What is left to discuss is the ``SimpleFilterExampleNode`` that does all the management of classes defined earlier.

.. code-block:: python

    class SimpleFilterExampleNode(Node):

        def __init__(self):
            super().__init__("simple_filter_example_node")

            self.pub_count = 0
            self.publisher = self.create_publisher(
                String,
                TUTORIAL_TOPIC,
                QOS_PROFILE,
            )

First of all we initialize a basic ``ros2`` ``publisher`` that will generate the input for our filters. 

.. code-block:: python

            self.subscriber_filter = Subscriber(
                node=self,
                msg_type=String,
                topic=TUTORIAL_TOPIC,
                qos_profile=QOS_PROFILE,
            )

Next the ``Subscriber`` filter for the messages coming from publisher to enter a chain of filters.

.. code-block:: python

            self.cache = LastMessageCache()

            self.filter = CounterWithCallback(
                message_filter=self.subscriber_filter,
            )
            self.filter.registerCallback(
                self.cache.new_message_callback
            )

After the ``publisher`` and ``Subscruber`` are set up, we initialize ``LastMessageCache`` and ``CounterWithCallback`` filter.
We register the ``new_message_callback`` as a callback with the ``registerCallback`` call.

.. code-block:: python

            self.publisher_timer = self.create_timer(
                TIMER_PERIOD_SECONDS,
                self.publisher_timer_callback,
            )

            self.query_timer = self.create_timer(
                TIMER_PERIOD_SECONDS,
                self.counter_query_timer_callback,
            )

        def publisher_timer_callback(self):
            msg = String()
            msg.data = f"Example string data № {self.pub_count}"
            self.publisher.publish(msg)
            self.pub_count += 1

        def counter_query_timer_callback(self):
            self.filter.print_counter_value()
            self.cache.print_last_msg_data()
            print("")

What is left to do is to create timers and timer callbacks.

The ``main`` function as usual in this tutorials is pretty straightforward.

.. code-block:: python

    def main():
        rclpy.init()

        example_node = SimpleFilterExampleNode()
        rclpy.spin(example_node)

        example_node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

2. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>message_filters</depend>
    <depend>rclpy</depend>
    <depend>std_msgs</depend>

3. Add Entry Point in setup.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Update the ``console_scripts`` section of your ``setup.py``:

.. code-block:: python

    entry_points={
        'console_scripts': [
            'simple_filter_tutorial = pkg_name.simple_filter_tutorial:main',
        ],
    },

Replace ``pkg_name`` with your actual package name.

4. Build Your Package
~~~~~~~~~~~~~~~~~~~~~

From the root of your workspace:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            $ colcon build && . install/setup.bash

    .. group-tab:: macOS

        .. code-block:: console

            $ colcon build && . install/setup.bash

    .. group-tab:: Windows

        .. code-block:: console

            $ colcon build
            $ call C:\dev\ros2\local_setup.bat

5. Run the Node
~~~~~~~~~~~~~~~

Now run the node using:

.. code-block:: console

    ros2 run pkg_name simple_filter_tutorial

The output of the node is going to look something like this

.. code-block:: console

    Filter counter value: 0
    Cache is empty

    Filter counter value: 1
    Last cached message: Example string data № 0

    Filter counter value: 2
    Last cached message: Example string data № 1

    Filter counter value: 3
    Last cached message: Example string data № 2

    Filter counter value: 4
    Last cached message: Example string data № 3

    Filter counter value: 5
    Last cached message: Example string data № 4

    Filter counter value: 6
    Last cached message: Example string data № 5

Note that when the first query to the filter and the ``LastMessageCache`` is executed,
they both report that there was no messages, passing through before.
The following queries show that the count of messages passed through the filter is increasing
and that the last cached message is constantly updated.
That means that both ``CounterWithCallback.counter_callback``
and ``LastMessageCache.new_message_callback`` are executed for every new message.
As designed.