DeltaFilter (Python):
+====================

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters.DeltaFilter`` class in ROS 2 using Python.
With the ``DeltaFilter`` given a stream of messages, the message is passed down
to the next filter if any of the message fields, that may be acquired
by ``field_getters`` have changed compared to the previously accepted message.

To demonstrate the functionality of the ``DeltaFilter`` filter in a more clear manner, we are going to add a custom filter to this tutorial.
This is going to be the ``CounterFilter`` that will be counting the number of messages passing through it.
This filter class will be a successor to the ``SimpleFilter`` class, but this is a topic for another tutorial.
For more information on this topic, please refer to the `SimpleFilter for Python tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Python.html>`_.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Create a Basic Node
~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for Python.
The next step is to create a new Python file inside your package, e.g., ``delta_tutorial.py``, and write an example code:

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import Subscriber, SimpleFilter
    from message_filters.delta_filter import DeltaFilter
    from std_msgs.msg import String


    TUTORIAL_TOPIC = "/example/topic"
    QOS_PROFILE = QoSProfile(depth=10)
    TIMER_PERIOD_SECONDS = 1.0


    class CounterFilter(SimpleFilter):
        def __init__(self, message_filter: SimpleFilter | None = None):
            SimpleFilter.__init__(self)

            self.incoming_connection = None
            self._counter: int = 0

            if message_filter:
                self.connectInput(message_filter)

        def connectInput(self, message_filter):
            if self.incoming_connection is not None:
                raise RuntimeError('Already connected')
            self.incoming_connection = message_filter.registerCallback(self.add)

        def add(self, message):
            self._counter += 1
            self.signalMessage(message)

        @property
        def counter(self):
            return self._counter


    class DeltaFilterExampleNode(Node):
        def __init__(self):
            super().__init__("delta_filter_node")

            self.subscriber = Subscriber(
                self,
                String,
                TUTORIAL_TOPIC,
                qos_profile=QOS_PROFILE,
            )

            self.delta_filter = DeltaFilter(
                field_getters=[lambda message: message.data],
                message_filter=self.subscriber,
            )

            self.counter_filter = CounterFilter(self.delta_filter)

            self.publisher = self.create_publisher(String, TUTORIAL_TOPIC, QOS_PROFILE)

            self.publisher_timer = self.create_timer(
                timer_period_sec=TIMER_PERIOD_SECONDS,
                callback=self.publisher_callback,
            )

            self.query_timer = self.create_timer(
                timer_period_sec=TIMER_PERIOD_SECONDS,
                callback=self.query_callback,
            )

            self.pub_count = 0

        def publisher_callback(self):
            if self.pub_count % 2 == 0:
                self.publisher.publish(String(data=f"pub_count: {self.pub_count}"))
            else:
                # So that a message content does not change
                self.publisher.publish(String(data=f"pub_count: {self.pub_count - 1}"))
            self.pub_count += 1

        def query_callback(self):
            print(f"Messages passed down the chain: {self.counter_filter.counter}")


    def main() -> None:
        rclpy.init()

        delta_filter_node = DeltaFilterExampleNode()
        rclpy.spin(delta_filter_node)

        delta_filter_node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's break down this code and examine the details.

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import Subscriber, SimpleFilter
    from message_filters.delta_filter import DeltaFilter
    from std_msgs.msg import String


    TUTORIAL_TOPIC = "/example/topic"
    QOS_PROFILE = QoSProfile(depth=10)
    TIMER_PERIOD_SECONDS = 1.0

We start with importing ``rclpy``, ``Node`` and ``QoSProfile`` classes
that are required for constructing node and its subscriptions and publishers,
and for running the node itself.
After that we import message filters: ``Subscriber``, ``SimpleFilter`` and ``DeltaFilter``.
And in the end we import the ``String`` message class that we are going to use for this example.

In the end of this section we define a few constants such as ``TUTORIAL_TOPIC``, ``QOS_PROFILE`` and ``TIMER_PERIOD_SECONDS``.
These will be used by the publisher and the ``Subscriber`` filter for communication.

Next we define a ``CounterFilter`` class.

.. code-block:: python

    class CounterFilter(SimpleFilter):
        def __init__(self, message_filter: SimpleFilter | None = None):
            SimpleFilter.__init__(self)

            self.incoming_connection = None
            self._counter: int = 0

            if message_filter:
                self.connectInput(message_filter)

        def connectInput(self, message_filter):
            if self.incoming_connection is not None:
                raise RuntimeError('Already connected')
            self.incoming_connection = message_filter.registerCallback(self.add)

        def add(self, message):
            self._counter += 1
            self.signalMessage(message)

        @property
        def counter(self):
            return self._counter

This filter counts the number of messages passing through it.
The ``add`` method increases messages count, and passes messages to the following filter via ``signalMessage`` call.
The ``connectInput`` connects this filter to a previous filter's output.
The ``counter`` property grants access to the current messages count.

For more information on this succession mechanism, please refer to the `SimpleFilter for Python tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Python.html>`_.

And now we can turn our attention to the main tutorial class, that is the ``DeltaFilter`` class.
For starters, let's take a look at the ``__init__`` method of this class:

.. code-block:: python

    class DeltaFilterExampleNode(Node):
        def __init__(self):
            super().__init__("delta_filter_node")

            self.subscriber = Subscriber(
                self,
                String,
                TUTORIAL_TOPIC,
                qos_profile=QOS_PROFILE,
            )

            self.delta_filter = DeltaFilter(
                field_getters=[lambda message: message.data],
                message_filter=self.subscriber,
            )

            self.counter_filter = CounterFilter(self.delta_filter)

            self.publisher = self.create_publisher(String, TUTORIAL_TOPIC, QOS_PROFILE)

            self.publisher_timer = self.create_timer(
                timer_period_sec=TIMER_PERIOD_SECONDS,
                callback=self.publisher_callback,
            )

            self.query_timer = self.create_timer(
                timer_period_sec=TIMER_PERIOD_SECONDS,
                callback=self.query_callback,
            )

            self.pub_count = 0

First we create an instance of the ``Subscriber`` filter to serve as an entrypoint for our chain of filters.
Next we create an instance of the ``DeltaFilter`` and connect it to the ``self.subscriber`` filter.
To wrap things up we add an instance of the ``CounterFilter`` to the end of our chain so it can count the messages
that have reached the end of the chain.

Please take a look at the following line of the ``DeltaFilter`` constructor call.

.. code-block:: python

    field_getters=[lambda message: message.data],

The ``DeltaFilter`` accepts a list of callable objects each one of which is expected to return a basic data field
that may be then compared between old and new message.
Each of these ``field_getters`` is applied to a new message and some old message that was the last to pass through this filter.
And if the values differ, new message is accepted, stored as a new "old" message and passed down the chain of filters to the next connected filter.

To put some messages into the chain of filters we need to create ourselves a ``publisher``.
So that is done as well.

And to automate things we create two timers.
The ``publisher_timer`` to automate message publishing.
And the ``query_timer`` to query the number of messages that have reached the ``CounterFilter`` and to print the number to console.

The last thing left to do is to define callbacks that are going to be called by the timers.

.. code-block:: python

        def publisher_callback(self):
            if self.pub_count % 2 == 0:
                self.publisher.publish(String(data=f"pub_count: {self.pub_count}"))
            else:
                # So that a message content does not change
                self.publisher.publish(String(data=f"pub_count: {self.pub_count - 1}"))
            self.pub_count += 1

        def query_callback(self):
            print(f"Messages passed down the chain: {self.counter_filter.counter}")

The ``publisher_callback`` is responsible for publishing messages.
Note that each odd message is published with the data same as the previous even message.
It leads to the odd messages being ignored and even messages being accepted.
The ``query_callback`` just prints the number of messages that have reached the ``counter_filter``.

The ``main`` function as usual in these tutorials is pretty straightforward.

.. code-block:: python

    def main() -> None:
        rclpy.init()

        delta_filter_node = DeltaFilterExampleNode()
        rclpy.spin(delta_filter_node)

        delta_filter_node.destroy_node()
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
            'delta_tutorial = pkg_name.delta_tutorial:main',
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

    ros2 run delta_tutorial delta_tutorial

The output of the node is going to look something like this

.. code-block:: console

    Messages passed down the chain: 0
    Messages passed down the chain: 1
    Messages passed down the chain: 1
    Messages passed down the chain: 2
    Messages passed down the chain: 2
    Messages passed down the chain: 3
    Messages passed down the chain: 3

Note that when the first query is executed, no message have reached the counter filter yet.
So that is indicated by first line of the console output.
At the time the next query is executed, the first message has reached the counter.
So the number of messages is now ``1``.
Then, when the third query is executed, the third message has been published,
but was ignored because the data in it was the same as in previous message.
And so on.

6. Other options provided by the DeltaFilter package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``delta_filter`` provides you with one more filter that may be used right out of the box.
It is the ``PathDeltaFilter``.
To use it you should import it from the package

.. code-block:: python

    from message_filters.delta_filter import PathDeltaFilter

And use it instead of the ``DeltaFilter`` in the ``DeltaFilterExampleNode`` constructor.

.. code-block:: python

            self.delta_filter = PathDeltaFilter(
                field_path_list=["data"],
                message_filter=self.subscriber,
            )

In this case, the message structure is pretty straightforward.
You need to access the ``data`` field and compare it between messages.
But if your message has some messages within messages you just combine the path to the final comparable field,
and separate the field names along this path with dots as such

.. code-block:: python

            self.delta_filter = PathDeltaFilter(
                field_path_list=["message.nested_message.field"],
                message_filter=self.subscriber,
            )

Also, if you wish to make your comparison faster and to compare multiple messages with one field_getter call
you may create your own ``ComparisonHandler``, inheriting from the ``CachedComparisonHandler``
and define your own comparison rules for it.
With such custom ``ComparisonHandler`` you may declare a comparison filter as such

.. code-block:: python

    from message_filters.delta_filter import CachedComparisonHandler, ComparisonFilter

    your_custom_comparison_filter = ComparisonFilter(
        comparison_handler=YourCustomComparisonHandler,
        message_filter=subscriber_filter,   # for example
    )

and you should be good to go.

This ``CustomComparisonHandler`` should implement the ``message_fits`` method that accepts a message as a parameter.
It should return ``True`` if the message fits your custom condition and ``False`` if it does not.