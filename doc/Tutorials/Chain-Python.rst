Chain (Python):
---------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters.Chain`` class in ROS 2 using Python.
The ``Chain`` filter provides a container for simple filters.
It allows you to store an N-long set of filters inside a single structure, making it much easier to manage them.

To demonstrate the functionality of the ``Chain`` filter in a more clear manner, we are going to add a custom filter to this tutorial.
This is going to be the ``CounterFilter`` that will be counting the number of messages passing through it.
This filter class will be a successor to the ``SimpleFilter`` class, but this is a topic for another tutorial.
For more information on this topic, please refer to the `SimpleFilter for Python tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Python.html>`.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node
~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for Python.
The next step is to create a new Python file inside your package, e.g., ``chain_tutorial.py``, and write an example code:

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import Subscriber, Chain, SimpleFilter
    from std_msgs.msg import String


    TUTORIAL_TOPIC = "/example/topic"


    class CounterFilter(SimpleFilter):
        def __init__(self):
            SimpleFilter.__init__(self)

            self.incoming_connection = None
            self._counter: int = 0

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


    class ChainNode(Node):

        def __init__(self):
            super().__init__('chain_node')

            qos_profile = QoSProfile(depth=10)

            self.publisher = self.create_publisher(String, TUTORIAL_TOPIC, qos_profile)
            self.subscriber = Subscriber(
                self,
                String,
                TUTORIAL_TOPIC,
                qos_profile=qos_profile,
            )
            self.first_counter = CounterFilter()
            self.second_counter = CounterFilter()
            self.chain_counter = 0

            self.chain_filter = Chain(self.subscriber)
            self.chain_filter.addFilter(self.first_counter)
            self.chain_filter.addFilter(self.second_counter)
            self.chain_filter.registerCallback(self.chain_callback)

            self.publisher_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.publisher_timer_callback,
            )

            self.query_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.querty_timer_callback,
            )

        def chain_callback(self, message):
            self.chain_counter += 1

        def publisher_timer_callback(self):
            self.publisher.publish(String(data='example message'))

        def querty_timer_callback(self):
            first_filter_count = self.chain_filter.getFilter(0).counter
            second_filter_count = self.chain_filter.getFilter(1).counter

            print(f"first counter messages count: {first_filter_count}, second counter messages count: {second_filter_count}")
            print(f"messages reached the end of chain: {self.chain_counter}\n")


    def main():
        rclpy.init()

        chain_node = ChainNode()
        rclpy.spin(chain_node)

        chain_node.destroy_node()
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

    from message_filters import Subscriber, Chain, SimpleFilter
    from std_msgs.msg import String


    TUTORIAL_TOPIC = "/example/topic"

We start with importing ``rclpy``, ``Node`` and ``QoSProfile`` classes
that are required for constructing node and it's subscriptions and publishers,
and for running the node itself. After that we import message filters:
``Subscriber``, ``Chain`` and ``SimpleFilter``.
And in the end we import the ``String`` message class that we are going to use for this example.
In the end of this section we define a ``TUTORIAL_TOPIC`` constant.
It will be convenient when defining publishers and subscribers later in this tutorial.
Next we define a ``CounterFilter`` class.

.. code-block:: python

    class CounterFilter(SimpleFilter):
        def __init__(self):
            SimpleFilter.__init__(self)

            self.incoming_connection = None
            self._counter: int = 0

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

For more information on this succession mechanism, please refer to the `SimpleFilter for Python tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Python.html>`.

And now we can turn our attention to the main tutorial class, that is the ``ChainNode`` class.
For starters, let's take a look at the ``__init__`` method of this class:

.. code-block:: python

    class ChainNode(Node):

        def __init__(self):
            super().__init__('chain_node')

            qos_profile = QoSProfile(depth=10)

            self.publisher = self.create_publisher(String, TUTORIAL_TOPIC, qos_profile)
            self.subscriber = Subscriber(
                self,
                String,
                TUTORIAL_TOPIC,
                qos_profile=qos_profile,
            )
            self.first_counter = CounterFilter()
            self.second_counter = CounterFilter()
            self.chain_counter = 0

First we declare a Quality of Service profile.
After that we initialize a basic ``ros2`` ``publisher`` that will generate the input for our filters chain. 
The chain is giong to contain three filters.
A ``Subscriber`` filter and two instances of a ``CounterFilter`` that is defined earlier.
We initialize all of those.
In the end of this section we create the ``chain_counter`` field that is going to count all the messages that have passed through all the filters in the ``Chain``.
When a message passes through all the filters in the chain, it is passed to a ``Chain`` filter's callback.
This callback is where ``chain_counter`` is updated.

.. code-block:: python

            self.chain_filter = Chain(self.subscriber)
            self.chain_filter.addFilter(self.first_counter)
            self.chain_filter.addFilter(self.second_counter)
            self.chain_filter.registerCallback(self.chain_callback)

The next step is to build the ``Chain`` message filter and populate it with other filters.
We initialize the ``Chain`` filter itself and pass the ``Subscriber`` filter as an argument to the constructor function to create an entry point for messages into the chain.
At this moment the ``Chain`` filter holds only one filter - the ``Subscriber`` filter.
We add two instances of ``CounterFilter`` to the chain. wia ``addFilter`` method call.
Filnally we register the ``chain_callback`` method of our ``ChainNode`` class as a final callback for the ``Chain`` filter object.

Now, when all filters are set up, we need to publish some messages to the example topic.
For this purpose we set up a publisher and a publish timer, and a query timer to see the results.

.. code-block:: python

            self.publisher_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.publisher_timer_callback,
            )

            self.query_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.querty_timer_callback,
            )

And define the timer callbacks and the ``chain_callback``.

.. code-block:: python

        def chain_callback(self, message):
            self.chain_counter += 1

        def publisher_timer_callback(self):
            self.publisher.publish(String(data='example message'))

        def querty_timer_callback(self):
            first_filter_count = self.chain_filter.getFilter(0).counter
            second_filter_count = self.chain_filter.getFilter(1).counter

            print(f"first counter messages count: {first_filter_count}, second counter messages count: {second_filter_count}")
            print(f"messages reached the end of chain: {self.chain_counter}\n")

Please notice the ``query_timer_callback``.
It demonstrates one of the ``Chain`` filter main methods.
The ``getFilter`` method provides an access to any filter in the chain, given it's position.
The return value of this method is a shared pointer to the required filter.

The ``main`` function as usual in this tutorials is pretty straightforward.

.. code-block:: python

    def main():
        rclpy.init()

        chain_node = ChainNode()
        rclpy.spin(chain_node)

        chain_node.destroy_node()
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
            'chain_tutorial = pkg_name.chain_tutorial:main',
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

    ros2 run pkg_name chain_tutorial

The output of the node is going to look something like this

.. code-block:: console

    first counter messages count: 0, second counter messages count: 0
    messages reached the end of chain: 0

    first counter messages count: 1, second counter messages count: 1
    messages reached the end of chain: 1

    first counter messages count: 2, second counter messages count: 2
    messages reached the end of chain: 2

    first counter messages count: 3, second counter messages count: 3
    messages reached the end of chain: 3

    first counter messages count: 4, second counter messages count: 4
    messages reached the end of chain: 4

Note that when the first query to the both counter filters executed, they both report that there was no messages, passing through before.
And there were no messages that have reached the end of the chain.
After that the first message passes through the all filters in chain as indicated by increased counter values

.. code-block:: console

    first counter messages count: 1, second counter messages count: 1
    messages reached the end of chain: 1

From this point on, all three counters increase their values as more messages are passed down the filter chain.

6. Other methods of the Chain filter interface 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this example we've passed the ``subscriber_filter`` object to the ``chain_filter`` as a constructor argument.
In this case, the ``subscriber_filter`` was used as an input filter for the ``chain_filter``.

.. code-block:: python

    self.chain_filter = Chain(self.subscriber)

The other way to do it is by calling ``connectInput`` method of the ``Chain`` class.

.. code-block:: python
    
    self.chain_filter = Chain()
    self.subscriber = Subscriber()
    self.chain_filter.connectInput(self.subscriber)
