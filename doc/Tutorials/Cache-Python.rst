Cache (Python):
---------------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters.Cache`` class in ROS 2 using Python.
The ``Cache`` filter stores a time history of messages and allows querying based on timestamps.

We will use ``std_msgs.msg.String`` message for clarity and simplicity.

Prerequisites
~~~~~~~~~~~~~
This tutorial assumes you have a working knowledge of ROS 2

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create an Example Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Let's assume, you've already created an empty ros package for Python.
The next step is to create a new Python file inside your package, e.g., ``cache_tutorial.py``, and create an example node:

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from message_filters import Subscriber, Cache
    from std_msgs.msg import String


    class CacheNode(Node):

        def __init__(self):
            super().__init__('cache_node')

            qos = QoSProfile(depth=10)

            self.publisher = self.create_publisher(String, 'input', qos_profile)
            self.subscriber = Subscriber(
                self,
                String,
                "/example/topic",
                qos_profile=qos_profile,
            )
            self.cache = Cache(
                self.subscriber,
                cache_size=5,
                allow_headerless=True,  # To allow caching basic String message
            )

            # Simulate publishing via timer
            self.publisher_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.publisher_timer_callback,
            )

            # Check on cached data
            self.query_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.query_timer_callback,
            )

            self.counter = 0

        def publisher_timer_callback(self):
            self.publisher.publish(
                String(
                    data=f"Message {self.counter}"
                )
            )
            self.counter += 1

        def query_timer_callback(self):
            latest_time = self.cache.getLatestTime()
            if latest_time is None:
                self.get_logger().info("Cache is empty.")
                return

            oldest_time = self.cache.getOldestTime()

            self.get_logger().info(f"oldest_time: {oldest_time.seconds_nanoseconds()[0]},"
                                   f"latest_time: {latest_time.seconds_nanoseconds()[0]}")

            cached_messages = self.cache.getInterval(oldest_time, latest_time)

            for msg in cached_messages:
                self.get_logger().info(f"Cached: {msg.data}")


    def main(args=None):
        rclpy.init(args=args)

        cache_node = CacheNode()
        rclpy.spin(cache_node)

        cache_node.destroy_node()
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

    from message_filters import Subscriber, Cache
    from std_msgs.msg import String

We start with importing ``rclpy``, ``Node`` and ``QoSProfile`` classes
that are required for constructing node and it's subscriptions and publishers,
and for running the node itself. After that we import message filters:
``Subscriber`` and ``Cache``. And in the end we import the ``String``
message class that we are going to use for this example.

.. code-block:: python

    class CacheNode(Node):

        def __init__(self):
            super().__init__('cache_node')

            qos = QoSProfile(depth=10)

            self.publisher = self.create_publisher(String, 'input', qos_profile)

After declaring imports, we create a class for this example, declare a Quality of Service profile
that we are going to use for all our interfaces in this example, and create a publisher
that is going to populate the example topic with messages to cache.

.. code-block:: python

            self.subscriber = Subscriber(
                self,
                String,
                "/example/topic",
                qos_profile=qos_profile,
            )
            self.cache = Cache(
                self.subscriber,
                cache_size=5,
                allow_headerless=True,  # To allow caching basic String message
            )

The next step is to create filters and to chain them together.
We start with a Subscriber filter, that is going to be and entry point for
the messages into our chain of filters. And after that we create a
cache filter object, that is going to cache the messages, passing down
the filters chain. Please note, that when the ``cache`` is created,
the previous filter, the ``subscriber`` is passed as the first argument.
It is the way to chain these two filters together. Message is going to pass through
``subscriber`` into ``cache``, and in some other filter if it is added
down the chain.

It may be useful to point out that the ``Subscriber`` filter is not the only
way to start a chain of filters. One may consider using ``SimpleFilter``. 
It does not create a new subscription on it's own and may be used directly
in a subscription callback instead.

In this case, we set the argument ``allow_headerless`` value to ``true``, to allow caching
``std_msgs/String`` message, as it does not have a ``Header``. In case we've decided
to set this value to ``False``, the filter would log a corresponding error message,
when trying to store message in cache.

What is left to be done is to set timers

.. code-block:: python

            # Simulate publishing via timer
            self.publisher_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.publisher_timer_callback,
            )

            # Check on cached data
            self.query_timer = self.create_timer(
                timer_period_sec=1.0,
                callback=self.query_timer_callback,
            )

And define the timer callbacks

.. code-block:: python

        def publisher_timer_callback(self):
            self.publisher.publish(
                String(
                    data=f"Message {self.counter}"
                )
            )
            self.counter += 1

        def query_timer_callback(self):
            latest_time = self.cache.getLatestTime()
            if latest_time is None:
                self.get_logger().info("Cache is empty.")
                return

            oldest_time = self.cache.getOldestTime()

            self.get_logger().info(f"oldest_time: {oldest_time.seconds_nanoseconds()[0]},"
                                   f"latest_time: {latest_time.seconds_nanoseconds()[0]}")

            cached_messages = self.cache.getInterval(oldest_time, latest_time)

            for msg in cached_messages:
                self.get_logger().info(f"Cached: {msg.data}")



2. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>rclpy</depend>
    <depend>message_filters</depend>
    <depend>std_msgs</depend>

3. Add Entry Point in setup.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Update the ``console_scripts`` section of your ``setup.py``:

.. code-block:: python

    entry_points={
        'console_scripts': [
            'cache_tutorial = pkg_name.cache_tutorial:main',
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

.. code-block:: bash

    ros2 run pkg_name cache_tutorial

The first message in the output is going to be

.. code-block:: bash

	[INFO] [1750884527.235426721] [cache_node]: Cache filters cache is empty
	
As there were no messages published yet, and the cache is empty.
After that, the publisher will start populate the cache with messages:

.. code-block:: bash

	[INFO] [1750887122.590581767] [cache_node]: oldest_time: 1750887121, latest_time: 1750887121
	[INFO] [1750887123.593117081] [cache_node]: oldest_time: 1750887121, latest_time: 1750887122
	[INFO] [1750887124.593130934] [cache_node]: oldest_time: 1750887121, latest_time: 1750887123
	[INFO] [1750887125.592839265] [cache_node]: oldest_time: 1750887121, latest_time: 1750887124
	[INFO] [1750887126.592716962] [cache_node]: oldest_time: 1750887121, latest_time: 1750887125
	[INFO] [1750887127.592824186] [cache_node]: oldest_time: 1750887122, latest_time: 1750887126  <-- drop old msgs
	[INFO] [1750887128.590810767] [cache_node]: oldest_time: 1750887123, latest_time: 1750887127


Note as the oldest time is starting to update after the 5'th message is added to the cache.
The cache size for the ``cache`` in this example is 5. So as the 5'th message is added to
the cache, the oldest messages are being removed from it, thus updating oldest time.

6. Other methods of the Cache filter interface 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``Cache`` filter stores the last N messages (in this case, 5), and allows querying:

- Entire history: ``getInterval(start_time, end_time)``
- Most recent message: ``getLast()``
- Oldest timestamp: ``getOldestTime()``
- Newest timestamp: ``getLatestTime()``
- Messages after a certain time: ``getElemAfterTime(time)``
- Messages before a certain time: ``getElemBeforeTime(time)``

This is especially useful when you need to look back in time (e.g., align with previous sensor data).
