InputAligner (Python):
----------------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters.InputAligner`` filter in ROS 2 using Python.

Unlike synchronizers (such as ``TimeSynchronizer`` or ``ApproximateTimeSynchronizer``) that combine multiple input
messages into a single composite callback, ``InputAligner`` accepts ``N`` input message streams and forwards
them to ``N`` separate callbacks in strictly chronological order according to their header timestamps.

In sensor fusion and perception architectures (such as combining IMUs, LiDARs, cameras, and odometry), sensors
publish at different rates and suffer varying network or processing delays. When messages arrive out of order,
an ``InputAligner`` buffers and orders them before downstream processing:

- **Separate Outputs**: Each input stream maintains its own independent callback function.
- **Period Hints**: Specifying expected message periods via ``setInputPeriod()`` allows optimistic lookahead, reducing latency.
- **Timeout Protection**: If an input stops publishing, it times out after a configurable duration so other active streams are not blocked indefinitely.
- **Flexible Dispatching**: Messages can be automatically flushed via an integrated ROS 2 timer with ``setupDispatchTimer()`` or manually triggered via ``dispatchMessages()``.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already, `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Create a Basic Node
~~~~~~~~~~~~~~~~~~~~~~

Let's assume you've already created an empty ROS 2 package for Python.
Create a new Python file inside your package directory, e.g., ``input_aligner_tutorial.py``:

.. code-block:: python

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from geometry_msgs.msg import PointStamped
    from message_filters import Subscriber
    from message_filters.input_aligner import InputAligner


    class InputAlignerExampleNode(Node):

        def __init__(self):
            super().__init__('input_aligner_example_node')
            qos = QoSProfile(depth=10)

            # Publishers for two simulated sensor streams
            self.pub_fast = self.create_publisher(PointMsg := PointStamped, 'sensor_fast', qos)
            self.pub_slow = self.create_publisher(PointMsg, 'sensor_slow', qos)

            # Subscriptions wrapped with message_filters
            self.sub_fast = Subscriber(self, PointStamped, 'sensor_fast', qos_profile=qos)
            self.sub_slow = Subscriber(self, PointStamped, 'sensor_slow', qos_profile=qos)

            # Initialize InputAligner with a 1.0 second timeout and the input filters
            self.aligner = InputAligner(
                timeout=Duration(seconds=1.0),
                filters=[self.sub_fast, self.sub_slow],
            )

            # Register separate callbacks for each input channel (channel index 0 and 1)
            self.aligner.registerCallback(0, self.fast_sensor_callback)
            self.aligner.registerCallback(1, self.slow_sensor_callback)

            # Provide period hints (fast: 50ms, slow: 100ms) to minimize buffering delay
            self.aligner.setInputPeriod(0, Duration(nanoseconds=int(50e6)))
            self.aligner.setInputPeriod(1, Duration(nanoseconds=int(100e6)))

            # Setup dispatch timer to periodically evaluate and flush aligned messages
            self.aligner.setupDispatchTimer(self, Duration(nanoseconds=int(10e6)))

            self.count_fast = 0.0
            self.count_slow = 0.0

            # Timers simulating sensor data generation
            self.timer_fast = self.create_timer(0.05, self.publish_fast_sensor)
            self.timer_slow = self.create_timer(0.10, self.publish_slow_sensor)

        def fast_sensor_callback(self, msg: PointStamped):
            stamp = msg.header.stamp
            self.get_logger().info(
                f'[Channel 0 - Fast] Processed msg seq={msg.point.x:.0f} at stamp={stamp.sec}.{stamp.nanosec:09d}'
            )

        def slow_sensor_callback(self, msg: PointStamped):
            stamp = msg.header.stamp
            self.get_logger().info(
                f'[Channel 1 - Slow] Processed msg seq={msg.point.x:.0f} at stamp={stamp.sec}.{stamp.nanosec:09d}'
            )

        def publish_fast_sensor(self):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sensor_fast_frame'
            self.count_fast += 1.0
            msg.point.x = self.count_fast
            self.pub_fast.publish(msg)

        def publish_slow_sensor(self):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'sensor_slow_frame'
            self.count_slow += 1.0
            msg.point.x = self.count_slow
            self.pub_slow.publish(msg)


    def main(args=None):
        rclpy.init(args=args)
        node = InputAlignerExampleNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Let's break down how this works:

.. code-block:: python

    from message_filters import Subscriber
    from message_filters.input_aligner import InputAligner

We import ``Subscriber`` and ``InputAligner`` from the ``message_filters`` package.
The input messages must contain a standard header with timestamp information (``msg.header.stamp``).

.. code-block:: python

    self.aligner = InputAligner(
        timeout=Duration(seconds=1.0),
        filters=[self.sub_fast, self.sub_slow],
    )

We instantiate the aligner by specifying the timeout duration and the list of input filters.
Alternatively, you can create an aligner and connect upstream filters later using ``self.aligner.connectInput([sub1, sub2])``.

.. code-block:: python

    self.aligner.registerCallback(0, self.fast_sensor_callback)
    self.aligner.registerCallback(1, self.slow_sensor_callback)

Callbacks are registered per channel index. Channel ``0`` receives forwarded messages from ``sub_fast``,
and channel ``1`` receives forwarded messages from ``sub_slow``.

.. code-block:: python

    self.aligner.setInputPeriod(0, Duration(nanoseconds=int(50e6)))
    self.aligner.setInputPeriod(1, Duration(nanoseconds=int(100e6)))

Setting the expected period provides a lookahead hint. When the aligner knows the next message on a channel
will not arrive before a given time, it can immediately dispatch pending messages from other streams that have
earlier timestamps.

.. code-block:: python

    self.aligner.setupDispatchTimer(self, Duration(nanoseconds=int(10e6)))

Calling ``setupDispatchTimer()`` creates a ROS 2 timer on the node that triggers ``aligner.dispatchMessages()``
at the specified interval (e.g., every 10 ms). You can also invoke ``aligner.dispatchMessages()`` manually
within your own processing loop.

2. Monitoring Queue Status
~~~~~~~~~~~~~~~~~~~~~~~~~~

The aligner allows querying the queue health for any channel:

.. code-block:: python

    status = self.aligner.getQueueStatus(0)
    self.get_logger().info(
        f"Active: {status.active}, Queued: {status.queue_size}, "
        f"Processed: {status.msgs_processed}, Dropped: {status.msgs_dropped}"
    )

- ``active``: ``True`` if the channel is currently receiving messages within the timeout threshold; ``False`` if timed out.
- ``queue_size``: Number of messages currently buffered for this channel.
- ``msgs_processed``: Total count of messages dispatched successfully.
- ``msgs_dropped``: Count of messages dropped (e.g. if a message arrives with a timestamp older than the last dispatched output).

3. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Open ``package.xml`` in your package root and declare the dependencies:

.. code-block:: xml

    <depend>geometry_msgs</depend>
    <depend>message_filters</depend>
    <depend>rclpy</depend>

4. Update setup.py
~~~~~~~~~~~~~~~~~~

In ``setup.py``, add the node executable to the ``entry_points`` dictionary:

.. code-block:: python

    entry_points={
        'console_scripts': [
            'input_aligner_tutorial = <your_package_name>.input_aligner_tutorial:main',
        ],
    },

5. Build Your Package
~~~~~~~~~~~~~~~~~~~~~

From the root of your workspace:

**Linux and macOS**

.. code-block:: console

    $ colcon build --packages-select <your_package_name> && . install/setup.bash

**Windows**

.. code-block:: console

    $ colcon build --packages-select <your_package_name>
    $ call install\setup.bat

6. Run the Node
~~~~~~~~~~~~~~~

Run the executable:

.. code-block:: console

    ros2 run <your_package_name> input_aligner_tutorial

You will see the messages from both fast and slow channels processed in strictly increasing chronological order:

.. code-block:: console

    [INFO] [1773695000.052100000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=1 at stamp=1773695000.050000000
    [INFO] [1773695000.102300000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=2 at stamp=1773695000.100000000
    [INFO] [1773695000.102500000] [input_aligner_example_node]: [Channel 1 - Slow] Processed msg seq=1 at stamp=1773695000.100000000
    [INFO] [1773695000.152400000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=3 at stamp=1773695000.150000000
    [INFO] [1773695000.202100000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=4 at stamp=1773695000.200000000
    [INFO] [1773695000.202400000] [input_aligner_example_node]: [Channel 1 - Slow] Processed msg seq=2 at stamp=1773695000.200000000
