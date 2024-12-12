Approximate Time Synchronizer (Python):
---------------------------------------

Prerequisites
~~~~~~~~~~~~~
This tutorial assumes you have a working knowledge of ROS 2

If you have not done so already `create a workspace <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: Python

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.clock import Clock

    from message_filters import Subscriber, ApproximateTimeSynchronizer
    from sensor_msgs.msg import Temperature, FluidPressure


    class TimeSyncNode(Node):

        def __init__(self):
            super().__init__('sync_node')
            qos = QoSProfile(depth=10)
            self.temp_pub = self.create_publisher(Temperature, 'temp', qos)
            self.fluid_pub = self.create_publisher(FluidPressure, 'fluid', qos)
            self.temp_sub = Subscriber(self, Temperature, "temp")
            self.fluid_sub = Subscriber(self, FluidPressure, "fluid")

            self.timer = self.create_timer(1, self.TimerCallback)
            self.second_timer = self.create_timer(1.05, self.SecondTimerCallback)

            queue_size = 10
            max_delay = 0.05
            self.time_sync = ApproximateTimeSynchronizer([self.temp_sub, self.fluid_sub],
                                                         queue_size, max_delay)
            self.time_sync.registerCallback(self.SyncCallback)


For this example we will be using the ``temperature`` and ``fluid_pressure`` messages found in
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs/msg>`_.
To simulate a working ``ApproximateTimeSynchronizer``. We will be publishing and subscribing to topics of those respective types, to showcase how real sensors would be working. To simulate them we will also need two ``Timers`` on different intervals. Then, we will be utilizing an ``ApproximateTimeSynchronizer`` to get these messages from the sensor topics aligned with a slight delay between messages..
It is essential that the QoS is the same for all of the publishers and subscribers, otherwise the Message Filter cannot align the topics together. So, create one ``QoSProfile`` and stick with it, or find out what ``qos`` is being used in the native sensor code, and replicate it. Do basic construction of each object relating to the ``Node`` and callback methods that may be used in the future. Both of the two timers we utilize will have different timer values of ``1`` and ``1.05`` which causes the timers to off at different points, which is an advantage of using ``ApproximateTime``. Notice that we must call ``sync->registerCallback`` to sync up the two (or more) chosen topics.

So, we must create three (or more) private callbacks, one for the ``ApproximateTimeSynchronizer``, then two for our ``Timers`` which are each for a certain ``sensor_msg``.

.. code-block:: Python

    def SyncCallback(self, temp, fluid):
        temp_sec = temp.header.stamp.sec
        fluid_sec = fluid.header.stamp.sec
        self.get_logger().info(f'Sync callback with {temp_sec} and {fluid_sec} as times')
        if (temp.header.stamp.sec > 2.0):
            new_fluid = FluidPressure()
            new_fluid.header.stamp = Clock().now().to_msg()
            new_fluid.header.frame_id = 'test'
            new_fluid.fluid_pressure = 2.5
            self.fluid_pub.publish(new_fluid)

    def TimerCallback(self):
        temp = Temperature()
        self.now = Clock().now().to_msg()

        temp.header.stamp = self.now
        temp.header.frame_id = 'test'
        temp.temperature = 1.0
        self.temp_pub.publish(temp)

    def SecondTimerCallback(self):
        fluid = FluidPressure()
        self.now = Clock().now().to_msg()

        fluid.header.stamp = self.now
        fluid.header.frame_id = "test"
        fluid.fluid_pressure = 2.0
        self.fluid_pub.publish(fluid)


``SyncCallback`` takes both messages relating to both topics, then, from here you can compare these topics, set values, etc. This callback is the final goal of synching multiple topics and the reason why the qos must be the same. This will be seen with the logging statement as both of the times will be the same. Though, the headers have to have the same ``stamp`` value, they don't have to be triggered at the same time using an ``ApproximateTimeSynchronizer`` which will be seen in a delay between logging calls. For both ``TimerCallbacks`` just initialize both the ``Temperature`` and ``FluidPressure`` in whatever way necessary. .

Finally, create a main function and spin the node

.. code-block:: Python

    def main(args=None):
        rclpy.init(args=args)

        time_sync = TimeSyncNode()

        rclpy.spin(time_sync)

        time_sync.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

2. Add the Node to Python Setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

2.1 Update package.xml
^^^^^^^^^^^^^^^^^^^^^^
Navigate to the root of your package's directory, where ``package.xml`` is located, open, and add the following dependencies:

.. code-block:: Python

   <exec_depend>rclpy</exec_depend>
   <exec_depend>message_filters</exec_depend>
   <exec_depend>sensor_msgs</exec_depend>

2.2 Add an entry point
^^^^^^^^^^^^^^^^^^^^^^
To allow the ``ros2 run`` command to run your node, you must add the entry point to ``setup.py``.

Add the following line between the 'console_scripts': brackets, with the name of your package:

.. code-block:: Python

   'approximate_time_sync = pkg_name.approximate_time_sync:main',


3. Build
~~~~~~~~
From the root of your package, build and source.

.. code-block:: bash

    colcon build && . install/setup.zsh

4. Run
~~~~~~
Run replacing the package name with whatever you named your workspace.

.. code-block:: bash

   ros2 run pkg_name approximate_time_sync

You should end up with a result similar to the following:

.. code-block:: bash

   [INFO] [1714927893.485850000] [sync_node]: Sync callback with 1714927893 and 1714927893 as times
   [INFO] [1714927894.489608000] [sync_node]: Sync callback with 1714927894 and 1714927894 as times
