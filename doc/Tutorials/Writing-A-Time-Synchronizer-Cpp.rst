Time Synchronizer (C++):
---------------------------

Prerequisites
~~~~~~~~~~~~~
This tutorial assumes you have a working knowledge of ROS 2

If you have not done so already `create a workspace <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_


1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: C++

    #include "rclcpp/rclcpp.hpp"

    #include <chrono>
    #include <functional>
    #include <memory>

    #include "message_filters/subscriber.h"
    #include "message_filters/time_synchronizer.h"

    #include "sensor_msgs/msg/temperature.hpp"
    #include "sensor_msgs/msg/fluid_pressure.hpp"

    using namespace std::chrono_literals;

    using std::placeholders::_1;
    using std::placeholders::_2;

    class TimeSyncNode : public rclcpp::Node
    {
    public:

    private:

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pub;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> temp_sub;
    message_filters::Subscriber<sensor_msgs::msg::FluidPressure> fluid_sub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Temperature,
      sensor_msgs::msg::FluidPressure>> sync;
    rclcpp::TimerBase::SharedPtr timer;
    };


For this example we will be using the ``temperature`` and ``fluid_pressure`` messages found in
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs/msg>`_.
To simulate a working ``TimeSynchronizer`` we will be publishing and subscribing to topics of those respective types, to showcase how real sensors would be working.

.. code-block:: C++

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pub;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> temp_sub;
    message_filters::Subscriber<sensor_msgs::msg::FluidPressure> fluid_sub

Notice that the ``Subscribers`` are in the ``message_filters`` namespace, while we can utilize ``rclcpp::Publishers``. To simulate them we will also need some sort of ``TimerBase``. Then, we will be utilizing a ``TimeSynchronizer`` to get these messages from the sensor topics aligned.

Next, we can initialize these private elements within a basic ``Node`` constructor

.. code-block:: C++

  public:

    TimeSyncNode() : Node("sync_node")
    {
      rclcpp::QoS qos = rclcpp::QoS(10);
      temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>("temp", qos);
      fluid_pub = this->create_publisher<sensor_msgs::msg::FluidPressure>("fluid", qos);

      temp_sub.subscribe(this, "temp", qos.get_rmw_qos_profile());
      fluid_sub.subscribe(this, "fluid", qos.get_rmw_qos_profile());

      timer = this->create_wall_timer(1000ms, std::bind(&TimeSyncNode::TimerCallback, this));

      uint32_t queue_size = 10;
      sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Temperature,
        sensor_msgs::msg::FluidPressure>>(temp_sub, fluid_sub, queue_size);

      sync->registerCallback(std::bind(&TimeSyncNode::SyncCallback, this, _1, _2));

    }

It is essential that the QoS is the same for all of the publishers and subscribers, otherwise the Message Filter cannot align the topics together. So, create one ``rclcpp::QoS`` and stick with it, or find out what ``qos`` is being used in the native sensor code, and replicate it. For each private class member, do basic construction of the object relating to the ``Node`` and callback methods that may be used in the future. Notice that we must call ``sync->registerCallback`` to sync up the two (or more) chosen topics.

So, we must create some private callbacks.

.. code-block:: C++

    private:

    void SyncCallback(const sensor_msgs::msg::Temperature::ConstSharedPtr & temp,
        const sensor_msgs::msg::FluidPressure::ConstSharedPtr & fluid)
    {
      RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times",
        temp->header.stamp.sec, fluid->header.stamp.sec);
      if (temp->temperature > 2.0)
      {
        sensor_msgs::msg::FluidPressure new_fluid;
        new_fluid.header.stamp = rclcpp::Clock().now();
        new_fluid.header.frame_id = "test";
        new_fluid.fluid_pressure = 2.5;
        fluid_pub->publish(new_fluid);
      }
    }

    void TimerCallback()
    {
      sensor_msgs::msg::Temperature temp;
      sensor_msgs::msg::FluidPressure fluid;
      auto now = rclcpp::Clock().now();

      temp.header.stamp = now;
      temp.header.frame_id = "test";
      temp.temperature = 1.0;
      temp_pub->publish(temp);

      fluid.header.stamp = now;
      fluid.header.frame_id = "test";
      fluid.fluid_pressure = 2.0;
      fluid_pub->publish(fluid);
    }

``SyncCallback`` takes ``const shared_ptr references`` relating to both topics becasue they will be taken at the exact time, from here you can compare these topics, set values, etc. This callback is the final goal of synching multiple topics and the reason why the qos and header stamps must be the same. This will be seen with the logging statement as both of the times will be the same. For the ``TimerCallback`` just initialize both the ``Temperature`` and ``FluidPressure`` in whatever way necessary, but make sure the header stamp of both have the same exact time, otherwise the ``TimeSynchronizer`` will be misaligned and won't do anything. This is becasue the ``TimeSynchronizer`` has an ``ExactTime`` sync policy.

Finally, create a main function and spin the node

.. code-block:: C++

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<TimeSyncNode>();
      rclcpp::spin(node);
      rclcpp::shutdown();

      return 0;
    }


2. Add the Node to a CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Now open the ``CMakeLists.txt`` add the executable and name it ``time_sync``, which you’ll use later with ``ros2 run``.

.. code-block:: C++

   find_package(rclcpp REQUIRED)
   find_package(sensor_msgs REQUIRED)
   find_package(message_filters REQUIRED)

   add_executable(time_sync src/time_synchronizer.cpp)
   ament_target_dependencies(time_sync rclcpp sensor_msgs message_filters)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: C++

    install(TARGETS
        time_sync
        DESTINATION lib/${PROJECT_NAME})


3. Build
~~~~~~~~
From the root of your package, build and source.

.. code-block:: bash

    colcon build && . install/setup.zsh

4. Run
~~~~~~
Run replacing the package name with whatever you named your workspace.

.. code-block:: bash

   ros2 run pkg_name time_sync

You should end up with a result similar to the following:

.. code-block:: bash

   [INFO] [1714504937.157035000] [sync_node]: Sync callback with 1714504937 and 1714504937 as times
