Approximate Time Synchronizer (C++):
---------------------------------------

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
  #include "message_filters/synchronizer.h"
  #include "message_filters/sync_policies/approximate_time.h"

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
      std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Temperature, sensor_msgs::msg::FluidPressure>>> sync;
      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::TimerBase::SharedPtr second_timer;
  };


For this example we will be using the ``temperature`` and ``fluid_pressure`` messages found in
`sensor_msgs <https://github.com/ros2/common_interfaces/tree/jazzy/sensor_msgs/msg>`_.
To simulate a working ``Synchronizer`` using the ``ApproximateTime`` Policy. We will be publishing and subscribing to topics of those respective types, to showcase how real sensors would be working.

.. code-block:: C++

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fluid_pub;
    message_filters::Subscriber<sensor_msgs::msg::Temperature> temp_sub;
    message_filters::Subscriber<sensor_msgs::msg::FluidPressure> fluid_sub;

Notice that the ``Subscribers`` are in the ``message_filters`` namespace, while we can utilize ``rclcpp::Publishers``. To simulate them we will also need two ``TimerBases``. Then, we will be utilizing a ``Synchronizer`` to get these messages from the sensor topics aligned.

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

      timer = this->create_wall_timer(500ms, std::bind(&TimeSyncNode::TimerCallback, this));
      second_timer = this->create_wall_timer(550ms, std::bind(&TimeSyncNode::SecondTimerCallback, this));

      uint32_t queue_size = 10;
      sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
        ApproximateTime<sensor_msgs::msg::Temperature, sensor_msgs::msg::FluidPressure>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Temperature,
        sensor_msgs::msg::FluidPressure>(queue_size), temp_sub, fluid_sub);

      sync->setAgePenalty(0.50);
      sync->registerCallback(std::bind(&TimeSyncNode::SyncCallback, this, _1, _2));

     }

It is essential that the QoS is the same for all of the publishers and subscribers, otherwise the Message Filter cannot align the topics together. So, create one ``rclcpp::QoS`` and stick with it, or find out what ``qos`` is being used in the native sensor code, and replicate it. For each private class member, do basic construction of the object relating to the ``Node`` and callback methods that may be used in the future. Both of the two timers we utilize will have different timer values of ``500ms`` and ``550ms`` which causes the timers to off at different points, which is an advantage of using ``ApproximateTime``. This will then work since we called ``setAgePenalty`` to ``0.50`` (50ms)  Notice that we must call ``sync->registerCallback`` to sync up the two (or more) chosen topics.

So, we must create three (or more) private callbacks, one for the ``Synchronizer``, then two for our ``TimerBases`` which are each for a certain ``sensor_msgs``.

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
      auto now = this->get_clock()->now();
      temp.header.stamp = now;
      temp.header.frame_id = "test";
      temp.temperature = 1.0;
      temp_pub->publish(temp);
    }

    void SecondTimerCallback()
    {
      sensor_msgs::msg::FluidPressure fluid;
      auto now = this->get_clock()->now();
      fluid.header.stamp = now;
      fluid.header.frame_id = "test";
      fluid.fluid_pressure = 2.0;
      fluid_pub->publish(fluid);
    }


``SyncCallback`` takes ``const shared_ptr references`` relating to both topics becasue they will be taken at the exact time, from here you can compare these topics, set values, etc. This callback is the final goal of synching multiple topics and the reason why the qos and header stamps must be the same. This will be seen with the logging statement as both of the times will be the same. Though, the headers have to have the same ``stamp`` value, they don't have to be triggered at the same time with ``ApproximateTime`` which will be seen in a delay between logging calls. For the ``TimerCallback`` just initialize both the ``Temperature`` and ``FluidPressure`` in whatever way necessary. .

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
Now open the ``CMakeLists.txt`` add the executable and name it ``approximate_time_sync``, which you’ll use later with ``ros2 run``.

.. code-block:: C++

   find_package(rclcpp REQUIRED)
   find_package(sensor_msgs REQUIRED)
   find_package(message_filters REQUIRED)

   add_executable(approximate_time_sync src/approximate_time_synchronizer.cpp)
   ament_target_dependencies(approximate_time_sync rclcpp sensor_msgs message_filters)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: C++

    install(TARGETS
        approximate_time_sync
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

   ros2 run pkg_name approximate_time_sync

You should end up with a result similar to the following:

.. code-block:: bash

    [INFO] [1714888439.264005000] [sync_node]: Sync callback with 1714888438 and 1714888438 as times
    [INFO] [1714888445.263986000] [sync_node]: Sync callback with 1714888444 and 1714888444 as times

* Note the ~0.5 second difference between each callback, this is because the ``ApproximateTime`` calls will be stored in a queue which can seen to trigger once the headers of the two (or more) elements are the same, which makes sense because our longest timer wait is ``550ms``, aligning with our age penalty.
