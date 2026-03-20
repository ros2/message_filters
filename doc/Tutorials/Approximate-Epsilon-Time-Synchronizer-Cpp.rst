Approximate Epsilon Time Synchronizer (C++):
---------------------------------------

Prerequisites
~~~~~~~~~~~~~
This tutorial assumes you have a working knowledge of ROS 2

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for C++.
The next step is to create a new C++ file inside your package, e.g., ``approximate_time_sync_tutorial``, and create an example node:

.. code-block:: C++


  #include <chrono>
  #include <functional>
  #include <memory>

  #include "rclcpp/rclcpp.hpp"

  #include <sensor_msgs/msg/temperature.hpp>
  #include <sensor_msgs/msg/fluid_pressure.hpp>

  #include "message_filters/subscriber.hpp"
  #include "message_filters/synchronizer.hpp"
  #include "message_filters/sync_policies/approximate_epsilon_time.hpp"

  using namespace std::chrono_literals;

  using std::placeholders::_1;
  using std::placeholders::_2;

  class EpsTimeSyncNode : public rclcpp::Node {
  public:
    typedef sensor_msgs::msg::Temperature TemperatureMsg;
    typedef sensor_msgs::msg::FluidPressure FluidPressureMsg;
    typedef message_filters::sync_policies::ApproximateEpsilonTime<
      sensor_msgs::msg::Temperature,
      sensor_msgs::msg::FluidPressure
    > SyncPolicy;

    EpsTimeSyncNode(): Node("epsilon_time_sync_node") {
      rclcpp::QoS qos = rclcpp::QoS(10);
      temp_pub = this->create_publisher<TemperatureMsg>("temp", qos);
      fluid_pub = this->create_publisher<FluidPressureMsg>("fluid", qos);

      temp_sub.subscribe(this, "temp", qos);
      fluid_sub.subscribe(this, "fluid", qos);

      temperature_timer = this->create_wall_timer(500ms, std::bind(&EpsTimeSyncNode::TemperatureTimerCallback, this));
      fluid_pressure_timer = this->create_wall_timer(550ms, std::bind(&EpsTimeSyncNode::FluidPressureTimerCallback, this));

      uint32_t queue_size = 10;
      sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>> (
        SyncPolicy (
          queue_size,
          rclcpp::Duration(std::chrono::seconds(1))
        ),
        temp_sub,
        fluid_sub
      );

      sync->registerCallback(std::bind(&EpsTimeSyncNode::SyncCallback, this, _1, _2));
    }

  private:

    void SyncCallback(
      const TemperatureMsg::ConstSharedPtr & temp,
      const FluidPressureMsg::ConstSharedPtr & fluid
    ) {
      RCLCPP_INFO(
        this->get_logger(),
        "Sync callback with %u.%u and %u.%u as times",
        temp->header.stamp.sec,
        temp->header.stamp.nanosec,
        fluid->header.stamp.sec,
        fluid->header.stamp.nanosec
      );

      if (temp->temperature > 2.0)
      {
        FluidPressureMsg new_fluid;
        new_fluid.header.stamp = rclcpp::Clock().now();
        new_fluid.header.frame_id = "test";
        new_fluid.fluid_pressure = 2.5;
        fluid_pub->publish(new_fluid);
      }
    }

    void TemperatureTimerCallback()
    {
      TemperatureMsg temp;
      auto now = this->get_clock()->now();
      temp.header.stamp = now;
      temp.header.frame_id = "test";
      temp.temperature = 1.0;
      temp_pub->publish(temp);
    }

    void FluidPressureTimerCallback()
    {
      FluidPressureMsg fluid;
      auto now = this->get_clock()->now();
      fluid.header.stamp = now;
      fluid.header.frame_id = "test";
      fluid.fluid_pressure = 2.0;
      fluid_pub->publish(fluid);
    }

  private:
    rclcpp::Publisher<TemperatureMsg>::SharedPtr temp_pub;
    rclcpp::Publisher<FluidPressureMsg>::SharedPtr fluid_pub;

    message_filters::Subscriber<TemperatureMsg> temp_sub;
    message_filters::Subscriber<FluidPressureMsg> fluid_sub;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    rclcpp::TimerBase::SharedPtr temperature_timer;
    rclcpp::TimerBase::SharedPtr fluid_pressure_timer;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EpsTimeSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
  }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~
Now, let's break down this code and examine the details.

.. code-block:: C++

  #include <chrono>
  #include <functional>
  #include <memory>

  #include "rclcpp/rclcpp.hpp"

  #include <sensor_msgs/msg/temperature.hpp>
  #include <sensor_msgs/msg/fluid_pressure.hpp>

  #include "message_filters/subscriber.hpp"
  #include "message_filters/synchronizer.hpp"
  #include "message_filters/sync_policies/approximate_epsilon_time.hpp"

  using namespace std::chrono_literals;

  using std::placeholders::_1;
  using std::placeholders::_2;

We start with the include section. First of all we include the headers from standard library.
The ``chrono`` header is for the duration classes and the ``chrono_literals`` namespace.
The ``functional`` header is included to gain access to the ``std::bind`` function template.
And the ``memory`` header provides us with ``std::shared_ptr`` class and ``std::make_shared`` function template.
The ``rclcpp.hpp`` needed to access the classes from ``rclcpp`` namespace.
Next we add ``temperature.hpp`` and ``fluid_pressure.hpp`` message headers from ``sensor_msgs`` library.
These provide us with ``TemperatureMsg`` and ``FluidPressureMsg`` classes respectively.
These messages will serve as an example of a sensory data.
Also both of them do have a ``Header`` field, that is required by the ``Synchronizer`` filter.
The last includes are ``subscriber.hpp``, ``synchronizer.hpp`` and ``approximate_epsilon_time.hpp``.
The first will provide us with ``Subscriber`` and ``Synchronizer`` filters.
The last one will give us the access to the ``ApproximateEpsilonTime`` sync policy.

Next we define a tutorial class.
In this case it is the ``EpsTimeSyncNode`` class.
For starters, let's take a look at the class members declared in the ``private`` section of this class:

.. code-block:: C++

    rclcpp::Publisher<TemperatureMsg>::SharedPtr temp_pub;
    rclcpp::Publisher<FluidPressureMsg>::SharedPtr fluid_pub;

    message_filters::Subscriber<TemperatureMsg> temp_sub;
    message_filters::Subscriber<FluidPressureMsg> fluid_sub;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    rclcpp::TimerBase::SharedPtr temperature_timer;
    rclcpp::TimerBase::SharedPtr fluid_pressure_timer;

To publish messages we will need two ``Publisher`` objects.
One will be used for publishing ``TemperatureMsg`` and the other for publishing ``FluidPressureMsg``.

To get inputs for the ``Synchronizer`` filter we add two instances of the ``Subscriber`` filter.
Next is the main star of the show, the ``Synchronizer`` filter.

And finally, to automate publishing messages and for querying the cache filter we add two timers,
the ``temperature_timer`` and the ``fluid_pressure_timer``.
These are going to invoke publishing of ``TemperatureMsg`` and ``FluidPressureMsg`` messages respectively.

Next let's take a look at the public interface of this class.

.. code-block:: C++

    typedef sensor_msgs::msg::Temperature TemperatureMsg;
    typedef sensor_msgs::msg::FluidPressure FluidPressureMsg;
    typedef message_filters::sync_policies::ApproximateEpsilonTime<
      sensor_msgs::msg::Temperature,
      sensor_msgs::msg::FluidPressure
    > SyncPolicy;

    EpsTimeSyncNode(): Node("epsilon_time_sync_node") {
      rclcpp::QoS qos = rclcpp::QoS(10);
      temp_pub = this->create_publisher<TemperatureMsg>("temp", qos);
      fluid_pub = this->create_publisher<FluidPressureMsg>("fluid", qos);

      temp_sub.subscribe(this, "temp", qos);
      fluid_sub.subscribe(this, "fluid", qos);

      temperature_timer = this->create_wall_timer(500ms, std::bind(&EpsTimeSyncNode::TemperatureTimerCallback, this));
      fluid_pressure_timer = this->create_wall_timer(550ms, std::bind(&EpsTimeSyncNode::FluidPressureTimerCallback, this));

      uint32_t queue_size = 10;
      sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>> (
        SyncPolicy (
          queue_size,
          rclcpp::Duration(std::chrono::seconds(1))
        ),
        temp_sub,
        fluid_sub
      );

      sync->registerCallback(std::bind(&EpsTimeSyncNode::SyncCallback, this, _1, _2));
    }

Ultimately, the public interface of the ``EpsTimeSyncNode`` class
boils down to a few ``typedef`` aliases and a constructor.
We define aliases for the ``Temperature`` and the ``FluidPressure`` messages
and for the ``ApproximateEpsilonTime`` synchronization policy.

In the constructor we declare ``QoS`` and message publishers.
Next we initialize subscription to the corresponding topics for ``temp_sub`` and ``fluid_sub`` message filters.
Note that all these subscriptions and publishers utilize the same ``QoS``.
After that the publisher timers are created.
And the last thing to do is to construct the ``Synchronizer`` filter.
The ``Synchronizer`` class constructor accepts a synchronization policy and a set of filters with incoming data.
The synchronization policy requires a ``queue_size`` and an instance of the ``rclcpp::Duration`` class
that will serve as an ``Epsilon`` of the ``ApproximateEpsilonTime``.
We'll touch on the meaning of this ``Epsilon`` a bit later.

Now let's take a look at the timer and synchronization callbacks defined also in the ``private`` section of this class.

.. code-block:: C++

    void SyncCallback(
      const TemperatureMsg::ConstSharedPtr & temp,
      const FluidPressureMsg::ConstSharedPtr & fluid
    ) {
      RCLCPP_INFO(
        this->get_logger(),
        "Sync callback with %u.%u and %u.%u as times",
        temp->header.stamp.sec,
        temp->header.stamp.nanosec,
        fluid->header.stamp.sec,
        fluid->header.stamp.nanosec
      );

      if (temp->temperature > 2.0)
      {
        FluidPressureMsg new_fluid;
        new_fluid.header.stamp = rclcpp::Clock().now();
        new_fluid.header.frame_id = "test";
        new_fluid.fluid_pressure = 2.5;
        fluid_pub->publish(new_fluid);
      }
    }

    void TemperatureTimerCallback()
    {
      TemperatureMsg temp;
      auto now = this->get_clock()->now();
      temp.header.stamp = now;
      temp.header.frame_id = "test";
      temp.temperature = 1.0;
      temp_pub->publish(temp);
    }

    void FluidPressureTimerCallback()
    {
      FluidPressureMsg fluid;
      auto now = this->get_clock()->now();
      fluid.header.stamp = now;
      fluid.header.frame_id = "test";
      fluid.fluid_pressure = 2.0;
      fluid_pub->publish(fluid);
    }

These are pretty straightforward.
The ``TemperatureTimerCallback`` is executed by the ``temperature_timer``.
The ``FluidPressureTimerCallback`` is executed by the ``fluid_pressure_timer``.
These two just construct corresponding messages and publish them to corresponding topics.
The ``SyncCallback`` is invoked by the ``Synchronizer`` filter in case if two messages meet the synchronization criterion.

Finally, we create a main function and spin the node there.

.. code-block:: C++

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<TimeSyncNode>();
      rclcpp::spin(node);
      rclcpp::shutdown();

      return 0;
    }

2. ApproximateEpsilonTime synchronization policy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's talk a bit about the ``ApproximateEpsilonTime`` synchronization policy.
As described `here <https://docs.ros.org/en/rolling/p/message_filters/doc/index.html#approximateepsilontime-policy>`
the ``ApproximateEpsilonTime`` policy requires messages to have the same timestamp within an ``Epsilon`` tolerance in order to match them.
For a synchronization callback to be invoked it is mandatory that messages should be received via all specified channels.
If all the channels have provided messages, but some of them do not fit the ``Epsilon``
a synchronization callback is not invoked and no messages are passed to the following filters.
We'll demonstrate this further down the line in this tutorial.
Now let's finish setting up the project, build and run it.

3. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>message_filters</depend>
    <depend>rclcpp</depend>
    <depend>sensor_msgs</depend>

4. Add the Node to a CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Now open the ``CMakeLists.txt`` add the executable and name it ``approximate_time_sync_tutorial``, which you’ll use later with ``ros2 run``.

.. code-block:: CMake

   find_package(ament_cmake_auto REQUIRED)
   ament_auto_find_build_dependencies()

   add_executable(approximate_time_sync_tutorial src/approximate_time_sync_tutorial.cpp)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: CMake

    install(TARGETS
        approximate_time_sync_tutorial
        DESTINATION lib/${PROJECT_NAME})

5. Build Your Package
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

    ros2 run approximate_time_sync_tutorial approximate_time_sync_tutorial

The console output should look something like this:

.. code-block:: console

  [INFO] [1773177603.600396546] [epsilon_time_sync_node]: Sync callback with 1773177603.550153015 and 1773177603.600124849 as times
  [INFO] [1773177604.151056035] [epsilon_time_sync_node]: Sync callback with 1773177604.50326582 and 1773177604.150421822 as times
  [INFO] [1773177604.700414783] [epsilon_time_sync_node]: Sync callback with 1773177604.550350034 and 1773177604.700177922 as times
  [INFO] [1773177605.250957771] [epsilon_time_sync_node]: Sync callback with 1773177605.50110559 and 1773177605.250282496 as times
  [INFO] [1773177605.800939553] [epsilon_time_sync_node]: Sync callback with 1773177605.550309955 and 1773177605.800346632 as times
  [INFO] [1773177606.350348802] [epsilon_time_sync_node]: Sync callback with 1773177606.50311161 and 1773177606.350160222 as times
  [INFO] [1773177606.900890167] [epsilon_time_sync_node]: Sync callback with 1773177606.550157175 and 1773177606.900339428 as times

Let's take a look at a timestamp difference between values from the first line.

.. code-block:: console
  1773177603.600124849 sec - 1773177603.550153015 sec = 0.04997181892 sec

The resulting ``0.04997181892`` seconds difference is definitely within the specified ``Epsilon``.
This the synchronization callback is executed.

.. code-block:: C++

    rclcpp::Duration(std::chrono::seconds(1))

Let's change the ``Epsilon`` value to ``5000000`` nanoseconds
Navigate to the ``EpsTimeSyncNode`` constructor and make the change.

.. code-block:: C++

    // replace
    rclcpp::Duration(std::chrono::seconds(1))

    // with
    rclcpp::Duration(std::chrono::nanoseconds(5000000))

After this change, you'll need to recompile the project and run it again.
The output will look as follows:

.. code-block:: console

  [INFO] [1773177900.106703711] [epsilon_time_sync_node]: Sync callback with 1773177900.106191759 and 1773177900.105995382 as times
  [INFO] [1773177905.606790305] [epsilon_time_sync_node]: Sync callback with 1773177905.606144962 and 1773177905.605920351 as times

There will be much less message pairs causing an execution of the synchronization callback.
And the time delta between them will be less then the specified ``epsilon``.
For example, for the timestamps from the first line output ``1773177900.106191759`` and ``1773177900.105995382``
the delta is ``196377`` nanoseconds which is less then ``5000000`` nanoseconds.