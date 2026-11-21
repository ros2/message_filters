InputAligner (C++):
-------------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters::InputAligner`` filter in ROS 2 using C++.

Unlike synchronizers (such as ``TimeSynchronizer`` or ``ApproximateTime``) that group multiple input messages
into a single combined callback tuple, ``InputAligner`` takes ``N`` input streams and forwards them to
``N`` individual callbacks in strictly chronological order based on their message header timestamps.

In multi-sensor systems (such as robotics state estimation and sensor fusion pipelines combining IMUs, LiDARs,
cameras, and odometry), different sensors publish at different rates and undergo different processing latencies.
Messages frequently arrive at downstream nodes out of order. If processed immediately, an older message arriving late
can break time-monotonic assumptions. ``InputAligner`` buffers messages across all inputs and dispatches them in
timestamp order, while providing:

- **Separate Outputs**: Each input stream maintains its own independent callback signature.
- **Period Hints**: Specifying expected message periods via ``setInputPeriod()`` allows optimistic lookahead, reducing latency.
- **Timeout Protection**: If an input stops publishing, it times out after a configurable duration so other active streams are not blocked indefinitely.
- **Flexible Dispatching**: Messages can be flushed periodically via an automated timer with ``setupDispatchTimer()`` or manually triggered via ``dispatchMessages()``.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already, `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume you've already created an empty ROS 2 package for C++.
Create a new C++ file inside your package's ``src/`` directory, e.g., ``input_aligner_tutorial.cpp``:

.. code-block:: C++

    #include <chrono>
    #include <cinttypes>
    #include <functional>
    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include <geometry_msgs/msg/point_stamped.hpp>

    #include "message_filters/subscriber.hpp"
    #include "message_filters/input_aligner.hpp"

    using namespace std::chrono_literals;

    class InputAlignerExampleNode : public rclcpp::Node
    {
    public:
      using PointMsg = geometry_msgs::msg::PointStamped;
      using Aligner = message_filters::InputAligner<PointMsg, PointMsg>;

      InputAlignerExampleNode()
      : Node("input_aligner_example_node")
      {
        rclcpp::QoS qos(10);

        // Publishers for two simulated sensor streams
        pub_fast_ = this->create_publisher<PointMsg>("sensor_fast", qos);
        pub_slow_ = this->create_publisher<PointMsg>("sensor_slow", qos);

        // Subscriptions wrapped with message_filters
        sub_fast_.subscribe(this, "sensor_fast", qos.get_rmw_qos_profile());
        sub_slow_.subscribe(this, "sensor_slow", qos.get_rmw_qos_profile());

        // Initialize InputAligner with a 1.0 second timeout and the two input filters
        aligner_ = std::make_shared<Aligner>(rclcpp::Duration::from_seconds(1.0), sub_fast_, sub_slow_);

        // Register separate callbacks for each input channel
        aligner_->registerCallback<0>(&InputAlignerExampleNode::fast_sensor_callback, this);
        aligner_->registerCallback<1>(&InputAlignerExampleNode::slow_sensor_callback, this);

        // Provide period hints (fast: 50ms, slow: 100ms) to minimize buffering delay
        aligner_->setInputPeriod<0>(rclcpp::Duration::from_nanoseconds(50'000'000));
        aligner_->setInputPeriod<1>(rclcpp::Duration::from_nanoseconds(100'000'000));

        // Periodic dispatch timer to evaluate and flush aligned messages
        dispatch_timer_ = this->create_wall_timer(
          10ms, [this]() { aligner_->dispatchMessages(); });

        // Timers simulating sensor data generation
        timer_fast_ = this->create_wall_timer(
          50ms, std::bind(&InputAlignerExampleNode::publish_fast_sensor, this));
        timer_slow_ = this->create_wall_timer(
          100ms, std::bind(&InputAlignerExampleNode::publish_slow_sensor, this));
      }

    private:
      void fast_sensor_callback(const PointMsg::ConstSharedPtr & msg)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[Channel 0 - Fast] Processed msg seq=%.0f at stamp=%" PRId64 ".%09u",
          msg->point.x,
          static_cast<int64_t>(msg->header.stamp.sec),
          msg->header.stamp.nanosec);
      }

      void slow_sensor_callback(const PointMsg::ConstSharedPtr & msg)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[Channel 1 - Slow] Processed msg seq=%.0f at stamp=%" PRId64 ".%09u",
          msg->point.x,
          static_cast<int64_t>(msg->header.stamp.sec),
          msg->header.stamp.nanosec);
      }

      void publish_fast_sensor()
      {
        PointMsg msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "sensor_fast_frame";
        msg.point.x = ++count_fast_;
        pub_fast_->publish(msg);
      }

      void publish_slow_sensor()
      {
        PointMsg msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "sensor_slow_frame";
        msg.point.x = ++count_slow_;
        pub_slow_->publish(msg);
      }

      rclcpp::Publisher<PointMsg>::SharedPtr pub_fast_;
      rclcpp::Publisher<PointMsg>::SharedPtr pub_slow_;

      message_filters::Subscriber<PointMsg> sub_fast_;
      message_filters::Subscriber<PointMsg> sub_slow_;

      std::shared_ptr<Aligner> aligner_;

      rclcpp::TimerBase::SharedPtr dispatch_timer_;
      rclcpp::TimerBase::SharedPtr timer_fast_;
      rclcpp::TimerBase::SharedPtr timer_slow_;

      double count_fast_{0.0};
      double count_slow_{0.0};
    };

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<InputAlignerExampleNode>();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Let's break down how this works:

.. code-block:: C++

    #include "message_filters/subscriber.hpp"
    #include "message_filters/input_aligner.hpp"

We include the necessary ``message_filters`` headers. Note that ``message_filters/input_aligner.hpp`` requires
messages with timestamp information (satisfying ``message_traits::TimeGetterFor``, such as any ROS 2 message
containing a standard ``std_msgs/msg/Header``).

.. code-block:: C++

    using PointMsg = geometry_msgs::msg::PointStamped;
    using Aligner = message_filters::InputAligner<PointMsg, PointMsg>;

The ``InputAligner`` template takes the message types for each input stream in order. Here, we instantiate
an aligner handling two ``PointStamped`` channels.

.. code-block:: C++

    aligner_ = std::make_shared<Aligner>(rclcpp::Duration::from_seconds(1.0), sub_fast_, sub_slow_);

We construct the aligner with a 1.0 second timeout and connect the two subscribers. Alternatively, you can
instantiate an empty aligner and connect filters subsequently with ``aligner_->connectInput(sub_fast_, sub_slow_)``.

.. code-block:: C++

    aligner_->registerCallback<0>(&InputAlignerExampleNode::fast_sensor_callback, this);
    aligner_->registerCallback<1>(&InputAlignerExampleNode::slow_sensor_callback, this);

Unlike synchronizers, callbacks are registered per input index using a non-type template argument (``<0>``, ``<1>``, etc.).
Each callback receives only the message from its respective channel.

.. code-block:: C++

    aligner_->setInputPeriod<0>(rclcpp::Duration::from_nanoseconds(50'000'000));
    aligner_->setInputPeriod<1>(rclcpp::Duration::from_nanoseconds(100'000'000));

Providing an expected message period allows the aligner to look ahead by that duration. This enables it to release
available messages sooner when it knows a subsequent message is not expected immediately.

.. code-block:: C++

    dispatch_timer_ = this->create_wall_timer(
      10ms, [this]() { aligner_->dispatchMessages(); });

Messages in the buffer are evaluated and flushed when ``dispatchMessages()`` is invoked. Calling this
regularly inside a timer ensures low forwarding latency. Alternatively, you can use ``setupDispatchTimer(node_shared_ptr, update_rate)``
after node construction.

2. Monitoring Queue Status
~~~~~~~~~~~~~~~~~~~~~~~~~~

The aligner allows querying the queue health for any channel:

.. code-block:: C++

    auto status = aligner_->getQueueStatus<0>();
    RCLCPP_INFO(
      this->get_logger(),
      "Active: %d, Queued: %zu, Processed: %zu, Dropped: %zu",
      status.active, status.queue_size, status.msgs_processed, status.msgs_dropped);

- ``active``: ``true`` if the channel is currently healthy; ``false`` if the channel has timed out.
- ``queue_size``: Number of messages waiting in the buffer for this channel.
- ``msgs_processed``: Total messages successfully forwarded.
- ``msgs_dropped``: Messages dropped (e.g., if a message arrives with a timestamp older than the last dispatched output).

3. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Open ``package.xml`` in your package root and add:

.. code-block:: xml

    <depend>geometry_msgs</depend>
    <depend>message_filters</depend>
    <depend>rclcpp</depend>

4. Add the Node to CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the executable in your ``CMakeLists.txt``:

.. code-block:: CMake

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(message_filters REQUIRED)
    find_package(geometry_msgs REQUIRED)

    add_executable(input_aligner_tutorial src/input_aligner_tutorial.cpp)
    ament_target_dependencies(input_aligner_tutorial
      rclcpp
      message_filters
      geometry_msgs
    )

    install(TARGETS
      input_aligner_tutorial
      DESTINATION lib/${PROJECT_NAME}
    )

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

You will observe output where messages from both channels are dispatched in strictly increasing timestamp order:

.. code-block:: console

    [INFO] [1773695000.052100000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=1 at stamp=1773695000.050000000
    [INFO] [1773695000.102300000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=2 at stamp=1773695000.100000000
    [INFO] [1773695000.102500000] [input_aligner_example_node]: [Channel 1 - Slow] Processed msg seq=1 at stamp=1773695000.100000000
    [INFO] [1773695000.152400000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=3 at stamp=1773695000.150000000
    [INFO] [1773695000.202100000] [input_aligner_example_node]: [Channel 0 - Fast] Processed msg seq=4 at stamp=1773695000.200000000
    [INFO] [1773695000.202400000] [input_aligner_example_node]: [Channel 1 - Slow] Processed msg seq=2 at stamp=1773695000.200000000
