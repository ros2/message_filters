DeltaFilter (C++):
------------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters::DeltaFilter`` class in ROS 2 using C++.
With the ``DeltaFilter`` given a stream of messages, the message is passed down
to the next filter if any of the message fields, that may be acquired
by ``field_getters`` have changed compared to the previously accepted message.

To demonstrate the functionality of the ``DeltaFilter`` filter in a more clear manner, we are going to add a custom filter to this tutorial.
This is going to be the ``CounterFilter`` that will be counting the number of messages passing through it.
This filter class will be a successor to the ``SimpleFilter`` class, but this is a topic for another tutorial.

For more information on this succession mechanism, please refer to the `SimpleFilter for C++ tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Cpp.html>`_.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for C++.
The next step is to create a new C++ file inside your package, e.g., ``delta_tutorial.cpp``, and write an example code:

.. code-block:: C++

  #include <chrono>
  #include <functional>
  #include <string>

  #include <rclcpp/rclcpp.hpp>

  #include <message_filters/delta_filter.hpp>
  #include <message_filters/subscriber.hpp>

  #include <std_msgs/msg/string.hpp>

  using namespace std::chrono_literals;

  const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";
  const rclcpp::QoS QOS_PROFILE = rclcpp::QoS(10);

  class CounterFilter : public message_filters::SimpleFilter<std_msgs::msg::String> {
  public:
    typedef message_filters::MessageEvent<std_msgs::msg::String const> EventType;

    CounterFilter() : message_filters::SimpleFilter<std_msgs::msg::String>() {}

    void add(const EventType & evt) {
      ++counter_;
      signalMessage(evt);
    }

    template<class F>
    void connectInput(F & f) {
      incoming_connection_.disconnect();
      incoming_connection_ = f.registerCallback(
        message_filters::SimpleFilter<std_msgs::msg::String>::EventCallback(
          std::bind(
            &CounterFilter::add,
            this,
            std::placeholders::_1
          )
        )
      );
    }

    size_t getCounterValue() {
      return counter_;
    }
  private:
    size_t counter_ = 0;

    message_filters::Connection incoming_connection_;
  };

  class DeltaNode : public rclcpp::Node {
  public:
    typedef std::shared_ptr<std_msgs::msg::String const> MConstPtr;

    DeltaNode():
      Node("DeltaNode"),
      subscriber_filter_(),
      delta_filter_(
        subscriber_filter_,
        {[] (const MConstPtr& msg) {return msg->data;}}
      ),
      counter_filter_()
    {
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, QOS_PROFILE);

      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, QOS_PROFILE);

      counter_filter_.connectInput(delta_filter_);

      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::publisher_timer_callback, this)
      );

      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::query_timer_callback, this)
      );
    }

    void query_timer_callback() {
      RCLCPP_INFO(
        get_logger(),
        "Messages passed down the chain count: %zu",
        counter_filter_.getCounterValue()
      );
    }

    void publisher_timer_callback() {
      auto message = std_msgs::msg::String();
      if (pub_count_ % 2 == 0) {
        message.data = "pub_count = " + std::to_string(pub_count_);
      } else {
        message.data = "pub_count = " + std::to_string(pub_count_ - 1);
      }
      publisher_->publish(message);
      ++pub_count_;
    }

  private:
    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;

    message_filters::DeltaFilter<std_msgs::msg::String> delta_filter_;
    CounterFilter counter_filter_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;

    int pub_count_ { 0 };
  };

  int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
  {
    rclcpp::init(argc, argv);

    auto delta_node = std::make_shared<DeltaNode>();

    rclcpp::spin(delta_node);
    rclcpp::shutdown();

    return 0;
  }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's break down this code and examine the details.

.. code-block:: C++

  #include <chrono>
  #include <functional>
  #include <string>

  #include <rclcpp/rclcpp.hpp>

  #include <message_filters/delta_filter.hpp>
  #include <message_filters/subscriber.hpp>

  #include <std_msgs/msg/string.hpp>

  using namespace std::chrono_literals;

  const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";
  const rclcpp::QoS QOS_PROFILE = rclcpp::QoS(10);

We start by including ``chrono``, ``string`` and ``functional`` headers.
The ``chrono`` header is required for the ``chrono_literals`` namespace, necessary for creating timers.
The ``string`` header provides us with ``std::to_string`` function required for the main functionality demonstration.
The ``functional`` header is also required to use ``std::bind`` function to bind timer callbacks to timers.

After that we include the ``rclcpp.hpp`` header that provides us with classes from ``rclcpp`` namespace.
To use filters in our code we need corresponding headers as well.
In this case we include ``delta_filter.hpp`` for ``DeltaFilter`` and ``subscriber.hpp`` for ``Subscriber`` filter respectively.
And finally we add ``std_msgs/msg/string.hpp`` to get access to ``String`` message class from the ROS standard messages library.

Lastly, in this section we declare that we are using ``chrono_literals`` namespace.
We define the ``TUTORIAL_TOPIC_NAME`` and ``QOS_PROFILE`` to use them for subscription and for publishing.

Next we define ``CounterFilter`` class

.. code-block:: C++

  public:
    typedef message_filters::MessageEvent<std_msgs::msg::String const> EventType;

    CounterFilter() : message_filters::SimpleFilter<std_msgs::msg::String>() {}
    void add(const EventType & evt) {
      ++counter_;
      signalMessage(evt);
    }

    template<class F>
    void connectInput(F & f) {
      incoming_connection_.disconnect();
      incoming_connection_ = f.registerCallback(
        message_filters::SimpleFilter<std_msgs::msg::String>::EventCallback(
          std::bind(
            &CounterFilter::add,
            this,
            std::placeholders::_1
          )
        )
      );
    }

    size_t getCounterValue() {
      return counter_;
    }
  private:
    size_t counter_ = 0;

    message_filters::Connection incoming_connection_;
  };

This filter counts the number of messages passing through it.
The ``add`` method increases the message count, and passes the message to the following filter via ``signalMessage``.
The ``connectInput`` connects this filter to a previous filter's output by registering ``add`` as an ``EventCallback`` so the event is forwarded with no extra copies.
The ``getCounterValue`` grants access to the current message count.

More on this succession mechanism may be found in the corresponding tutorial: `SimpleFilter for C++ tutorial <https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/SimpleFilter-Cpp.html>`_.

And now we can turn our attention to the main tutorial class, that is the ``DeltaNode`` class.
First let's turn our attention to the ``private`` section of this class.

.. code-block:: C++

    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;

    message_filters::DeltaFilter<std_msgs::msg::String> delta_filter_;
    CounterFilter counter_filter_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;

    int pub_count_ { 0 };

We start with a ``Subscriber`` filter, that is going to be an entry point for the messages into our chain of filters.
After that we add ``DeltaFilter`` and ``CounterFilter`` to the class.
To publish messages we will need a ``Publisher`` object.
To automate publishing messages and for querying the chain filter we add two timers, the ``publisher_timer_`` and the ``query_timer_`` respectively.

To demonstrate the functionality of the ``DeltaFilter`` we need to change only some of the messages.
For that purpose we add ``pub_count_``.
Every even message embeds the current value of this counter, and every odd message re-uses the previous value, so its ``data`` field is identical to its predecessor.
The details of the implementation may be found in ``publisher_timer_callback``.

Now let's take a look at the ``DeltaNode`` constructor.

.. code-block:: C++

    DeltaNode():
      Node("DeltaNode"),
      subscriber_filter_(),
      delta_filter_(
        subscriber_filter_,
        {[] (const MConstPtr& msg) {return msg->data;}}
      ),
      counter_filter_()
    {
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, QOS_PROFILE);

      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, QOS_PROFILE);

      counter_filter_.connectInput(delta_filter_);

      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::publisher_timer_callback, this)
      );

      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::query_timer_callback, this)
      );
    }

We start with constructing the basic ``Node`` class and all the filters required in this example.

.. code-block:: C++

      Node("DeltaNode"),
      subscriber_filter_(),
      delta_filter_(
        subscriber_filter_,
        {[] (const MConstPtr& msg) {return msg->data;}}
      ),
      counter_filter_()

After that we subscribe to the example topic via ``subscriber_filter_``.
Following we create the ``delta_filter_``.
Please note this line of code.

.. code-block:: C++

        {[] (const MConstPtr& msg) {return msg->data;}}

Here we pass the list of callable objects, in this case consisting only of one element.
In this case it is a ``lambda``.
This ``lambda`` will be applied to every message passing through this ``delta_filter_``
and to the last message that have passed it successfully.
If any field, that was retrieved by any ``field_getter`` callable, differs between new and old message
the new message is accepted and passed down the chain of filters.
Any subsequent message will be compared to this one.

And finally the ``counter_filter_``.

The body of the constructor starts with initializing the ``subscriber_filter_``'s subscription.

.. code-block:: C++

      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, QOS_PROFILE);

Next we create a publisher for us to be able to populate the ``TUTORIAL_TOPIC`` with some messages.

.. code-block:: C++

      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, QOS_PROFILE);

Lastly we connect the input of the ``counter_filter_`` to the ``delta_filter_`` so that the messages that pass the ``delta_filter_`` end up in the ``counter_filter_``.
To automate the flow of this tutorial we create two timers.
The ``publisher_timer_`` and the ``query_timer_``.
The first one automates messages publishing.
The second one automates the monitoring of the state of the ``counter_filter_``.

.. code-block:: C++

      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::publisher_timer_callback, this)
      );

      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&DeltaNode::query_timer_callback, this)
      );

And this is it for the constructor of the ``DeltaNode``.
The ``DeltaNode`` also has two callbacks, that are executed by the timers.
Let's take a look at them.

.. code-block:: C++

    void query_timer_callback() {
      RCLCPP_INFO(
        get_logger(),
        "Messages passed down the chain count: %zu",
        counter_filter_.getCounterValue()
      );
    }

    void publisher_timer_callback() {
      auto message = std_msgs::msg::String();
      if (pub_count_ % 2 == 0) {
        message.data = "pub_count = " + std::to_string(pub_count_);
      } else {
        message.data = "pub_count = " + std::to_string(pub_count_ - 1);
      }
      publisher_->publish(message);
      ++pub_count_;
    }

As is already said, the ``query_timer_callback`` automates the ``counter_filter_`` state monitoring.
On the other hand, the ``publisher_timer_callback`` automates messages publishing.
To demonstrate how the ``DeltaFilter`` works, we need to publish messages that may or may not change.
In this case every ``even`` message is published with the actual number of published messages.
And every ``odd`` message is published with the previous value of this counter.
Hence, every ``even`` message passes the ``delta_filter_`` as it has the ``data`` different from previously published.
And every ``odd`` message has the ``data`` same as the previous message, so it does not pass the ``delta_filter_``.

The ``main`` function as usual in this tutorials is pretty straightforward.

.. code-block:: C++

  int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
  {
    rclcpp::init(argc, argv);

    auto delta_node = std::make_shared<DeltaNode>();

    rclcpp::spin(delta_node);
    rclcpp::shutdown();

    return 0;
  }

2. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>message_filters</depend>
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>

3. Add the Node to a CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` add the executable and name it ``delta_tutorial``, which you’ll use later with ``ros2 run``.

.. code-block:: CMake

	find_package(ament_cmake_auto REQUIRED)
	ament_auto_find_build_dependencies()

	ament_auto_add_executable(delta_tutorial src/delta_tutorial.cpp)

Finally, add the ``install(TARGETS…)`` section so ros2 run can find your executable:

.. code-block:: CMake

  install(TARGETS delta_tutorial
    DESTINATION lib/${PROJECT_NAME})

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

The output should look something like this:

.. code-block:: console

  [INFO] [1779744355.607542155] [DeltaNode]: Messages passed down the chain count: 0
  [INFO] [1779744356.607404257] [DeltaNode]: Messages passed down the chain count: 1
  [INFO] [1779744357.607475653] [DeltaNode]: Messages passed down the chain count: 1
  [INFO] [1779744358.607123073] [DeltaNode]: Messages passed down the chain count: 2
  [INFO] [1779744359.607084631] [DeltaNode]: Messages passed down the chain count: 2
  [INFO] [1779744360.607405810] [DeltaNode]: Messages passed down the chain count: 3
  [INFO] [1779744361.607395038] [DeltaNode]: Messages passed down the chain count: 3

Note that when the first query is executed, no message have reached the counter filter yet.
So that is indicated by first line of the console output.
At the time the next query is executed, the first message has reached the counter.
So the number of messages is now ``1``.
Then, when the third query is executed, the third message has been published,
but was ignored because the data in it was the same as in previous message.
And so on.

Both timers run at the same 1 s period.
Even though ``publisher_timer_`` is created (and therefore fires) before ``query_timer_``,
message delivery from publisher to subscriber is asynchronous, so on the first tick the published message
has not yet been routed through the filter chain by the time the query callback runs — which is why
the very first line reports ``0``.
From the second tick onward the prior message has been delivered, so the count advances as expected.

6. Other options provided by the DeltaFilter class                                                                                                                          
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want different comparison semantics than "any field differs", you can plug a custom comparison handler into ``ComparisonFilter``.
The recommended path is to inherit from ``CachedComparisonHandler`` — it already implements the per-field caching and iteration logic in ``message_fits``, and exposes the actual comparison as a pure-virtual hook.
In your subclass, override ``do_fields_fit``:

.. code-block:: C++

  template<typename MessageType>
  class MyCompare : public message_filters::CachedComparisonHandler<MessageType>
  {
  public:
    using FieldGetterFunctionType =
      typename message_filters::CachedComparisonHandler<MessageType>::FieldGetterFunctionType;

    explicit MyCompare(std::forward_list<FieldGetterFunctionType> field_getters)
    : message_filters::CachedComparisonHandler<MessageType>(field_getters) {}
    bool do_fields_fit(message_filters::MFieldType a, message_filters::MFieldType b) const override
    {
      // return true to accept the new message, false to drop it
      return a != b;
    }
  };

``do_fields_fit`` receives the value produced by one of the ``field_getters`` for the cached message (``a``) and for the incoming message (``b``).
Return ``true`` to accept the incoming message (it then becomes the new cached message and is passed down the chain); return ``false`` to drop it.
``DeltaCompare`` itself is just this pattern with ``return a != b;``.
You can then use your handler through ``ComparisonFilter``:

.. code-block:: C++

  message_filters::ComparisonFilter<std_msgs::msg::String, MyCompare> my_filter(
    subscriber_filter_,
    {[] (const MConstPtr & msg) {return msg->data;}}
  );