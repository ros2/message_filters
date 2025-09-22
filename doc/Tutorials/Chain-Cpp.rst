Chain (C++):
------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters::Chain`` class in ROS 2 using C++.
The ``Chain`` filter provides a container for simple filters.
It allows you to store an N-long set of filters inside a single structure, making it much easier to manage them.

To demonstrate the functionality of the ``Chain`` filter in a more clear manner, we are going to add a custom filter to this tutorial.
This is going to be the ``CounterFilter`` that will be counting the number of messages passing through it.
This filter class will be a successor to the ``SimpleFilter`` class, but this is a topic for another tutorial.

.. TODO: @EsipovPA: Add message_filters::SimpleFilter tutorial reference, when ready

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS package for C++.
The next step is to create a new C++ file inside your package, e.g., ``chain_tutorial.cpp``, and write an example code:

.. code-block:: C++

  #include <chrono>
  #include <cstddef>
  #include <functional>
  #include <memory>
  #include <string>

  #include <rclcpp/rclcpp.hpp>

  #include <message_filters/message_event.hpp>
  #include <message_filters/chain.hpp>
  #include <message_filters/subscriber.hpp>
  #include <message_filters/simple_filter.hpp>

  #include <std_msgs/msg/string.hpp>

  using namespace std::chrono_literals;

  const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";

  class CounterFilter : public message_filters::SimpleFilter<std_msgs::msg::String> {
  public:
    CounterFilter() : message_filters::SimpleFilter<std_msgs::msg::String>() {}

    void add(const message_filters::MessageEvent<std_msgs::msg::String>& e) {
      ++counter_;
      signalMessage(e);
    }

    template<class F>
    void connectInput(F& f) {
      incoming_connection_ = f.registerCallback(
        std::bind(
          &CounterFilter::callback,
          this,
          std::placeholders::_1
        )
      );
    }

    void counterCallback(const std_msgs::msg::String::ConstSharedPtr& _) {
      ++counter_;
    }

    size_t getCounterValue() {
      return counter_;
    }
  private:
    size_t counter_ = 0;

    message_filters::Connection incoming_connection_;

    void callback(const std_msgs::msg::String::ConstSharedPtr& evt) {
      add(evt);
    }
  };

  class ChainNode : public rclcpp::Node {
  public:
    ChainNode() :
      Node("ChainNode"),
      subscriber_filter_(),
      first_counter_(std::make_shared<CounterFilter>()),
      second_counter_(std::make_shared<CounterFilter>()),
      chain_filter_(subscriber_filter_)
    {
      auto qos = rclcpp::QoS(10);

      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);

      // Set up the chain of filters
      chain_filter_.addFilter(first_counter_);
      chain_filter_.addFilter(second_counter_);

      chain_filter_.registerCallback(
        std::bind(
          &ChainNode::chain_exit_callback,
          this,
          std::placeholders::_1
        )
      );

      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);

      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::publisher_timer_callback, this)
      );

      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::query_timer_callback, this)
      );
    }

    void chain_exit_callback(const std_msgs::msg::String::ConstSharedPtr& _) {
      RCLCPP_INFO(
        get_logger(),
        "%zu messages have reached the end of this chain",
        ++chain_counter_
      );
    }

    void publisher_timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "example string";
      publisher_->publish(message);
    }

    void query_timer_callback() {
      RCLCPP_INFO(
        get_logger(),
        "First counter messages count: %zu, Second counter messages count: %zu",
        chain_filter_.getFilter<CounterFilter>(0)->getCounterValue(),
        chain_filter_.getFilter<CounterFilter>(1)->getCounterValue()
      );
    }
  private:
    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;

    std::shared_ptr<CounterFilter> first_counter_;
    std::shared_ptr<CounterFilter> second_counter_;

    size_t chain_counter_ = 0;

    message_filters::Chain<std_msgs::msg::String> chain_filter_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;
  };

  int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
  {
    rclcpp::init(argc, argv);

    auto chain_node = std::make_shared<ChainNode>();

    rclcpp::spin(chain_node);
    rclcpp::shutdown();

    return 0;
  }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's break down this code and examine the details.

.. code-block:: C++

  #include <chrono>
  #include <cstddef>
  #include <functional>
  #include <memory>
  #include <string>

  #include <rclcpp/rclcpp.hpp>

  #include <message_filters/message_event.hpp>
  #include <message_filters/chain.hpp>
  #include <message_filters/subscriber.hpp>
  #include <message_filters/simple_filter.hpp>

  #include <std_msgs/msg/string.hpp>

  using namespace std::chrono_literals;

We start by including ``chrono`` and ``functional`` headers.
The ``chrono`` header is required for the ``chrono_literals`` namespace, necessary for creating timers.
The ``cstddef`` header provides us with some basic types such as ``size_t``.
The ``memory`` header is required as we are going to utilize ``shared_ptr`` class.
The ``functional`` header is also required to use ``std::bind`` function to bind timer callbacks to timers.
After that we include the ``rclcpp.hpp`` header that provides us with classes from ``rclcpp`` namespace.
For the ``CounterFilter`` we need a ``message_filters::MessageEvent`` class from the ``message_event.hpp``, so it is included.
To use filters in our code we need corresponding headers as well.
In this case we include ``subscriber.hpp``, ``chain.hpp`` and ``simple_filter.hpp``.
And finally we add ``string.hpp`` to get access to ``String`` message class from the ROS standard messages library.

Next we define a ``CounterFilter`` class.

.. code-block:: C++

  class CounterFilter : public message_filters::SimpleFilter<std_msgs::msg::String> {
  public:
    CounterFilter() : message_filters::SimpleFilter<std_msgs::msg::String>() {}
  
    void add(const message_filters::MessageEvent<std_msgs::msg::String>& e) {
      ++counter_;
      signalMessage(e);
    }
  
    template<class F>
    void connectInput(F& f) {
      incoming_connection_ = f.registerCallback(
        std::bind(
          &CounterFilter::callback,
          this,
          std::placeholders::_1
        )
      );
    }
  
    void counterCallback(const std_msgs::msg::String::ConstSharedPtr& _) {
      ++counter_;
    }
  
    size_t getCounterValue() {
      return counter_;
    }
  private:
    size_t counter_ = 0;
  
    message_filters::Connection incoming_connection_;
  
    void callback(const std_msgs::msg::String::ConstSharedPtr& evt) {
      add(evt);
    }
  };
  
This filter counts the number of messages passing through it.
The ``add`` method increases messages count, and passes message to the following filter via ``signalMessage``.
The ``connectInput`` connects this filter to a previous filter's output.
The ``getCounterValue`` grants access to the current messages count.

.. More on this succession mechanism should be in the corresponding tutorial
.. TODO: @EsipovPA Add link to the message_filters::SimpleFilter tutorial, when added.

And now we can turn our attention to the main tutorial class, that is the ``ChainNode`` class.
For starters, let's take a look at the ``private`` section of this class:

.. code-block:: C++

    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;

    std::shared_ptr<CounterFilter> first_counter_;
    std::shared_ptr<CounterFilter> second_counter_;

    message_filters::Chain<std_msgs::msg::String> chain_filter_;

    size_t chain_counter_ = 0;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;

We start with a ``Subscriber`` filter, that is going to be an entry point for the messages into our chain of filters.
After that we add two ``CounterFilter`` objects.
Next up is the ``Chain`` filter itself. It is going to connect all the filters.

When a message passes through all the filters in the chain, it is passed to a ``Chain`` filter's callback.
This callback is where ``chain_counter_`` is updated.
The ``chain_counter_`` field, as it says, keeps the count of messages, that have reached the end of the chain.

To publish messages we will need a ``Publisher`` object.
To automate publishing messages and for querying the chain filter we add two timers, the ``publisher_timer_`` and the ``query_timer_`` respectively.

Now let's take a look at the ``ChainNode`` constructor

.. code-block:: C++

    ChainNode() :
      Node("ChainNode"),
      subscriber_filter_(),
      first_counter_(std::make_shared<CounterFilter>()),
      second_counter_(std::make_shared<CounterFilter>()),
      chain_filter_(subscriber_filter_)
    {
      auto qos = rclcpp::QoS(10);
  
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);
  
      // Set up the chain of filters
      chain_filter_.addFilter(first_counter_);
      chain_filter_.addFilter(second_counter_);
  
      chain_filter_.registerCallback(
        std::bind(
          &ChainNode::chain_exit_callback,
          this,
          std::placeholders::_1
        )
      );
  
      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);
      
      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::publisher_timer_callback, this)
      );
  
      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::query_timer_callback, this)
      );
    }

We start with constructing the basic ``Node`` class and all the filters required in this example.

.. code-block:: C++

      Node("ChainNode"),
      subscriber_filter_(),
      first_counter_(std::make_shared<CounterFilter>()),
      second_counter_(std::make_shared<CounterFilter>()),
      chain_filter_(subscriber_filter_)

After that we subscrie to the example topic via ``subscriber_filter_``.
Following it we create ``first_counter_`` and ``second_counter_`` ``CounterFilte`` instances.
Finally we create an instance of the ``Chain`` filter, that is stored in the ``chain_filter_`` field.
Please note that the ``subscriber_filter_`` is passed as a constructor argument.
This helps to set this filter as a first in a chain.
The other wat to do so is to call ``connectInput`` method after the ``chain_filter_`` is constructed.
Like so 

.. code-block:: C++

    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_();
    message_filters::Chain<std_msgs::msg::String> chain_filter_();
    chain_filter_.connectInput(subscriber_filter_);

The body of the constructor starts with initializing the ``subscriber_filter_``'s subscription.

.. code-block:: C++

      auto qos = rclcpp::QoS(10);
  
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);

The next step is to chain together two instances of a ``CounterFilter``.
We add both of this filters to the ``Chain`` filter via ``addFilter`` method call.

.. code-block:: C++

      // Set up the chain of filters
      chain_filter_.addFilter(first_counter_);
      chain_filter_.addFilter(second_counter_);

The last thing to do with the ``Chain`` filter is to add a callback.
This callback is called every time, when a message passes through all other filters in the chain.

.. code-block:: C++

      chain_filter_.registerCallback(
        std::bind(
          &ChainNode::chain_exit_callback,
          this,
          std::placeholders::_1
        )
      );

Now, when all filters are set up, we need to publish some messages to the example topic.
For this purpose we set up a publisher and a publish timer, and a query timer to see the results.

.. code-block:: C++

      publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);
      
      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::publisher_timer_callback, this)
      );

      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&ChainNode::query_timer_callback, this)
      );

Please notice the ``query_timer_callback``.
It demonstrates one of the ``Chain`` filter main methods.
The ``getFilter`` template method provides an access to any filter in the chain, given it's position.
The return value of this method is a shared pointer to the required filter.

.. code-block:: C++

    void query_timer_callback() {
      RCLCPP_INFO(
        get_logger(),
        "First counter messages count: %zu, Second counter messages count: %zu",
        chain_filter_.getFilter<CounterFilter>(0)->getCounterValue(),
        chain_filter_.getFilter<CounterFilter>(1)->getCounterValue()
      );


The ``main`` function as usual in this tutorials is pretty straightforward.

.. code-block:: C++

  int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
  {
    rclcpp::init(argc, argv);
  
    auto chain_node = std::make_shared<ChainNode>();
  
    rclcpp::spin(chain_node);
    rclcpp::shutdown();
  
    return 0;
  }

2. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>rclcpp</depend>
    <depend>message_filters</depend>
    <depend>std_msgs</depend>

3. Add the Node to a CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now open the ``CMakeLists.txt`` add the executable and name it ``chain_tutorial``, which you’ll use later with ``ros2 run``.

.. code-block:: CMake

	find_package(ament_cmake_auto REQUIRED)
	ament_auto_find_build_dependencies()

	ament_auto_add_executable(chain_tutorial src/chain_tutorial.cpp)

Finally, add the install(TARGETS…) section so ros2 run can find your executable:

.. code-block:: CMake

  install(TARGETS chain_tutorial
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

    ros2 run chain_tutorial chain_tutorial

The output is going to look something like this

.. code-block:: console

    [INFO] [1758299415.877982943] [ChainNode]: First counter messages count: 0, Second counter messages count: 0
    [INFO] [1758299415.879089260] [ChainNode]: 1 messages have reached the end of this chain
    [INFO] [1758299416.877867549] [ChainNode]: First counter messages count: 1, Second counter messages count: 1
    [INFO] [1758299416.878531764] [ChainNode]: 2 messages have reached the end of this chain
    [INFO] [1758299417.877748421] [ChainNode]: First counter messages count: 2, Second counter messages count: 2
    [INFO] [1758299417.877956617] [ChainNode]: 3 messages have reached the end of this chain
    [INFO] [1758299418.877758202] [ChainNode]: First counter messages count: 3, Second counter messages count: 3
    [INFO] [1758299418.878093589] [ChainNode]: 4 messages have reached the end of this chain
    [INFO] [1758299419.877953871] [ChainNode]: First counter messages count: 4, Second counter messages count: 4
    [INFO] [1758299419.878610243] [ChainNode]: 5 messages have reached the end of this chain
    [INFO] [1758299420.878002318] [ChainNode]: First counter messages count: 5, Second counter messages count: 5
    [INFO] [1758299420.878685845] [ChainNode]: 6 messages have reached the end of this chain
    [INFO] [1758299421.877966429] [ChainNode]: First counter messages count: 6, Second counter messages count: 6
    [INFO] [1758299421.878590495] [ChainNode]: 7 messages have reached the end of this chain
    [INFO] [1758299422.877695774] [ChainNode]: First counter messages count: 7, Second counter messages count: 7
    [INFO] [1758299422.878073509] [ChainNode]: 8 messages have reached the end of this chain
    [INFO] [1758299423.877839197] [ChainNode]: First counter messages count: 8, Second counter messages count: 8
    [INFO] [1758299423.878292931] [ChainNode]: 9 messages have reached the end of this chain

Note that when the first query to the both counter filters executed, they both report that there was no messages, passing through before.

.. code-block:: console

    [INFO] [1758299415.877982943] [ChainNode]: First counter messages count: 0, Second counter messages count: 0

After that, as we can see from the output of the ``chain_exit_callback``, the first message passes through the chain.
The next time we request the number of messages, every counter filter reports that one message has passed through.

.. code-block:: console

    [INFO] [1758299415.879089260] [ChainNode]: 1 messages have reached the end of this chain
    [INFO] [1758299416.877867549] [ChainNode]: First counter messages count: 1, Second counter messages count: 1

From this point on, all three counters increase their values as more messages are passed down the filter chain.

6. Other methods of the Chain filter interface 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this example we've passed the ``subscriber_filter_`` object to the ``chain_filter_`` as a constructor argument.
In this case, the ``subscriber_filter_`` was used as an input filter for the ``chain_filter_``.

.. code-block:: C++

    chain_filter_(subscriber_filter_)

The other way to do it is by calling ``connectInput`` method of the ``Chain`` class.

.. code-block:: C++

    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_();
    message_filters::Chain<std_msgs::msg::String> chain_filter_();
    chain_filter_.connectInput(subscriber_filter_);