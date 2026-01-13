SimpleFilter (C++):
-------------------

Overview
~~~~~~~~

This tutorial demonstrates how to create a custom filter that is going to be a successor to the ``SimpleFilter`` class.
The ``SimpleFilter`` is a base class for almost all of message filters implemented in ``C++``.
It provides the basic functionality for building filters.
To demonstrate the functionality of this filter we are going to create a ``CounterWithLastMessageCache`` filter class.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node
~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for ``C++``.
The next step is to create a new C++ file inside your package, e.g., ``simple_filter_tutorial.cpp``, and write an example code:

.. code-block:: C++

    #include <chrono>
    #include <cstddef>
    #include <string>

    #include <rclcpp/rclcpp.hpp>

    #include <message_filters/simple_filter.hpp>
    #include <message_filters/subscriber.hpp>
    #include <message_filters/connection.hpp>

    #include <std_msgs/msg/string.hpp>

    using namespace std::chrono_literals;

    const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";

    namespace message_filters
    {
    template<class M>
    class CounterWithLastMessageCache : public SimpleFilter<M> {
      typedef typename SimpleFilter<M>::MConstPtr MConstPtr;
      typedef typename SimpleFilter<M>::Callback Callback;
      typedef MessageEvent<M const> EventType;

    public:
      virtual ~CounterWithLastMessageCache() {}

      CounterWithLastMessageCache() {}

      template<typename F>
      explicit CounterWithLastMessageCache(F & filter)
      {
        connectInput(filter);
      }

      template<class F>
      void connectInput(F & filter)
      {
        incoming_connection_.disconnect();
        incoming_connection_ = filter.registerCallback(
          typename SimpleFilter<M>::EventCallback(
            std::bind(
              &CounterWithLastMessageCache::add,
              this,
              std::placeholders::_1
            )
          )
        );
      }

      size_t getCounter() const {
        return counter_;
      }

      const MConstPtr getLastMsgCache() const {
        return last_msg_cache_;
      }

      void add(const EventType & evt)
      {
        counter_ += 1;
        last_msg_cache_ = evt.getMessage();

        signalMessage(evt);
      }

    private:
      MConstPtr last_msg_cache_;
      size_t counter_ = 0;

      Connection incoming_connection_;
    };

    }

    class SimpleFilterExampleNode : public rclcpp::Node {
    public:
      SimpleFilterExampleNode() :
        Node("SimpleFilterExampleNode"),
        subscriber_filter_(),
        counter_filter_(subscriber_filter_)
      {
        auto qos = rclcpp::QoS(10);

        subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);

        publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);

        publisher_timer_ = create_wall_timer(
          1s,
          std::bind(&SimpleFilterExampleNode::publisher_timer_callback, this)
        );

        query_timer_ = create_wall_timer(
          1s,
          std::bind(&SimpleFilterExampleNode::query_timer_callback, this)
        );
      }

      void publisher_timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Pub count: " + std::to_string(++pub_counter_);
        publisher_->publish(message);
      }

      void query_timer_callback() {
        if (counter_filter_.getCounter() != 0) {
          RCLCPP_INFO(
            get_logger(),
            "Published messages count: %zu. Last message: %s",
            counter_filter_.getCounter(),
            counter_filter_.getLastMsgCache()->data.c_str()
          );
        } else {
          RCLCPP_INFO(
            get_logger(),
            "No messages published yet"
          );
        }
      }

    private:
      message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;
      message_filters::CounterWithLastMessageCache<std_msgs::msg::String> counter_filter_;

      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

      rclcpp::TimerBase::SharedPtr publisher_timer_;
      rclcpp::TimerBase::SharedPtr query_timer_;

      std::size_t pub_counter_ = 0;
    };

    int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
    {
      rclcpp::init(argc, argv);

      auto example_node = std::make_shared<SimpleFilterExampleNode>();

      rclcpp::spin(example_node);
      rclcpp::shutdown();

      return 0;
    }

1.1 Examine the code
~~~~~~~~~~~~~~~~~~~~

Now, let's break down this code and examine the details.

.. code-block:: C++

    #include <chrono>
    #include <cstddef>
    #include <string>

    #include <rclcpp/rclcpp.hpp>

    #include <message_filters/simple_filter.hpp>
    #include <message_filters/subscriber.hpp>
    #include <message_filters/connection.hpp>

    #include <std_msgs/msg/string.hpp>

    using namespace std::chrono_literals;

    const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";

We start by including ``C++`` standard library headers such as ``chrono``, ``cstddef`` and ``string``.
The ``chrono`` header is required for the ``chrono_literals`` namespace, necessary for creating timers.
The ``cstddef`` header provides us with some basic types such as ``size_t``.
The ``string`` header gives us access to the ``std::string`` class and ``std::to_string`` function.
After that we include the ``rclcpp.hpp`` header that provides us with classes from ``rclcpp`` namespace.
To use message filters and some other classes from ``message_filters`` library we need to include corresponding headers.
In this case we include ``simple_filter.hpp``, ``subscriber.hpp`` and ``connection.hpp``.

Next we defne ``CounterWithLastMessageCache`` and make it a part of the ``message_filters`` namespace. 

.. code-block:: C++

    namespace message_filters
    {
    template<class M>
    class CounterWithLastMessageCache : public SimpleFilter<M> {
      typedef typename SimpleFilter<M>::MConstPtr MConstPtr;
      typedef typename SimpleFilter<M>::Callback Callback;
      typedef MessageEvent<M const> EventType;

We start with a few ``typedef declarations`` for ``MConstPtr``, ``Callback`` and ``EventType`` to make the code cleaner.
Now let's take a look at the ``public`` section of the class.

.. code-block:: C++

      public:
      virtual ~CounterWithLastMessageCache() {}

      CounterWithLastMessageCache() {}

      template<typename F>
      explicit CounterWithLastMessageCache(F & filter)
      {
      connectInput(filter);
      }
``
It starts with a default destructor and constructor for the class, and one constructor that receives a reference to another filter.
The latter gives an option to create an instance of the ``CounterWithLastMessageCache`` filter that is already connected to another filter's output.
Following the last constructor is the ``connectInput`` method, which removes the previous connection with another filter, if there was any.
This is done by the ``incoming_connection_.disconnect()`` call.
This way, another filter, if there was any, stops passing messages to this filter.
Any previously registered callbacks are no longer executed.

.. code-block:: C++

      template<class F>
      void connectInput(F & filter)
      {
        incoming_connection_.disconnect();
        incoming_connection_ = filter.registerCallback(
          typename SimpleFilter<M>::EventCallback(
            std::bind(
              &CounterWithLastMessageCache::add,
              this,
              std::placeholders::_1
            )
          )
        );
      }

To create a connection with a parent class filter, ``connectInput`` registers the ``add`` method of this class as a callback with another filter.
Thus, when a message arrives to this parent filter and calls the ``signalMessage`` method,
a message is passed to this ``CounterWithLastMessageCache`` filter's ``add`` method.
And from there, to other filters if there are any.

The ``signalMessage`` method is of the base ``SimpleFilter`` class.
Every instance of the ``SimpleFilter`` class may have a collection of callbacks.
A callback may be added to this collection via ``registerCallback`` call.
When the ``signalMessage`` method of a ``SimpleFilter`` class is called,
every callback from this collection is executed.
So it is important to note that for your filter to be able pass messages to other filters,
it has to call the ``signalMessage`` method at some point in it's workflow.

Now let's turn to the public interface of the class.
The ``getCounter`` method returns the current value of the counter.
The ``getLastMsgCache`` method provides access to the last message, that has passed through this filter.
And the ``add`` method does all the message processing work.
It increases the ``counter_`` for every passing method, as well as it stores the last message data to the ``last_message_cache_``.

.. code-block:: C++

      size_t getCounter() const {
        return counter_;
      }

      const MConstPtr getLastMsgCache() const {
        return last_msg_cache_;
      }

      void add(const EventType & evt)
      {
        counter_ += 1;
        last_msg_cache_ = evt.getMessage();

        signalMessage(evt);
      }

The private section in this case is rather simple.
It consists of the ``counter_`` and the ``last_message_cache_`` fields and the ``incoming_connection_`` field.
First two are the part of the business logic of this class.
The last one manages the connection with a filter that passes messages to this one.

.. code-block:: C++

    private:
      size_t counter_ = 0;
      MConstPtr last_msg_cache_;

      Connection incoming_connection_;
    };

    }

This is it for the ``CounterWithLastMessageCache`` class.
Now let's take a look at the ``SimpleFilterExampleNode``.
The public interface of the node consists of three methods.
The node's constructor, the ``publisher_timer_callback`` and the ``query_timer_callback``.

.. code-block:: C++ 

  class SimpleFilterExampleNode : public rclcpp::Node {
    public:
      SimpleFilterExampleNode() :
        Node("SimpleFilterExampleNode"),
        subscriber_filter_(),
        counter_filter_(subscriber_filter_)
      {
        auto qos = rclcpp::QoS(10);

        subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);

        publisher_ = create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);

        publisher_timer_ = create_wall_timer(
          1s,
          std::bind(&SimpleFilterExampleNode::publisher_timer_callback, this)
        );

        query_timer_ = create_wall_timer(
          1s,
          std::bind(&SimpleFilterExampleNode::query_timer_callback, this)
        );
      }

      void publisher_timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Pub count: " + std::to_string(++pub_counter_);
        publisher_->publish(message);
      }

      void query_timer_callback() {
        if (counter_filter_.getCounter() != 0) {
          RCLCPP_INFO(
            get_logger(),
            "Published messages count: %zu. Last message: %s",
            counter_filter_.getCounter(),
            counter_filter_.getLastMsgCache()->data.c_str()
          );
        } else {
          RCLCPP_INFO(
            get_logger(),
            "No messages published yet"
          );
        }
      }

The constructor initializes the node itself, and the two filters.
The instance of the ``SubscriberFilter`` receives messages from the ``TUTORIAL_TOPIC``.
The instance of the ``CounterWithLastMessageCache`` filter is immediately connected to the ``subscriber_filter_``'s output.
The ``subscriber_filter_`` subscribes to the ``TUTORIAL_TOPIC``.
After that the ``publisher_`` is created to populate the topic with messages.

Two timers are added to automate the work:
The ``publisher_timer_`` automates message publishing.
The ``query_timer_`` automates the introspection of the node filter's state.

The ``private`` section of the node holds the declarations of all the fields, required for the work, described above.

.. code-block:: C++

    private:
      message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;
      message_filters::CounterWithLastMessageCache<std_msgs::msg::String> counter_filter_;

      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

      rclcpp::TimerBase::SharedPtr publisher_timer_;
      rclcpp::TimerBase::SharedPtr query_timer_;

      std::size_t pub_counter_ = 0;
    };

The main section is rather simple in this example.

.. code-block:: C++

    int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
    {
      rclcpp::init(argc, argv);

      auto example_node = std::make_shared<SimpleFilterExampleNode>();

      rclcpp::spin(example_node);
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

Now open the ``CMakeLists.txt`` add the executable and name it ``simple_filter_tutorial``, which you’ll use later with ``ros2 run``.

.. code-block:: CMake

	find_package(ament_cmake_auto REQUIRED)
	ament_auto_find_build_dependencies()

	ament_auto_add_executable(simple_filter_tutorial src/simple_filter_tutorial.cpp)

Finally, add the ``install(TARGETS…)`` section so ros2 run can find your executable:

.. code-block:: CMake

  install(TARGETS simple_filter_tutorial
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

    ros2 run simple_filter_tutorial simple_filter_tutorial

The output is going to look something like this

.. code-block:: console

  [INFO] [1766958211.195602859] [SimpleFilterExampleNode]: No messages published yet
  [INFO] [1766958212.195809044] [SimpleFilterExampleNode]: Published messages count: 1. Last message: Pub count: 1
  [INFO] [1766958213.195618995] [SimpleFilterExampleNode]: Published messages count: 2. Last message: Pub count: 2
  [INFO] [1766958214.195599466] [SimpleFilterExampleNode]: Published messages count: 3. Last message: Pub count: 3
  [INFO] [1766958215.195890964] [SimpleFilterExampleNode]: Published messages count: 4. Last message: Pub count: 4
  [INFO] [1766958216.195910443] [SimpleFilterExampleNode]: Published messages count: 5. Last message: Pub count: 5
  [INFO] [1766958217.195906785] [SimpleFilterExampleNode]: Published messages count: 6. Last message: Pub count: 6
  [INFO] [1766958218.195652168] [SimpleFilterExampleNode]: Published messages count: 7. Last message: Pub count: 7

Note that when the first query is executed, there were no messages that have passed the filter, as is indicated by the console output.
After that we see that the count of the messages starts to increase.

.. code-block:: console

  ... Published messages count: 1. ...
  ... Published messages count: 2. ...
  ... Published messages count: 3. ...
  ... Published messages count: 4. ...
  ... Published messages count: 5. ...
  ... Published messages count: 6. ...
  ... Published messages count: 7. ...

as well as the last message cache starts to update

.. code-block:: console

  ... Last message: Pub count: 1
  ... Last message: Pub count: 2
  ... Last message: Pub count: 3
  ... Last message: Pub count: 4
  ... Last message: Pub count: 5
  ... Last message: Pub count: 6
  ... Last message: Pub count: 7

