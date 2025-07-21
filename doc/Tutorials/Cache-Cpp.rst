Cache (C++):
------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``message_filters::Cache`` class in ROS 2 using C++.
The ``Cache`` filter stores a time history of messages and allows querying based on timestamps.

We will use ``std_msgs.msg.String`` message for clarity and simplicity.

Prerequisites
~~~~~~~~~~~~~
This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS package for C++.
The next step is to create a new C++ file inside your package, e.g., ``cache_tutorial.cpp``, and create an example node:

.. code-block:: C++

  #include <chrono>
  #include <functional>
  #include <memory>
  #include <string>

  #include "rclcpp/rclcpp.hpp"

  #include "message_filters/subscriber.hpp"
  #include "message_filters/cache.hpp"

  #include "std_msgs/msg/string.hpp"
	
  using namespace std::chrono_literals;
  
  const std::string TUTORIAL_TOPIC_NAME = "tutorial_topic";
  
  class CacheNode : public rclcpp::Node {
  public:
    CacheNode()
    : Node("cache_node")
    {
      auto qos = rclcpp::QoS(10);
      publisher_ = this->create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);
    
      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::publisher_timer_callback, this)
      );
    
      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::query_timer_callback, this)
      );
    }
  
    void publisher_timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "example string";
      publisher_->publish(message);
    }
  
    void query_timer_callback() {
      rclcpp::Time latest_time = cache_filter_.getLatestTime();
    
      if (latest_time == rclcpp::Time()) {
        RCLCPP_INFO(
          this->get_logger(), "Cache is empty"
        );
        return;
      }
    
      rclcpp::Time oldest_time = cache_filter_.getOldestTime();
    
      RCLCPP_INFO(
        this->get_logger(),
        "oldest_time: %f, latest_time: %f",
        latest_time.seconds(),
        oldest_time.seconds()
      );
    }
  
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;
  
    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;
    message_filters::Cache<std_msgs::msg::String> cache_filter_{subscriber_filter_, 10, true};
  };
  
  
  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    auto cache_node = std::make_shared<CacheNode>();
    rclcpp::spin(cache_node);
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
  #include <string>

  #include "rclcpp/rclcpp.hpp"

  #include "message_filters/subscriber.hpp"
  #include "message_filters/cache.hpp"

  #include "std_msgs/msg/string.hpp"

  using namespace std::chrono_literals;
  
We start by including ``chrono`` and ``functional`` headers.
The ``chrono`` header is required for the ``chrono_literals`` namespace, necessary for creating timers.
The ``functional`` header is also required to use ``std::bind`` function to bind timer callbacks to timers.
After that we include the ``rclcpp.hpp`` header that provides us with classes from ``rclcpp`` namespace.
To use filters in our code we need corresponding headers as well.
In this case we include ``subscriber.hpp`` and ``cache.hpp``.
And finally we add ``string.hpp`` to get access to ``String`` message class from the ROS standard messages library.

Next we define a tutorial class.
In this case it is the ``CacheNode`` class.
For starters, let's take a look at the ``private`` section of this class:

.. code-block:: C++

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;
  
    message_filters::Subscriber<std_msgs::msg::String> subscriber_filter_;
    message_filters::Cache<std_msgs::msg::String> cache_filter_{subscriber_filter_, 10, true};

To publish messages we will need a ``Publisher`` object.
To automate publishing messages and for querying the cache filter we add two timers, the ``publisher_timer_`` and the ``query_timer_`` respectively.
After all that we add two filters and chain them together.

We start with a ``Subscriber`` filter, that is going to be an entry point for the messages into our chain of filters.
After that we create a ``Cache`` filter object, which is going to store and create a history of messages.

Please note, that when the ``cache_filter_`` is created, the previous filter, the ``subscriber_filter_`` is passed as the first argument.
It is the way to chain these two filters together.
A message is going to pass from a topic, through ``subscriber_filter_`` into ``cache_filter_``, from which it will be processed.

The second argument of the ``Cache`` constructor is ``cache size``, the maximum allotted messages to be stored in the cache.
The last argument is the ``allow_headerless`` flag, which is required to cache the ``String`` messages, that do not have a ``Header`` field.
In this case, the time the message was received is used, but only in case if the message class does not have a ``Header``.
If there is one, then the time from ``Header`` is used.
If the ``allow_headerless`` flag is set to ``false``, it is impossible to use messages without header.

There is an option to directly put messages into ``Cache`` filter, by calling the ``add`` method of a ``cache`` filter.
There is, as well, an option to create an instance of the ``Cache`` filter on it's own, and pass messages to it via the ``add`` method.

Next let's take a look at timer callbacks.

.. code-block:: C++

  void publisher_timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "example string";
    publisher_->publish(message);
  }

  void query_timer_callback() {
    rclcpp::Time latest_time = cache_filter_.getLatestTime();

    if (latest_time == rclcpp::Time()) {
      RCLCPP_INFO(
        this->get_logger(), "Cache is empty"
      );
      return;
    }

    rclcpp::Time oldest_time = cache_filter_.getOldestTime();

    RCLCPP_INFO(
      this->get_logger(),
      "oldest_time: %f, latest_time: %f",
      latest_time.seconds(),
      oldest_time.seconds()
    );
  }

Now it is worthy to draw some attention to the following line of code.

.. code-block:: C++

	if (latest_time == rclcpp::Time())
	
Since we use the headerless ``String`` message in this tutorial, the time source for this message is the default ``RCL_SYSTEM_TIME``.
If we would use messages with headers, the expected time source for them would be the ``RCL_ROS_TIME``.

Finally, let's take a look at the class constructor.

.. code-block:: C++

    CacheNode()
    : Node("cache_node")
    {
      auto qos = rclcpp::QoS(10);
      publisher_ = this->create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);
      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);
    
      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::publisher_timer_callback, this)
      );
    
      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::query_timer_callback, this)
      );
    }
    
Here we create a ``publisher_``, that is going to publish messages to some topic.

.. code-block:: C++

      publisher_ = this->create_publisher<std_msgs::msg::String>(TUTORIAL_TOPIC_NAME, qos);

When the publisher is created we subscribe to a topic via ``subscriber_filter_``.

.. code-block:: C++

      subscriber_filter_.subscribe(this, TUTORIAL_TOPIC_NAME, qos);

After that all what's left to be done is to create timers and we are good to go.

.. code-block:: C++

      publisher_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::publisher_timer_callback, this)
      );
    
      query_timer_ = this->create_wall_timer(
        1s,
        std::bind(&CacheNode::query_timer_callback, this)
      );

The ``main`` function in this case is pretty straightforward.

.. code-block:: C++

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    auto cache_node = std::make_shared<CacheNode>();
    rclcpp::spin(cache_node);
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

Now open the ``CMakeLists.txt`` add the executable and name it ``cache_tutorial``, which you’ll use later with ``ros2 run``.

.. code-block:: CMake

	find_package(ament_cmake_auto REQUIRED)
	ament_auto_find_build_dependencies()

	ament_auto_add_executable(cache_tutorial src/cache_tutorial.cpp)

Finally, add the install(TARGETS…) section so ros2 run can find your executable:

.. code-block:: CMake

  install(TARGETS cache_tutorial
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

    ros2 run pkg_name cache_tutorial

The first message in the output is going to be

.. code-block:: console

	[INFO] [1752701571.845039452] [cache_node]: Cache is empty
	
As there were no messages published yet, and the cache is empty.
After that, the publisher will start populate the cache with messages:

.. code-block:: console

  [INFO] [1752701572.845232157] [cache_node]: oldest_time: 1752701571.846233, latest_time: 1752701571.846233
  [INFO] [1752701573.845208906] [cache_node]: oldest_time: 1752701572.846269, latest_time: 1752701571.846233
  [INFO] [1752701574.844841757] [cache_node]: oldest_time: 1752701573.846131, latest_time: 1752701571.846233
  [INFO] [1752701575.844989998] [cache_node]: oldest_time: 1752701574.845164, latest_time: 1752701571.846233
  [INFO] [1752701576.845013484] [cache_node]: oldest_time: 1752701575.845885, latest_time: 1752701571.846233
  [INFO] [1752701577.844898272] [cache_node]: oldest_time: 1752701576.845787, latest_time: 1752701571.846233
  [INFO] [1752701578.844905995] [cache_node]: oldest_time: 1752701577.845648, latest_time: 1752701571.846233
  [INFO] [1752701579.844954514] [cache_node]: oldest_time: 1752701578.845697, latest_time: 1752701571.846233
  [INFO] [1752701580.844988219] [cache_node]: oldest_time: 1752701579.845718, latest_time: 1752701571.846233
  [INFO] [1752701581.844955759] [cache_node]: oldest_time: 1752701580.845818, latest_time: 1752701571.846233
  [INFO] [1752701582.845005794] [cache_node]: oldest_time: 1752701581.845692, latest_time: 1752701572.846269  <-- drop old msgs
  [INFO] [1752701583.844966965] [cache_node]: oldest_time: 1752701582.845980, latest_time: 1752701573.846131
  [INFO] [1752701584.844954452] [cache_node]: oldest_time: 1752701583.845715, latest_time: 1752701574.845164

Note as the oldest time is starting to update after the 5'th message is added to the cache.
The cache size for the ``Cache`` in this example is 10. So as the 10'th message is added to
the cache, the oldest messages are being removed from it, thus updating oldest time.

6. Other methods of the Cache filter interface 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``Cache`` filter stores the last N messages (in this case, 5), and allows querying:

- Entire history: ``getInterval(const rclcpp::Time& start, const rclcpp::Time& end)``
- Oldest rclcpp::Time: ``getOldestTime()``
- Newest rclcpp::Time: ``getLatestTime()``
- Messages after a certain time: ``getElemAfterTime(const rclcpp::Time& time)``
- Messages before a certain time: ``getElemBeforeTime(const rclcpp::Time& time)``
- A vector of messages that occur between a start and end time (inclusive) ``getInterval(const rclcpp::Time & start, const rclcpp::Time & end)``
- The smallest interval of messages that surrounds an interval from start to end ``getSurroundingInterval(const rclcpp::Time & start, const rclcpp::Time & end)``
- Set new cache size. The actual cache size will change when new message is added ``setCacheSize``

This is especially useful when you need to look back in time (e.g., align with previous sensor data).