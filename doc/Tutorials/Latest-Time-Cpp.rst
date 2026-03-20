LatestTime Synchronizer (C++):
------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to use the ``Synchronizer`` filter with the ``LatestTime`` synchronization policy.

To demonstrate this policy we will implement a node that will be publishing messages to
three different topics with different time intervals.
At the same time this node will create subscriptions to these topics and synchronize
messages coming from these topics using ``LatestTime`` policy.

We are going to use ``std_msgs::msg::String`` message to make this tutorial clearer.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_

1. Create a Basic Node with Includes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's assume, you've already created an empty ROS 2 package for C++.
The next step is to create a new C++ file inside your package, e.g., ``latest_time_tutorial.cpp``, and create an example node:

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"

    #include <std_msgs/msg/string.hpp>

    #include "message_filters/subscriber.hpp"
    #include "message_filters/synchronizer.hpp"
    #include "message_filters/sync_policies/latest_time.hpp"

    using namespace std::chrono_literals;

    class LatestTimeExampleNode: public rclcpp::Node {
    public:

        typedef std_msgs::msg::String StringMsg;
        typedef message_filters::sync_policies::LatestTime<
            StringMsg,
            StringMsg,
            StringMsg
        > SyncPolicy;

        LatestTimeExampleNode(): Node("latest_time_example_node") {
            rclcpp::QoS qos = rclcpp::QoS(10);

            fast_publisher = this->create_publisher<StringMsg>("fast_topic", qos);
            slow_publisher = this->create_publisher<StringMsg>("slow_topic", qos);
            very_slow_publisher = this->create_publisher<StringMsg>("very_slow_topic", qos);

            fast_sub.subscribe(this, "fast_topic", qos);
            slow_sub.subscribe(this, "slow_topic", qos);
            very_slow_sub.subscribe(this, "very_slow_topic", qos);

            fast_timer = this->create_wall_timer(500ms, std::bind(&LatestTimeExampleNode::fast_timer_callback, this));
            slow_timer = this->create_wall_timer(1000ms, std::bind(&LatestTimeExampleNode::slow_timer_callback, this));
            very_slow_timer = this->create_wall_timer(2000ms, std::bind(&LatestTimeExampleNode::very_slow_timer_callback, this));

            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>> (
                SyncPolicy(),
                fast_sub,
                slow_sub,
                very_slow_sub
            );

            sync->registerCallback(
                std::bind(
                &LatestTimeExampleNode::sync_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3
                )
            );
        }

    private:
        void sync_callback(
            const StringMsg::ConstSharedPtr & msg_1,
            const StringMsg::ConstSharedPtr & msg_2,
            const StringMsg::ConstSharedPtr & msg_3
        ) {
            RCLCPP_INFO(
                this->get_logger(),
                "Sync callback. Msgs: %s, %s, %s",
                msg_1->data.c_str(),
                msg_2->data.c_str(),
                msg_3->data.c_str()
            );
        }

        void fast_timer_callback() {
            fast_publisher->publish(
                get_message(
                    "Fast msg. Count: " + std::to_string(++fast_counter)
                )
            );
        }

        void slow_timer_callback() {
            slow_publisher->publish(
                get_message(
                    "Slow msg. Count: " + std::to_string(++slow_counter)
                )
            );
        }

        void very_slow_timer_callback() {
            very_slow_publisher->publish(
                get_message(
                    "Very slow msg. Count: " + std::to_string(++very_slow_counter)
                )
            );
        }

        StringMsg get_message(const std::string& msg_data) {
            auto message = StringMsg();
            message.data = msg_data;
            return message;
        }

    private:
        rclcpp::Publisher<StringMsg>::SharedPtr fast_publisher;
        rclcpp::Publisher<StringMsg>::SharedPtr slow_publisher;
        rclcpp::Publisher<StringMsg>::SharedPtr very_slow_publisher;

        message_filters::Subscriber<StringMsg> fast_sub;
        message_filters::Subscriber<StringMsg> slow_sub;
        message_filters::Subscriber<StringMsg> very_slow_sub;

        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

        rclcpp::TimerBase::SharedPtr fast_timer;
        rclcpp::TimerBase::SharedPtr slow_timer;
        rclcpp::TimerBase::SharedPtr very_slow_timer;

        size_t fast_counter { 0 };
        size_t slow_counter { 0 };
        size_t very_slow_counter { 0 };
    };

    int main(int argc, char ** argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<LatestTimeExampleNode>();
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
    #include <string>

    #include "rclcpp/rclcpp.hpp"

    #include <std_msgs/msg/string.hpp>

    #include "message_filters/subscriber.hpp"
    #include "message_filters/synchronizer.hpp"
    #include "message_filters/sync_policies/latest_time.hpp"

    using namespace std::chrono_literals;

We start by including ``chrono`` and ``functional`` headers.
The ``chrono`` header is required for the ``chrono_literals`` namespace, necessary for creating timers.
The ``functional`` header is also required to use ``std::bind`` function to bind timer callbacks to timers.
And the ``memory`` header provides us with ``std::shared_ptr`` class and ``std::make_shared`` function template.
Also we are going to need the ``std::string`` class, so we add that via including ``string`` header.
After that we include the ``rclcpp.hpp`` header that provides us with classes from ``rclcpp`` namespace.
To use filters in our code we need corresponding headers as well.
Then we add ``string.hpp`` to get access to ``String`` message class from the ROS standard messages library.
The last includes are ``subscriber.hpp``, ``synchronizer.hpp`` and ``latest_time.hpp``.
The first will provide us with ``Subscriber`` and ``Synchronizer`` filters.
The last one will give us the access to the ``LatestTime`` sync policy.

Next we define a ``LatestTimeExampleNode`` class.
First of all let's take a look at the type definitions that will be useful for making our code clearer.

.. code-block:: C++

        typedef std_msgs::msg::String StringMsg;
        typedef message_filters::sync_policies::LatestTime<
            StringMsg,
            StringMsg,
            StringMsg
        > SyncPolicy;

Here we define an alias for the ``std_msgs::msg::String`` message class that will be the ``StringMsg``.
Next we define an alias for the synchronization policy.
We will address it as ``SyncPolicy`` from now on.

Now let's take a look at the class members declared in the ``private`` section of this class:

.. code-block:: C++

    private:
        rclcpp::Publisher<StringMsg>::SharedPtr fast_publisher;
        rclcpp::Publisher<StringMsg>::SharedPtr slow_publisher;
        rclcpp::Publisher<StringMsg>::SharedPtr very_slow_publisher;

        message_filters::Subscriber<StringMsg> fast_sub;
        message_filters::Subscriber<StringMsg> slow_sub;
        message_filters::Subscriber<StringMsg> very_slow_sub;

        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

        rclcpp::TimerBase::SharedPtr fast_timer;
        rclcpp::TimerBase::SharedPtr slow_timer;
        rclcpp::TimerBase::SharedPtr very_slow_timer;

        size_t fast_counter { 0 };
        size_t slow_counter { 0 };
        size_t very_slow_counter { 0 };

To publish messages we will need three ``Publisher`` objects.
These will be the ``fast_publisher``, the ``slow_publisher`` and the ``very_slow_publisher``.
All of these will work with ``StringMsg`` messages.
To subscribe to these topics we are going to need a few ``Subscriber`` filter objects as well.
The ``fast_sub`` the ``slow_sub`` and the ``very_slow_sub`` will play this role.
Next we declare a ``Synchronizer`` filter object that will be using ``SyncPolicy``.
And to automate the publishing process we add three timers.
That will be the ``fast_timer``, the ``slow_timer``, and the ``very_slow_timer``.
The last group of objects is the group of counters.
The ``fast_counter``, the ``slow_counter``, and the ``very_slow_counter``.
These will be useful to add some context to the final output.

Nest let's take a look at the ``LatestTimeExampleNode`` class constructor.

.. code-block:: C++

        LatestTimeExampleNode(): Node("latest_time_example_node") {
            rclcpp::QoS qos = rclcpp::QoS(10);

            fast_publisher = this->create_publisher<StringMsg>("fast_topic", qos);
            slow_publisher = this->create_publisher<StringMsg>("slow_topic", qos);
            very_slow_publisher = this->create_publisher<StringMsg>("very_slow_topic", qos);

            fast_sub.subscribe(this, "fast_topic", qos);
            slow_sub.subscribe(this, "slow_topic", qos);
            very_slow_sub.subscribe(this, "very_slow_topic", qos);

            fast_timer = this->create_wall_timer(500ms, std::bind(&LatestTimeExampleNode::fast_timer_callback, this));
            slow_timer = this->create_wall_timer(1000ms, std::bind(&LatestTimeExampleNode::slow_timer_callback, this));
            very_slow_timer = this->create_wall_timer(2000ms, std::bind(&LatestTimeExampleNode::very_slow_timer_callback, this));

            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>> (
                SyncPolicy(),
                fast_sub,
                slow_sub,
                very_slow_sub
            );

            sync->registerCallback(
                std::bind(
                &LatestTimeExampleNode::sync_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3
                )
            );
        }

We start the class construction by creating the publishers.
The ``fast_publisher``, the ``slow_publisher`` and the ``very_slow_publisher``.
Then the subscriptions as and entrypoint for the data flow.
The ``fast_sub`` the ``slow_sub`` and the ``very_slow_sub``.
For automation we add the ``fast_timer``, the ``slow_timer``, and the ``very_slow_timer``.
Now it is time for the main purpose of this tutorial.
The ``Synchronizer`` filter with ``LatestTime`` synchronization policy.
Finally we register a callback that will be invoked when any message comes into the filter.
This will be the ``sync_callback`` that is declared later.
The final part of this class that have not been addressed yet is the callbacks sections.
Let's take a look at them as well.

.. code-block:: C++

        void sync_callback(
            const StringMsg::ConstSharedPtr & msg_1,
            const StringMsg::ConstSharedPtr & msg_2,
            const StringMsg::ConstSharedPtr & msg_3
        ) {
            RCLCPP_INFO(
                this->get_logger(),
                "Sync callback. Msgs: %s, %s, %s",
                msg_1->data.c_str(),
                msg_2->data.c_str(),
                msg_3->data.c_str()
            );
        }

        void fast_timer_callback() {
            fast_publisher->publish(
                get_message(
                    "Fast msg. Count: " + std::to_string(++fast_counter)
                )
            );
        }

        void slow_timer_callback() {
            slow_publisher->publish(
                get_message(
                    "Slow msg. Count: " + std::to_string(++slow_counter)
                )
            );
        }

        void very_slow_timer_callback() {
            very_slow_publisher->publish(
                get_message(
                    "Very slow msg. Count: " + std::to_string(++very_slow_counter)
                )
            );
        }

        StringMsg get_message(const std::string& msg_data) {
            auto message = StringMsg();
            message.data = msg_data;
            return message;
        }

The synchronization callback, the ``sync_callback`` outputs the contents of the messages.
Following are the timer callbacks.
The ``fast_timer_callback``, the ``slow_timer_callback``, and the ``very_slow_timer_callback``.
Every callback generates a message that tells the subscriber which publisher it comes from.
Finally there is the ``get_message`` function that just creates a message object
with the contents specified by caller.
It is added mainly to avoid some of the code duplication.

To run the node and all the publishers, subscribers, timers and a filter we add the ``main`` function.

.. code-block:: C++

    int main(int argc, char ** argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<LatestTimeExampleNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();

        return 0;
    }

2. LatestTime synchronization policy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``Synchronizer`` class provides an option to use different policies
to synchronize messages from multiple topics.
The ``LatestTime`` policy synchronizes up to N incoming channels by the rates they are received.
The callback with all the messages will be triggered whenever the fastest message is received.
The slower messages will be repeated at the rate of the fastest message and will be updated
whenever a new one is received.

3. Update package.xml
~~~~~~~~~~~~~~~~~~~~~

Navigate to your package root and add the following dependencies in ``package.xml``:

.. code-block:: xml

    <depend>message_filters</depend>
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>

4. Add the Node to a CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Now open the ``CMakeLists.txt`` add the executable and name it ``latest_time_tutorial``, which you'll use later with ``ros2 run``.

.. code-block:: CMake

   find_package(ament_cmake_auto REQUIRED)
   ament_auto_find_build_dependencies()

   add_executable(latest_time_tutorial src/latest_time_tutorial.cpp)

Finally, add the ``install(TARGETS…)`` section so ``ros2 run`` can find your executable:

.. code-block:: CMake

    install(TARGETS
        latest_time_tutorial
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

6. Run the Node
~~~~~~~~~~~~~~~

Now run the node using:

.. code-block:: console

    ros2 run latest_time_tutorial latest_time_tutorial

The console output should look something like this:

.. code-block:: console

    [INFO] [1773694323.514600409] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 4, Slow msg. Count: 2, Very slow msg. Count: 1
    [INFO] [1773694324.014240647] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 5, Slow msg. Count: 2, Very slow msg. Count: 1
    [INFO] [1773694324.514252533] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 6, Slow msg. Count: 3, Very slow msg. Count: 1
    [INFO] [1773694325.014487832] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 7, Slow msg. Count: 3, Very slow msg. Count: 1
    [INFO] [1773694325.514573661] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 8, Slow msg. Count: 4, Very slow msg. Count: 2
    [INFO] [1773694326.014433118] [latest_time_example_node]: Sync callback. Msgs: Fast msg. Count: 9, Slow msg. Count: 4, Very slow msg. Count: 2

Note that on each line the ``Fast msg`` counter increases it's value by 1.
At the same time, the ``Slow msg`` counter increments once for every two lines.
And it takes four lines to increase ``Very slow msg`` counter by one.
This means that it takes ``very_slow_publisher`` almost four times more time to publish one message than that of the ``fast_publisher``.
But the ``sync_callback`` is invoked for every incoming fast message with the previous values of all the other messages.
This is essentially an upsampling of slower messages.