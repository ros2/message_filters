Message Filters
===============

:mod:`message_filters` is a collection of message "filters" which take messages in,
either from a ROS 2 subscription or another filter,
and may or may not output the message
at some time in the future, depending on a policy defined for that filter.

``message_filters`` also defines a common interface for these filters, allowing you to chain them together.

The filters currently implemented in this package are:

 * :class:`message_filters.Subscriber` A source filter, which wraps a ROS 2 subscription. Most filter chains will begin with a Subscriber.
 * :class:`message_filters.Cache` Caches messages which pass through it, allowing later lookup by time stamp.
 * :class:`message_filters.TimeSynchronizer` Synchronizes multiple messages by their timestamps, only passing them through when all have arrived.
 * :class:`message_filters.TimeSequencer` Tries to pass messages through ordered by their timestamps, even if some arrive out of order.

1. Filter Pattern
-----------------
All message filters follow the same pattern for connecting inputs and outputs. Inputs are connected either through the filter's constructor or through the ``connectInput()`` method. Outputs are connected through the ``registerCallback()`` method.

Note that the input and output types are defined perfilter, so not all filters are directly interconnectable.

Filter Construction (C++)::

    FooFilter foo;
    BarFilter bar;
    BazFilter baz(foo);
    bar.connectInput(foo);

Filter Construction (Python)::

    bar(foo)
    bar.connectInput(foo)

1.1 registerCallback()
~~~~~~~~~~~~~~~~~~~~~~
You can register multiple callbacks with the ``registerCallbacks()`` method. They will get called in the order they are registered. The signature of the callback depends on the definition of the filter

In C++ ``registerCallback()`` returns a ``message_filters::Connection`` object that allows you to disconnect the callback by calling its ``disconnect()``  method. You do not need to store this connection object if you do not need to manually disconnect the callback.

2. Subscriber
-------------
The Subscriber filter is simply a wrapper around a ROS 2 subscription that provides a source for other filters. The Subscriber filter cannot connect to another filter's output, instead it uses a ROS 2 topic as its input.

2.1 Connections
~~~~~~~~~~~~~~~
Input:
  No input connections
Output:
  * C++: ``void callback(const std::shared_ptr<M const>&)``
  * Python: ``callback(msg)``

2.2 Example (C++)
~~~~~~~~~~~~~~~~~
.. code-block:: C++

    message_filters::Subscriber<example_interfaces::msg::UInt32> sub(node, "my_topic", 1);
    sub.registerCallback(myCallback);

or

.. code-block:: C++

    message_filters::Subscriber sub = node.subscribe("my_topic", 1, myCallback);

2.3 Example (Python)
~~~~~~~~~~~~~~~~~~~~

.. code-block:: Python

   sub = message_filters.Subscriber(node_object, "pose_topic", robot_msgs.msg.Pose)
   sub.registerCallback(myCallback)

3. Time Synchronizer
--------------------
The TimeSynchronizer filter synchronizes incoming channels by the timestamps contained in their headers, and outputs them in the form of a single callback that takes the same number of channels. The C++ implementation can synchronize up to 9 channels.

3.1 Connections
~~~~~~~~~~~~~~~
Input:
  * C++: Up to 9 separate filters, each of which is of the form ``void callback(const std::shared_ptr<M const>&)``. The number of filters supported is determined by the number of template arguments the class was created with.
  * Python: N separate filters, each of which has signature ``callback(msg)``.

Output:
  * C++: For message types M0..M8, ``void callback(const std::shared_ptr<M0 const>&, ..., const std::shared_ptr<M8 const>&)``. The number of parameters is determined by the number of template arguments the class was created with.
  * Python: ``callback(msg0.. msgN)``. The number of parameters is determined by the number of template arguments the class was created with.

4. Time Sequencer
-----------------
* Python: the TimeSequencer filter is not yet implemented.

The TimeSequencer filter guarantees that messages will be called in temporal order according to their header's timestamp. The TimeSequencer is constructed with a specific delay which specifies how long to queue up messages before passing them through. A callback for a message is never invoked until the messages' time stamp is out of date by at least delay. However, for all messages which are out of date by at least the delay, their callback are invoked and guaranteed to be in temporal order. If a message arrives from a time prior to a message which has already had its callback invoked, it is thrown away.

4.1 Connections
~~~~~~~~~~~~~~~
Input:
  * C++: ``void callback(const std::shared_ptr<M const>&)``
Output:
  * C++: ``void callback(const std::shared_ptr<M const>&)``

4.2 Example (C++)
~~~~~~~~~~~~~~~~~
.. code-block:: C++

     message_filters::Subscriber<example_interfaces::msg::String> sub(node, "my_topic", 1);
     message_filters::TimeSequencer<example_interfaces::msg::String> seq(
       sub, rclcpp::Duration(0.1), rclcpp::Duration(0.01), 10);
     seq.registerCallback(myCallback);

5. Cache
--------
Stores a time history of messages.

Given a stream of messages, the most recent N messages are cached in a ring buffer, from which time intervals of the cache can then be retrieved by the client. The timestamp of a message is determined from its header field.

If the message type doesn't contain a header, see below for workaround.

The Cache immediately passes messages through to its output connections.

5.1 Connections
~~~~~~~~~~~~~~~
Input:
  * C++: ``void callback(const std::shared_ptr<M const>&)``
  * Python: ``callback(msg)``
Output:
  * C++: ``void callback(const std::shared_ptr<M const>&)``
  * Python: ``callback(msg)``

5.2 Example (C++)
~~~~~~~~~~~~~~~~~
.. code-block:: C++

    message_filters::Subscriber<example_interfaces::msg::String> sub(node, "my_topic", 1);
    message_filters::Cache<example_interfaces::msg::String> cache(sub, 100);
    cache.registerCallback(myCallback);

5.3 Example (Python)
~~~~~~~~~~~~~~~~~~~~
.. code-block:: Python

    sub = message_filters.Subscriber(node_object, 'my_topic', sensor_msgs.msg.Image)
    cache = message_filters.Cache(sub, 100)

In this example, the Cache stores the last 100 messages received on ``my_topic``, and ``myCallback`` is called on the addition of every new message. The user can then make calls like ``cache.getInterval(start, end)`` to extract part of the cache.
If the message type does not contain a header field that is normally used to determine its timestamp, and the Cache is contructed with ``allow_headerless=True``, the current ROS 2 time is used as the timestamp of the message. This is currently only available in Python.

6. PolicyBased Synchronizers
----------------------------
The Synchronizer filter synchronizes incoming channels by the timestamps contained in their headers, and outputs them in the form of a single callback that takes the same number of channels. The C++ implementation can synchronize up to 9 channels.

The Synchronizer filter is templated on a policy that determines how to synchronize the channels. There are currently three policies: ExactTime, ApproximateEpsilonTime and ApproximateTime.

6.1 Connections
~~~~~~~~~~~~~~~
Input:
  * C++: Up to 9 separate filters, each of which is of the form ``void callback(const std::shared_ptr<M const>&)``. The number of filters supported is determined by the number of template arguments the class was created with.
  * Python: N separate filters, each of which has signature ``callback(msg)``.
Output:
  * C++: For message types M0..M8, ``void callback(const std::shared_ptr<M0 const>&, ..., const std::shared_ptr<M8 const>&)``. The number of parameters is determined by the number of template arguments the class was created with.
  * Python: ``callback(msg0.. msgN)``. The number of parameters is determined by the number of template arguments the class was created with.

6.2 ExactTime Policy
~~~~~~~~~~~~~~~~~~~~
The ``message_filters::sync_policies::ExactTime`` policy requires messages to have exactly the same timestamp in order to match. Your callback is only called if a message has been received on all specified channels with the same exact timestamp. The timestamp is read from the header field of all messages (which is required for this policy).

6.3 ApproximateEpsilonTime Policy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The ``message_filters::sync_policies::ApproximateEpsilonTime`` policy requires messages to have exactly the same timestamp within an "epsilon" tolerance in order to match. Your callback is only called if a message has been received on all specified channels with that same exact timestamp within the tolerance. The timestamp is read from the header field of all messages (which is required for this policy).

6.4 ApproximateTime Policy
~~~~~~~~~~~~~~~~~~~~~~~~~~
The ``message_filters::sync_policies::ApproximateTime`` policy uses an adaptive algorithm to match messages based on their timestamp.

If not all messages have a header field from which the timestamp could be determined, see below for a workaround. If some messages are of a type that doesn't contain the header field, ``ApproximateTimeSynchronizer`` refuses by default adding such messages. However, its Python version can be constructed with ``allow_headerless=True``, which uses current ROS 2 time in place of any missing header.stamp field:

7. Chain
--------
**Python:** the Chain filter is not yet implemented.
The Chain filter allows you to dynamically chain together multiple singleinput/single-output (simple) filters. As filters are added to it they are automatically connected together in the order they were added. It also allows you to retrieve added filters by index.

Chain is most useful for cases where you want to determine which filters to apply at runtime rather than compiletime.

7.1 Connections
~~~~~~~~~~~~~~~
Input:
  * C++: ``void callback(const std::shared_ptr<M const>&)``
Output:
  * C++: ``void callback(const std::shared_ptr<M const>&)``

7.2 Examples (C++)
~~~~~~~~~~~~~~~~~~
**Simple Example**

.. code-block:: C++

     void myCallback(const MsgConstPtr& msg)
     {
     }

     Chain<Msg> c;
     c.addFilter(std::shared_ptr<Subscriber<Msg> >(new Subscriber<Msg>));
     c.addFilter(std::shared_ptr<TimeSequencer<Msg> >(new TimeSequencer<Msg>));
     c.registerCallback(myCallback);

**Bare Pointers**
It is possible to pass bare pointers in. These will not be automatically deleted when Chain is destructed.

.. code-block:: C++

     Chain<Msg> c;
     Subscriber<Msg> s;
     c.addFilter(&s);
     c.registerCallback(myCallback);

**Retrieving a Filter**

.. code-block:: C++

     Chain<Msg> c;
     size_t sub_index = c.addFilter(std::shared_ptr<Subscriber<Msg> >(new Subscriber<Msg>));
     std::shared_ptr<Subscriber<Msg> > sub = c.getFilter<Subscriber<Msg> >(sub_index);


The message filter interface
----------------------------

For an object to be usable as a message filter, it needs to have one method,
``registerCallback``.  To collect messages from a message filter, register a callback with::

    anyfilter.registerCallback(my_callback)

The signature of ``my_callback`` varies according to the message filter.  For many filters it is simply::

    def my_callback(msg):

where ``msg`` is the message.

Message filters that accept input from an upstream
message filter (e.g. :class:`message_filters.Cache`) register their own
message handler as a callback.

Output connections are registered through the ``registerCallback()`` function.

Here's a simple example of using a Subscriber with a Cache::

    def myCallback(posemsg):
       print(posemsg)

    sub = message_filters.Subscriber(node_object, robot_msgs.msg.Pose, "pose_topic")
    cache = message_filters.Cache(sub, 10)
    cache.registerCallback(myCallback)

The Subscriber here acts as the source of messages. Each message is passed to the cache,
which then passes it through to the user's callback ``myCallback``.

**Note**: It is **VERY IMPORTANT** that each subscriber has the same ``qos_profile`` as the one specified in the corresponding publisher code for each topic you want to subscribe to.
If they don't match, the callback won't be executed (without any warning) and you will be very frustrated.

.. toctree::
    :maxdepth: 2

    self
    tutorials
    references

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
