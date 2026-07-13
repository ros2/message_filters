^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package message_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.3.19 (2026-07-13)
-------------------
* Defer annotation evaluation to fix RHEL import. (backport `#309 <https://github.com/ros2/message_filters/issues/309>`_) (`#318 <https://github.com/ros2/message_filters/issues/318>`_)
* Tutorials: Add DeltaFilter C++ tutorial (`#304 <https://github.com/ros2/message_filters/issues/304>`_) (`#308 <https://github.com/ros2/message_filters/issues/308>`_)
* Contributors: mergify[bot]

4.3.18 (2026-05-25)
-------------------
* fix tutorials table of contents fix input aligner type hint (backport `#290 <https://github.com/ros2/message_filters/issues/290>`_) (`#300 <https://github.com/ros2/message_filters/issues/300>`_)
* Contributors: mergify[bot]

4.3.17 (2026-05-20)
-------------------
* feat(python): add python implementation of InputAligner  (backport `#283 <https://github.com/ros2/message_filters/issues/283>`_) (`#289 <https://github.com/ros2/message_filters/issues/289>`_)
* (`#221 <https://github.com/ros2/message_filters/issues/221>`_) Tutorials: Add DeltaFilter Python tutorial (backport `#277 <https://github.com/ros2/message_filters/issues/277>`_) (`#280 <https://github.com/ros2/message_filters/issues/280>`_)
* Contributors: mergify[bot]

4.3.16 (2026-04-09)
-------------------
* DeltaFilter(C++): Add DeltaFilter class. (backport `#273 <https://github.com/ros2/message_filters/issues/273>`_) (`#276 <https://github.com/ros2/message_filters/issues/276>`_)
* Contributors: mergify[bot]

4.3.15 (2026-03-24)
-------------------
* Tutorials minor fixers: Replace the TODOs with the actual links to other tutorials as required. Rename Approximate-Tyme tutorial to Approximate-Time (`#266 <https://github.com/ros2/message_filters/issues/266>`_)
* Tutorials: Add LatestTime synchronization policy tutorial (`#266 <https://github.com/ros2/message_filters/issues/266>`_)
* Tutorials: Approximate-Synchronizer: Label CMake code blocks with the right language markings
* Tutorials: Add C++ tutorial for Approximate Epsilon Time Sync policy
* DeltaFilter(Python): Add DeltaFilter for Python. Add tests. Add docstring to filters and comparison handlers (`#252 <https://github.com/ros2/message_filters/issues/252>`_) (`#260 <https://github.com/ros2/message_filters/issues/260>`_)
  Co-authored-by: Pavel Esipov <38457822+EsipovPA@users.noreply.github.com>
* Contributors: EsipovPA, mergify[bot]

4.3.14 (2026-02-18)
-------------------
* `#253 <https://github.com/ros2/message_filters/issues/253>`_ message_filters.chain.python: Remove dataclass. Make FilterInfo a regular class with regular __init_\_ (`#254 <https://github.com/ros2/message_filters/issues/254>`_)
* Contributors: Pavel Esipov

4.3.13 (2026-02-04)
-------------------
* Add kwargs passing from Subscriber to node.create_subscription (`#247 <https://github.com/ros2/message_filters/issues/247>`_) (`#250 <https://github.com/ros2/message_filters/issues/250>`_)
* Contributors: mergify[bot]

4.3.12 (2026-01-15)
-------------------
* `#200 <https://github.com/ros2/message_filters/issues/200>`_ fix inconsistensy between cpp and python exact time synchronizer impl (backport `#238 <https://github.com/ros2/message_filters/issues/238>`_) (`#245 <https://github.com/ros2/message_filters/issues/245>`_)
* `#130 <https://github.com/ros2/message_filters/issues/130>`_ add simple filter tutorial for cpp (backport `#239 <https://github.com/ros2/message_filters/issues/239>`_) (`#242 <https://github.com/ros2/message_filters/issues/242>`_)
* Add simple filter tutorials (backport `#226 <https://github.com/ros2/message_filters/issues/226>`_) (`#230 <https://github.com/ros2/message_filters/issues/230>`_)
* Add chain tutorial python (`#219 <https://github.com/ros2/message_filters/issues/219>`_) (`#225 <https://github.com/ros2/message_filters/issues/225>`_)
* Contributors: mergify[bot]

4.3.11 (2025-10-15)
-------------------
* Add Python implementation for a Chain filter (backport `#213 <https://github.com/ros2/message_filters/issues/213>`_) (`#216 <https://github.com/ros2/message_filters/issues/216>`_)
* Update repository URL in package.xml (`#217 <https://github.com/ros2/message_filters/issues/217>`_) (`#218 <https://github.com/ros2/message_filters/issues/218>`_)
* Contributors: mergify[bot]

4.3.10 (2025-10-03)
-------------------
* Some fixes to documentation (backport `#208 <https://github.com/ros2/message_filters/issues/208>`_) (`#212 <https://github.com/ros2/message_filters/issues/212>`_)
* Create a Chain class tutorial for C++ (`#203 <https://github.com/ros2/message_filters/issues/203>`_) (`#207 <https://github.com/ros2/message_filters/issues/207>`_)
* Contributors: mergify[bot]

4.3.9 (2025-09-17)
------------------
* Add 'Cache (C++)' tutorial (`#196 <https://github.com/ros2/message_filters/issues/196>`_) (`#199 <https://github.com/ros2/message_filters/issues/199>`_)
* Contributors: mergify[bot]

4.3.8 (2025-07-16)
------------------
* Fix cache tutorial: added tab extension (backport `#190 <https://github.com/ros2/message_filters/issues/190>`_) (`#193 <https://github.com/ros2/message_filters/issues/193>`_)
* Add tutorial for Cache filter for Python (`#185 <https://github.com/ros2/message_filters/issues/185>`_) (`#189 <https://github.com/ros2/message_filters/issues/189>`_)
* Contributors: mergify[bot]

4.3.7 (2025-04-08)
------------------
* Future port hpp files (`#170 <https://github.com/ros2/message_filters/issues/170>`_) (`#172 <https://github.com/ros2/message_filters/issues/172>`_)
* Contributors: mergify[bot]

4.3.6 (2025-03-25)
------------------
* Move from Wiki and Updated Python docs (backport `#150 <https://github.com/ros2/message_filters/issues/150>`_) (`#152 <https://github.com/ros2/message_filters/issues/152>`_)
* Contributors: mergify[bot]

4.3.5 (2024-07-31)
------------------
* [LatestTimeSync] Fix crash when Synchronizer is started before the messges are available. (`#136 <https://github.com/ros2/message_filters/issues/136>`_)
* Contributors: Dr. Denis

4.3.4 (2024-05-15)
------------------
* Fix cache.h std::placeholder namespace (`#87 <https://github.com/ros2/message_filters/issues/87>`_) (`#121 <https://github.com/ros2/message_filters/issues/121>`_)
* Contributors: mergify[bot]

4.3.3 (2023-04-25)
------------------
* Add latest time zero-order-hold sync policy (`#73 <https://github.com/ros2/message_filters/issues/73>`_) (`#89 <https://github.com/ros2/message_filters/issues/89>`_)
* Contributors: mergify[bot]

4.3.2 (2022-06-20)
------------------
* Adding fix to subscribe() call with raw node pointer and subscriber options (`#76 <https://github.com/ros2/message_filters/issues/76>`_) (`#77 <https://github.com/ros2/message_filters/issues/77>`_)
* Contributors: Steve Macenski

4.3.1 (2022-03-25)
------------------
* Use RCL_ROS_TIME for message_traits::TimeStamp  (`#72 <https://github.com/ros2/message_filters/issues/72>`_)
* Contributors: Kenji Brameld

4.3.0 (2022-03-01)
------------------
* Install includes to include/${PROJECT_NAME} (`#71 <https://github.com/ros2/message_filters/issues/71>`_)
* Update maintainers (`#67 <https://github.com/ros2/message_filters/issues/67>`_)
* Contributors: Audrow Nash, Shane Loretz

4.2.0 (2021-10-18)
------------------
* Suppress rclcpp deprecation warnings in unit tests (`#62 <https://github.com/ros2/message_filters/issues/62>`_)
* Contributors: Abrar Rahman Protyasha

4.1.0 (2021-07-29)
------------------
* Add missing overrides to subscriber.h (`#60 <https://github.com/ros2/message_filters/issues/60>`_)
* Add lifecycle node support (`#59 <https://github.com/ros2/message_filters/issues/59>`_)
* Correct package.xml and CMakeLists.txt (`#58 <https://github.com/ros2/message_filters/issues/58>`_)
* Contributors: Hunter L. Allen, Michel Hidalgo, Rebecca Butler

4.0.0 (2021-05-26)
------------------
* Expose Subscription Options - V2 (`#56 <https://github.com/ros2/message_filters/issues/56>`_)
* Contributors: Audrow Nash

3.2.6 (2021-05-12)
------------------
* Find and export dependencies properly (`#54 <https://github.com/ros2/message_filters/issues/54>`_)
* Contributors: Michel Hidalgo

3.2.5 (2020-12-10)
------------------
* Add pytest.ini so local tests don't display warning (`#47 <https://github.com/ros2/message_filters/issues/47>`_)
* Contributors: Chris Lalancette

3.2.4 (2020-06-03)
------------------
* export targets in a addition to include directories / libraries (`#46 <https://github.com/ros2/message_filters/issues/46>`_)
* Contributors: Dirk Thomas

3.2.3 (2019-11-18)
------------------
* allow custom qos for message filters

3.2.2 (2019-11-08)
------------------
* Fix  unhashable type 'Time' error (`#33 <https://github.com/ros2/message_filters/issues/33>`_)
* Contributors: Jamie Diprose

3.2.1 (2019-10-23)
------------------
* Resolve ambiguity with boost/bind.hpp (`#40 <https://github.com/ros2/message_filters/issues/40>`_)
* Contributors: Shane Loretz

3.2.0 (2019-09-26)
------------------

3.1.2 (2019-05-20)
------------------
* Add .gitignore
* Fix deprecation warning
* Contributors: Jacob Perron

3.1.1 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#26 <https://github.com/ros2/message_filters/issues/26>`_)
* Merge pull request `#25 <https://github.com/ros2/message_filters/issues/25>`_ from ros2/ivanpauno/deprecate-shared-ptr-publish
* adding code import references in comments (`#6 <https://github.com/ros2/message_filters/issues/6>`_)
* Make format string agree with argument type. (`#24 <https://github.com/ros2/message_filters/issues/24>`_)
* Contributors: Steven! Ragnarök, Tully Foote, William Woodall, ivanpauno

3.1.0 (2019-04-14)
------------------
* Added direct dependency on python_cmake_module. (`#19 <https://github.com/ros2/message_filters/issues/19>`_)
* Updated to use Python debug interpreter on Windows. (`#18 <https://github.com/ros2/message_filters/issues/18>`_)
* Contributors: Dirk Thomas, Steven! Ragnarök

3.0.0 (2018-11-22)
------------------
* Move sensor_msgs to be a test dependency. (`#17 <https://github.com/ros2/message_filters/issues/17>`_)
* Fix Duration signature for Windows CI. (`#16 <https://github.com/ros2/message_filters/issues/16>`_)
* Add the ability to use raw pointers in Subscriber (`#14 <https://github.com/ros2/message_filters/issues/14>`_)
* fixed using wrong type of stamp (`#12 <https://github.com/ros2/message_filters/issues/12>`_)
* Add message trait support to frame id of message (`#13 <https://github.com/ros2/message_filters/issues/13>`_)
* Merge pull request `#10 <https://github.com/ros2/message_filters/issues/10>`_ from ros2/fix_windows
* Change argument name to better reflect behavior.
* Fix signedness of some comparisons.
* Correctly export and depend on ament_cmake_ros.
* Cleanup windows warnings.
* Fix visibility control.
* Fix include guards and include order.
* enable message_filters support of python interfaces and tests (`#7 <https://github.com/ros2/message_filters/issues/7>`_)
* Remove references in pointer API calls.
* Remove ROS1 specific connection header info from API.
* demote std_msgs to test dependency
* Update docs to remove nodehandle reference.
* Use Node::SharedPtr instead of Node*
* Make node required as an internal one is required to spin.
* Add noncopyable base class
* Deboostify and change from ros::Time to rclcpp::Time
* Convert to ament_cmake
* Changed invocation to `add` to conform template syntax (`#1388 <https://github.com/ros2/message_filters/issues/1388>`_)
  This change fixes issue `#1383 <https://github.com/ros2/message_filters/issues/1383>`_
* fix sphinx warning (`#1371 <https://github.com/ros2/message_filters/issues/1371>`_)
* Contributors: Dirk Thomas, Ethan Gao, Gary Liu, Jørgen Nordmoen, Michael Carroll, Tully Foote


1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* change invocation to `add` to conform template syntax (`#1388 <https://github.com/ros/ros_comm/issues/1388>`_)
* fix sphinx warning (`#1371 <https://github.com/ros/ros_comm/issues/1371>`_)

1.13.6 (2018-02-05)
-------------------
* use SteadyTimer in message_filters (`#1247 <https://github.com/ros/ros_comm/issues/1247>`_)
* remove unnecessary xmlrpcpp dependency from message_filters (`#1264 <https://github.com/ros/ros_comm/issues/1264>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------

1.12.6 (2016-10-26)
-------------------
* use boost::bind to bind the callback function (`#906 <https://github.com/ros/ros_comm/pull/906>`_)

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* add fast approximate time synchronization in message_filters (in pure Python) (`#802 <https://github.com/ros/ros_comm/issues/802>`_)

1.12.2 (2016-06-03)
-------------------
* allow saving timestamp-less messages to Cache, add getLast method (`#806 <https://github.com/ros/ros_comm/pull/806>`_)

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------
* fix compiler warnings

1.11.17 (2016-03-11)
--------------------
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)
* add __getattr_\_ to handle sub in message_filters as standard one (`#700 <https://github.com/ros/ros_comm/pull/700>`_)

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------
* add unregister() method to message_filter.Subscriber (`#683 <https://github.com/ros/ros_comm/pull/683>`_)

1.11.14 (2015-09-19)
--------------------

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* implement message filter cache in Python (`#599 <https://github.com/ros/ros_comm/pull/599>`_)

1.11.10 (2014-12-22)
--------------------

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* add approximate Python time synchronizer (used to be in camera_calibration) (`#424 <https://github.com/ros/ros_comm/issues/424>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* update API to use boost::signals2 (`#267 <https://github.com/ros/ros_comm/issues/267>`_)

1.11.0 (2014-03-04)
-------------------
* suppress boost::signals deprecation warning (`#362 <https://github.com/ros/ros_comm/issues/362>`_)

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------
* add kwargs for message_filters.Subscriber

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* update code after refactoring into rosbag_storage and roscpp_core (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* fix segmentation fault on OS X 10.9 (clang / libc++)

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------
* fix template syntax for signal\_.template addCallback() to work with Intel compiler

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------
* fix exports of message filter symbols for Windows

1.9.42 (2013-03-08)
-------------------

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
