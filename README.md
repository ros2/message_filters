# ROS2 message filters
Package that implements different filters for ros messages.

## C++ api

## Python Api
The python api has the following classes:
* Subscriber: ROS2 subscription filter, similar arguments as `rclpy.Subscriber`.
* Cache: Stores a time history of messages.
* TimeSynchronizer: Synchronizes messages by their timestamps.
* ApproximateTimeSynchronizer: Approximately synchronizes messages by their timestamps.

The implementation with the details can be found in [src/message_filters/__init__.py](src/message_filters/__init__.py)

### Example
And example for syncronizing one image topic and one pointcloud2 topic.

```python
import message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

class ExampleNode(Node):
    def __init__(self):
        super().__init__("ExampleNode")

        self.image_sub = message_filters.Subscriber(self, Image, "/image_topic")
        self.plc_sub = message_filters.Subscriber(
            self, PointCloud2, "/point_cloud_topic", qos_profile=qos_profile_sensor_data
        )
        queue_size = 30
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.plc_sub], queue_size)
        self.ts.registerCallback(self.callback)
        # or you can use ApproximateTimeSynchronizer if msgs dont have exactly the same timestamp
        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [self.image_sub, self.plc_sub],
        #     30,
        #     0.01,  # defines the delay (in seconds) with which messages can be synchronized
        # )

    def callback(self, image_msg, pcl_msg):
        # do your stuff here
        pass
```
**Note**: It is **VERY IMPORTANT** that each subscriber has the same `qos_profile` than the one specified in the corresponding publisher code. If they don't match, the callback won't be executed (without any warning) and you will be very frustated. In the example above, if the pointcloud subscriber didn't have explicitly that `qos_profile`, the callback wouldn't have been called.

The latest version (foxy) has the stable version of the python api. If you are using older distros please upgrade the code with the master branch.