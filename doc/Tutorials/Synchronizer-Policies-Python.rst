.. redirect-from::

    Tutorials/Synchronizer-Policies-Python

Approximate Time Synchronizer (Python):
---------------------------------------
.. code-block:: Python

    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from sensor_msgs.msg import Image, PointCloud2

    class ExampleNode(Node):
        def __init__(self):
            super().__init__("ExampleNode")

            self.image_sub = Subscriber(self, Image, "/image_topic")
            self.plc_sub = Subscriber(
                self, PointCloud2, "/point_cloud_topic", qos_profile=qos_profile_sensor_data
            )
            queue_size = 30
            # you can use ApproximateTimeSynchronizer if msgs dont have exactly the same timestamp
            self.ts = ApproximateTimeSynchronizer(
                [self.image_sub, self.plc_sub],
                queue_size,
                0.01,  # defines the delay (in seconds) with which messages can be synchronized
            )

        def callback(self, image_msg, pcl_msg):
            # do your stuff here
            pass


