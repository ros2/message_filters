.. redirect-from::

   Tutorials/Synchronizer-Policies-Cpp

Approximate Time Synchronizer (C++):
---------------------------------------
.. code-block:: C++

    #include <message_filters/subscriber.h>
    #include <message_filters/synchronizer.h>
    #include <message_filters/sync_policies/approximate_time.h>
    #include <sensor_msgs/image.hpp>

    using namespace sensor_msgs::msg;
    using namespace message_filters;

    void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
    {
      // Solve all of perception here...
    }

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);

      rclcpp::Node node("node", 10);
      message_filters::Subscriber<Image> image1_sub(node, "image1", 1);
      message_filters::Subscriber<Image> image2_sub(node, "image2", 1);

      typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
      Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
      sync.registerCallback(std::bind(&callback, _1, _2));

      rclcpp::spin(node);
      rclcpp::shutdown();

      return 0;
    }
