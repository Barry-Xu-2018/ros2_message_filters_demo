#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


using std::placeholders::_1;
using std::placeholders::_2;

class MessageFiltersDemoNode : public rclcpp::Node {
public:
  MessageFiltersDemoNode()
  :rclcpp::Node("check_message_filter_node", rclcpp::NodeOptions())
  {
    image_subscriber_.subscribe(this, "image_topic", rmw_qos_profile_default);
    pointcloud_subscriber_.subscribe(this, "pointcloud_topic", rmw_qos_profile_default);

    // queue_size = 10
    my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(10), pointcloud_subscriber_, image_subscriber_);
    my_sync_->registerCallback(std::bind(&MessageFiltersDemoNode::sync_callback, this, _1, _2));
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> approximate_policy;
  typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_subscriber_;

  std::shared_ptr<approximate_synchronizer> my_sync_;

  void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg, const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
  {
    static int old_sec = 0;
    if (old_sec != pointcloud_msg->header.stamp.sec) {
      if (old_sec != 0) {
        printf("\n\n");
      }
      old_sec = pointcloud_msg->header.stamp.sec;
    }
    RCLCPP_INFO(this->get_logger(),
      "synchronized timestamps:PointCloud  %u.%09u, Image %u.%09u",
       pointcloud_msg->header.stamp.sec, pointcloud_msg->header.stamp.nanosec, img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto demo_node = std::make_shared<MessageFiltersDemoNode>();

  #if 1 // Single thread executor
  rclcpp::spin(demo_node);
  #else // Multithread executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(check_node);
  executor.spin();
  #endif

  rclcpp::shutdown();
  return 0;
}
