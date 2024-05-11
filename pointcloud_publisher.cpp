#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;


class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("point_cloud_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PointCloudPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::PointCloud2();
    message.header.stamp = this->get_clock()->now();
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
