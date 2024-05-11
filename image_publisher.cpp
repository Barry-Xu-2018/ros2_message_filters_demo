#include <chrono>
#include <memory>
#include <cstdlib>
#include <ctime>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;


class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_node")
  {
    srand(time(0));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
    timer_ = this->create_wall_timer(
      45ms, std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = this->get_clock()->now();
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 11));  // sleep 0~10 ms
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
