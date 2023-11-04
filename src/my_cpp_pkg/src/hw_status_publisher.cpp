#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

using my_robot_interfaces::msg::HardwareStatus;

class HardwareStatusPublisher : public rclcpp::Node {
public:
  HardwareStatusPublisher() : Node("robot_news_station"), robot_name_("R2D2") {
    publisher_ = this->create_publisher<HardwareStatus>("hardware_status", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&HardwareStatusPublisher::publishHardwareStatus, this));
    RCLCPP_INFO(this->get_logger(), "Robot News has been started");
  }

private:
  void publishHardwareStatus() {
    auto msg = HardwareStatus();
    msg.temperature = 57;
    msg.are_motors_ready = true;
    msg.debug_message = "Motors are too hot!";
    publisher_->publish(msg);
  }

  std::string robot_name_;
  rclcpp::Publisher<HardwareStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
