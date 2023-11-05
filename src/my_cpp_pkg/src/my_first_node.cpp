#include "rclcpp/rclcpp.hpp"

class RobotNewsStationNode : public rclcpp::Node {
public:
  RobotNewsStationNode() : Node("cpp_test") {
    RCLCPP_INFO(this->get_logger(), "Hello Cpp Node with OOP!");

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotNewsStationNode::timerCallback, this));
  }

private:
  void timerCallback() {
    counter_++;
    RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  short counter_ = 0;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
