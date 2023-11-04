#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotNewsStationNode : public rclcpp::Node {
public:
  RobotNewsStationNode() : Node("smartphone") {
    subscriber_ = this->create_subscription<example_interfaces::msg::String>(
        "robot_news", 10,
        std::bind(&RobotNewsStationNode::callbackRobotNews, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Smartphone has been started");
  }

private:
  void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }

  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
