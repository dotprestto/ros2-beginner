#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("node_name") {
    RCLCPP_INFO(this->get_logger(), "Hello Cpp Node with OOP!");
  }

private:
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
