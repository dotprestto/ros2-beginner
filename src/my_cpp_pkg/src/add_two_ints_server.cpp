#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsClientNode : public rclcpp::Node {
public:
  AddTwoIntsClientNode() : Node("add_two_ints_server") {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsClientNode::callbackAddTwoInts, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Service server has been started");
  }

private:
  void callbackAddTwoInts(
      const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
      const example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a,
                (int)request->b, (int)response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
