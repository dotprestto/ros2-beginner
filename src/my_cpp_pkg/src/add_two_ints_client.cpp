#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using example_interfaces::srv::AddTwoInts;
using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsClientNode : public rclcpp::Node {
public:
  AddTwoIntsClientNode() : Node("add_two_ints_client") {

    threads_.push_back(std::thread(
        std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 2, 2)));
  }

  void callAddTwoIntsService(int a, int b) {
    auto client = this->create_client<AddTwoInts>("add_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1)))
      RCLCPP_WARN(this->get_logger(), "Waiting for the server...");

    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future = client->async_send_request(request);
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response->sum);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error while calling service");
    }
  }

private:
  std::vector<std::thread> threads_;
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
