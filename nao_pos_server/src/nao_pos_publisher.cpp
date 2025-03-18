#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class NaoPosPublisher : public rclcpp::Node
{
public:
  NaoPosPublisher() : Node("nao_pos_publisher"), is_walking_(false)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter sets the desired pos file to execute";
    this->declare_parameter("pos_file", "only_legs_fast", param_desc);

    publisher_ = this->create_publisher<std_msgs::msg::String>("action_req", 10);

    // Subscribe to /walk_status to know if the robot is walking
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/walk_status", 10, std::bind(&NaoPosPublisher::walk_status_callback, this, std::placeholders::_1));

    // Wait to receive the first message from /walk_status before starting the timer
    while (rclcpp::ok() && !received_first_status_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the first message from /walk_status...");
      rclcpp::sleep_for(500ms);
      rclcpp::spin_some(this->get_node_base_interface());
    }

    timer_ = this->create_wall_timer(2000ms, std::bind(&NaoPosPublisher::timer_callback, this));
  }

private:
  void walk_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_walking_ = msg->data;
    received_first_status_ = true;  // Mark that we have received the first status
    if (is_walking_) {
      RCLCPP_INFO(this->get_logger(), "Robot is walking, stopping position publication.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Robot is stationary, resuming position publication.");
    }
  }

  void timer_callback()
  {
    if (is_walking_ && !fist_time_) {
      RCLCPP_DEBUG(this->get_logger(), "No action published, the robot is walking.");
      return; // Do not publish if the robot is walking
    }

    if (fist_time_) {
      fist_time_ = false;
    }

    std::string file_name = this->get_parameter("pos_file").as_string();
    auto message = std_msgs::msg::String();
    message.data = file_name;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  bool is_walking_;
  bool received_first_status_ = false;  // Variable to wait for the first message from /walk_status
  bool fist_time_ = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NaoPosPublisher>());
  rclcpp::shutdown();
  return 0;
}
