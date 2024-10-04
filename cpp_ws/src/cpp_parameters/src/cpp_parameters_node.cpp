#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// Inherit a Node class
class MinimalParam : public rclcpp::Node
{
public:
  // In the initialization of Node class
  MinimalParam()
  : Node("minimal_param_node")	// Initialize the super class
  {
    // Create a parameter bu declaring it
    this->declare_parameter("my_parameter", "world"); // my_parameter is a parameter, a configurable variable. its current value is 'world'

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
    counter_ = 0;
  }

  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", std::to_string(counter_++))};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
