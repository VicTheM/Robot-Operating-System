/**
 * THIS FILE DEFINES A ROS2 PUBLISHER NODE THAT PUBLISHES A STRING
 * VIA THE TOPIC 'messages' AND ALSO LOGS IT TO THE CONSOLE
 */

// Include core c++ clibraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Inlude ROS2 Library and the message type definition
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// A subclass of a ROS2 Node { The main element of our program }
class GreetingsNode : public rclcpp::Node
{
	public:
		// Constructor
		// As u'd notice, most returned variables are pointers to the main object
		GreetingsNode()
			: Node("greetings_node"), count_(0)
		{
			publisher_ = this->create_publisher<std_msgs::msg::String>("messages", 10);
			timer_ = this->create_wall_timer(
					1000ms, std::bind(&GreetingsNode::timer_callback, this)
					);
		}

	private:
		void timer_callback()
		{
			auto message = std_msgs::msg::String();
			message.data = "[" + std::to_string(count_++) + "]\tGreetings from my end :)";
			RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
			publisher_->publish(message);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GreetingsNode>());
	rclcpp::shutdown();

	return (0);
}
