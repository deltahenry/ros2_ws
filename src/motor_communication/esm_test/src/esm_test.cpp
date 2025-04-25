#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "uros_interface/srv/es_mcmd.hpp"
#include "uros_interface/msg/joint_arr.hpp"
#include "uros_interface/msg/joint2_d_arr.hpp"

using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
	JointPublisher() : Node("ESM_controller")
	{
		//create publisher for 2d array command
		pub_ = this->create_publisher<uros_interface::msg::Joint2DArr>("theta_command", 10);

		// cycle time of every 2d array command
		timer_ = this->create_wall_timer(40ms, std::bind(&JointPublisher::publish_array, this));

		//service for request servo on/off
		client_ = this->create_client<uros_interface::srv::ESMcmd>("esm_command");

		// Wait for the service to be available
		while (!client_->wait_for_service(2s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for esmcmd service.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "Waiting for esmcmd service...");
		}

		// Create and send the request
		// when the code is ros2 runned, servo on the servos
		auto request = std::make_shared<uros_interface::srv::ESMcmd::Request>();
		request->servo_status = true;
		request->mode = 4;
		request->speed_limit = 20;
		request->lpf = 10;

		using ServiceResponseFuture = rclcpp::Client<uros_interface::srv::ESMcmd>::SharedFuture;
		
		//get alarm code when request
		auto response_callback = [this](ServiceResponseFuture future) {
			auto response = future.get();
			RCLCPP_INFO(this->get_logger(), "Received ESM state: %d", response->esm_state);
		};

		client_->async_send_request(request, response_callback);
	}

private:

void publish_array() {
	uros_interface::msg::Joint2DArr msg;

	float per = 0.1;

	switch (current_state_) {
		case State::UP:
			base_val_ += per;
			break;
		case State::DOWN:
			base_val_ -= per;
			break;
		case State::PAUSE_AFTER_UP:
		case State::PAUSE_AFTER_DOWN:
			pause_count_++;
			if (pause_count_ >= 25) { // 1 second pause
				pause_count_ = 0;
				if (current_state_ == State::PAUSE_AFTER_UP) {
					current_state_ = State::DOWN;
				}
				else {
					current_state_ = State::UP;
				}
			}
			return; // skip publishing during pause
	}

	for (int i = 0; i < 10; ++i) {
		float val = base_val_ + i * per/10;

		uros_interface::msg::JointArr row;
		row.theta_arr[0] = 0 + val;
		row.theta_arr[1] = 0 + val;
		row.theta_arr[2] = 0 + val;
		row.theta_arr[3] = 0 + val;
		row.theta_arr[4] = 0 + val;
		row.theta_arr[5] = 0 + val;

		msg.theta_2d_arr.push_back(row);
	}

	pub_->publish(msg);

	step_++;
	if (step_ >= 25) {
		step_ = 0;
		if (current_state_ == State::UP) {
			current_state_ = State::PAUSE_AFTER_UP;
		}
		else if (current_state_ == State::DOWN) {
			current_state_ = State::PAUSE_AFTER_DOWN;
		}
	}
	
	}

	rclcpp::Publisher<uros_interface::msg::Joint2DArr>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Client<uros_interface::srv::ESMcmd>::SharedPtr client_;

	enum class State { UP, PAUSE_AFTER_UP, DOWN, PAUSE_AFTER_DOWN };
	State current_state_ = State::UP;

	int step_ = 0;
	int pause_count_ = 0;
	float base_val_ = 0;

};

//if Ctrl+c is triggered, servo off the servos
void sigint_handler(int)
{
	if (!rclcpp::ok()) return;

	auto shutdown_node = std::make_shared<rclcpp::Node>("shutdown_helper");

	auto client = shutdown_node->create_client<uros_interface::srv::ESMcmd>("esm_command");

	if (!client->wait_for_service(2s)) {
		RCLCPP_WARN(shutdown_node->get_logger(), "Shutdown: service not available.");
	}
	else{
		auto request = std::make_shared<uros_interface::srv::ESMcmd::Request>();
		request->servo_status = false;
		request->mode = 4;
		request->speed_limit = 20;
		request->lpf = 10;

		auto result = client->async_send_request(request);

		rclcpp::executors::SingleThreadedExecutor exec;
		exec.add_node(shutdown_node);

		if (exec.spin_until_future_complete(result, 3s) == rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_INFO(shutdown_node->get_logger(), "Shutdown: ESM state = %d", result.get()->esm_state);
		}
		else
		{
			RCLCPP_ERROR(shutdown_node->get_logger(), "Shutdown: Failed to call service.");
		}

		exec.remove_node(shutdown_node);
	}

	rclcpp::shutdown();
}

//main
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	std::signal(SIGINT, sigint_handler);  

	auto node = std::make_shared<JointPublisher>();

	rclcpp::spin(node); 

	return 0;
}