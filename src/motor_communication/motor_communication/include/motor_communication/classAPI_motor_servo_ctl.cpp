/** ******************************************************
	* @file		classAPI_motor_servo_ctl.cpp
	* @author	Tsai,Li-chun
	******************************************************
**/


/* System Includes ------------------------------------------*/
/* System Includes Begin */
/* System Includes End */
/* User Includes --------------------------------------------*/
/* User Includes Begin */
#include "classAPI_motor_servo_ctl.hpp"
/* User Includes End */

/* namespace ------------------------------------------------*/
/* namespace Begin */
/* namespace End */


/* Define ---------------------------------------------------*/
/* Define Begin */
/* Define End */


/* Typedef --------------------------------------------------*/
/* Typedef Begin */
/* Typedef End */


/* Class --------------------------------------------------*/
/* Class Begin */
/* Class End */


/* Variables ------------------------------------------------*/
/* Variables Begin */
/* Variables End */


/* Function -------------------------------------------------*/
/* Function Begin */
/* Function End */



/* ---------------------------------------------------------*/
/* ⇩⇩⇩⇩⇩⇩⇩⇩⇩⇩ Program ⇩⇩⇩⇩⇩⇩⇩⇩⇩⇩ ---------------------------*/
/* ---------------------------------------------------------*/
/* Program Begin */

/** * @brief constructor
 	* @param None
 	* @return None
**	**/
motor_servo_ctl::motor_servo_ctl():
	rclcpp::Node("motor_servo_ctl")
{
	/* create subscriber object for servo_switch */
	_subscriber_motor_servo_switch = this->create_subscription<std_msgs::msg::Bool>(
		topicname_motor_servo_switch,
		rclcpp::QoS(10),
		std::bind(
			&motor_servo_ctl::callback_topic_motor_servo_switch,
			this, std::placeholders::_1)	);

	/* create publisher object for InterfaceMultipleMotors */
	_publisher_motors_info = this->create_publisher<custom_msgs::msg::InterfaceMultipleMotors>(
		topicname_multi_motor_info,
		rclcpp::QoS(10)	);
	
	/*  */
	request_ESMcmd = std::make_shared<uros_interface::srv::ESMcmd::Request>();
	request_ESMcmd->servo_status = true;
	request_ESMcmd->mode = 4;
	request_ESMcmd->speed_limit = 20;
	request_ESMcmd->lpf = 10;
	servo_cmd_flag = false;
	motors_info.quantity = motor_quantity;
	motors_info.motor_info.resize(motor_quantity);

	/* create service-client-end object */
	_client_esm_command = this->create_client<uros_interface::srv::ESMcmd>(servicename_toESP_esm_command);
	/* Wait for the service to be available */
	while (!_client_esm_command->wait_for_service(std::chrono::seconds(1)))
	{
		RCLCPP_WARN(this->get_logger(), "Waiting for \"esm_command\" service...");
	}
	RCLCPP_INFO(this->get_logger(), "[motor_servo_ctl] node has been created!");

}
/** * @brief destructor
 	* @param None
 	* @return None
**	**/
motor_servo_ctl::~motor_servo_ctl()
{

}


/** * @brief topic callback function for _subscriber_motor_servo_switch
 	* @param servo_cmd(std_msgs::msg::Bool) servo on/off command
 	* @return None
**	**/
void motor_servo_ctl::callback_topic_motor_servo_switch(std_msgs::msg::Bool cmd)
{
	servo_cmd = cmd;
	RCLCPP_INFO(this->get_logger(), (servo_cmd.data)?"servo_cmd.data true":"servo_cmd.data false");
	RCLCPP_INFO(this->get_logger(), (servo_cmd_last.data)?"servo_cmd_last.data true":"servo_cmd_last.data false");
	if(servo_cmd.data != servo_cmd_last.data)
	{
		servo_cmd_flag = servo_cmd.data;
		RCLCPP_INFO(this->get_logger(), "motor servo state command has been changed to %d", servo_cmd.data);
		servo_cmd_last = servo_cmd;
	}
	
	for(int i=0;i<motor_quantity;i++)
	{
		motors_info.motor_info[i].servo_state = servo_cmd.data;
	}
}

/** * @brief get servo command
 	* @param None
 	* @return bool : servo command status
**	**/
bool motor_servo_ctl::get_servo_cmd(void)
{
	return servo_cmd_flag;
}
/** * @brief set servo to on
 	* @param None
 	* @return None
**	**/
void motor_servo_ctl::set_servo_on(void)
{
	result_ESMcmd = _client_esm_command->async_send_request(request_ESMcmd);

}
/** * @brief 
 	* @param None
 	* @return None
**	**/
int motor_servo_ctl::wait_esm_command_result(void)
{
	servo_cmd_flag = false;
	if (result_ESMcmd->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
	{
		RCLCPP_INFO(this->get_logger(), "Received ESM state: %d", result_ESMcmd->get()->esm_state);
		return 0;
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "No response received from ESMcmd service.");
		return 1;
	}
}

/** * @brief 
 	* @param None
 	* @return None
**	**/
void motor_servo_ctl::publish_motor_info_servostate(void)
{
	for(int i=0;i<motor_quantity;i++)
	{
		motors_info.motor_info[i].id = i+1;
		motors_info.motor_info[i].servo_state = servo_cmd.data;
	}
	_publisher_motors_info->publish(motors_info);
}



/* Program End */
/* ---------------------------------------------------------*/
/* ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ Program ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ ---------------------------*/
/* ---------------------------------------------------------*/


/* ***** END OF classAPI_motor_servo_ctl.cpp ***** */