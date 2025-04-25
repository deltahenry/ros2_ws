/** ******************************************************
	* @file		classAPI_motor_servo_ctl.hpp
	* @author	Tsai,Li-chun
	******************************************************
**/

/* Define to prevent recursive inclusi ----------------------*/
#ifndef __classAPI_motor_servo_ctl_HPP__
#define __classAPI_motor_servo_ctl_HPP__


/* System Includes ------------------------------------------*/
/* System Includes Begin */
#include <optional>
/* System Includes End */
/* User Includes --------------------------------------------*/
/* User Includes Begin */
#include "define_table.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_msgs/msg/interface_multiple_motors.hpp"
#include "uros_interface/srv/es_mcmd.hpp"
/* User Includes End */


/* Define ---------------------------------------------------*/
/* Define Begin */
/* Define End */


/* Extern Typedef -------------------------------------------*/
/* Extern Typedef Begin */

// using ServiceResponseFuture = rclcpp::Client<uros_interface::srv::ESMcmd>::SharedFuture;

/* Extern Typedef End */


/* Extern Class -------------------------------------------*/
/* Extern Class Begin */


/** * @brief motor servo on/off control object, inheritance from rclcpp::Node
 	* @param None
 	* @return None
**	**/
class motor_servo_ctl : public rclcpp::Node
{
private:
/* Class --------------------------------------------------*/
	/* declare subscription object, used for subscription servo_switch command */
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriber_motor_servo_switch;
	/* declare publishing object, used for publishing MultipleMotors data */
	rclcpp::Publisher<custom_msgs::msg::InterfaceMultipleMotors>::SharedPtr _publisher_motors_info;
	/* declare service client end object, used to call esm_command service in the ESP32 */
	rclcpp::Client<uros_interface::srv::ESMcmd>::SharedPtr _client_esm_command;
	/* declare MultipleMotors data type object */
	custom_msgs::msg::InterfaceMultipleMotors motors_info;
	/* declare SingleMotor data type object */
	custom_msgs::msg::InterfaceSingleMotor motor_info;
	/* declare service-client-end request paretemeter */
	std::shared_ptr<uros_interface::srv::ESMcmd::Request> request_ESMcmd;
	/* declare service-client-end response paretemeter */
	std::optional<rclcpp::Client<uros_interface::srv::ESMcmd>::FutureAndRequestId> result_ESMcmd;
	/* variable used store motor servo commands */
	std_msgs::msg::Bool servo_cmd, servo_cmd_last;
/* Variables ----------------------------------------------*/
	bool servo_cmd_flag;
/* Function -----------------------------------------------*/
	/* declare topic callback function for _subscriber_motor_servo_switch */
	void callback_topic_motor_servo_switch(std_msgs::msg::Bool cmd);

public:
/* setup -------------------------------------------------*/
	/* constructor */
	motor_servo_ctl();
	/* destructor */
	~motor_servo_ctl();

	bool get_servo_cmd(void);
	void set_servo_on(void);
	int wait_esm_command_result(void);
	void publish_motor_info_servostate(void);

/* Class --------------------------------------------------*/
/* Variables ----------------------------------------------*/
/* Function -----------------------------------------------*/

};

/* Extern Class End */


/* Extern Variables -----------------------------------------*/
/* Extern Variables Begin */
/* Extern Variables End */


/* Function -------------------------------------------------*/
/* Function Begin */
/* Function End */


#endif /*__classAPI_motor_servo_ctl_HPP__ */

/* ***** END OF classAPI_motor_servo_ctl.HPP ***** */