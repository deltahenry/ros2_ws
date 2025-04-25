/** ******************************************************
	* @file		classAPI_motor_running_ctl.hpp
	* @author	Tsai,Li-chun
	******************************************************
**/

/* Define to prevent recursive inclusi ----------------------*/
#ifndef __classAPI_motor_running_ctl_HPP__
#define __classAPI_motor_running_ctl_HPP__


/* System Includes ------------------------------------------*/
/* System Includes Begin */
/* System Includes End */
/* User Includes --------------------------------------------*/
/* User Includes Begin */
#include "define_table.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_msgs/msg/interface_multiple_motors.hpp"
#include "uros_interface/msg/joint2_d_arr.hpp"
/* User Includes End */


/* Define ---------------------------------------------------*/
/* Define Begin */
/* Define End */


/* Extern Typedef -------------------------------------------*/
/* Extern Typedef Begin */
/* Extern Typedef End */


/* Extern Class -------------------------------------------*/
/* Extern Class Begin */

/** * @brief motor servo on/off control object, inheritance from rclcpp::Node
 	* @param None
 	* @return None
**	**/
class motor_running_ctl : public rclcpp::Node
{
private:
/* Class --------------------------------------------------*/
	/* declare publishing object, used for publishing MultipleMotors data */
	rclcpp::Publisher<custom_msgs::msg::InterfaceMultipleMotors>::SharedPtr _publisher_motors_info;
	/* declare publishing object, used for publishing theta_command data(ESP32) */
	rclcpp::Publisher<uros_interface::msg::Joint2DArr>::SharedPtr _publisher_ESP32_theta_command;
	/* declare subscription object, used for subscription motor_position_ref */
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _subscriber_motor_position_ref;
	/* declare subscription object, used for subscription servo_switch command */
	rclcpp::Subscription<uros_interface::msg::Joint2DArr>::SharedPtr _subscriber_ESP32_position_feedback;
	/* declare MultipleMotors data type object */
	custom_msgs::msg::InterfaceMultipleMotors motors_info;
	/* declare SingleMotor data type object */
	custom_msgs::msg::InterfaceSingleMotor motor_info;
	/* declare ESP32_position_cmd data type object */
	uros_interface::msg::Joint2DArr ESP32_position_cmd, ESP32_position_fb;

/* Variables ----------------------------------------------*/
/* Function -----------------------------------------------*/

	/* declare topic callback function for _subscriber_motor_position_ref */
	void callback_topic_motor_position_ref(std_msgs::msg::Float32MultiArray cmd);
	/* declare topic callback function for _subscriber_motor_position_ref */
	void callback_topic_ESP32_motor_position_ref(std_msgs::msg::Float32MultiArray cmd);

public:
/* setup -------------------------------------------------*/
	/* constructor */
	motor_running_ctl();
	/* destructor */
	~motor_running_ctl();
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


#endif /*__classAPI_motor_running_ctl_HPP__ */

/* ***** END OF classAPI_motor_running_ctl.HPP ***** */