/** ******************************************************
	* @file		motor_servo_ctl_node.cpp
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

/* ESM command result flag */
bool result_ESMcmd_flag = false;		

/* Variables End */


/* Function -------------------------------------------------*/
/* Function Begin */
/* Function End */



/* ---------------------------------------------------------*/
/* ⇩⇩⇩⇩⇩⇩⇩⇩⇩⇩ Program ⇩⇩⇩⇩⇩⇩⇩⇩⇩⇩ ---------------------------*/
/* ---------------------------------------------------------*/
/* Program Begin */

/** * @brief Program entry point.
 	* @param argc(int) Number of input parameters
 	* @param argv(int) input parameters
 	* @return (int) Program Error.
**	**/
int main(int argc, char* argv[])
{
	/* initialization ROS2 Node */
	rclcpp::init(argc,argv);
	/* create motor_servo_ctl object */
	std::shared_ptr<motor_servo_ctl> msc = std::make_shared<motor_servo_ctl>();
	rclcpp::Rate delay(std::chrono::milliseconds(1));

	/* main loop, 按下ctrl+C跳出 */
	while( rclcpp::ok() )
	{
		/* update object execution */
		if(msc->get_servo_cmd() == true)
		{
			result_ESMcmd_flag = true;
			msc->set_servo_on();
		}
		if(result_ESMcmd_flag)
		{
			if(msc->wait_esm_command_result() == 0)
			result_ESMcmd_flag = false;
		}
		// msc->publish_motor_info_servostate();
		rclcpp::spin_some(msc);
		delay.sleep();
	}

	/* 關閉Node */
	rclcpp::shutdown();
}

/* Program End */
/* ---------------------------------------------------------*/
/* ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ Program ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ ---------------------------*/
/* ---------------------------------------------------------*/


/* ***** END OF motor_servo_ctl_node.cpp ***** */