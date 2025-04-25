/** ******************************************************
	* @file		03_service_two_int_add.cpp
	* @author	Tsai,Li-chun
	******************************************************
**	**/


/* System Includes ------------------------------------------*/
/* System Includes Begin */
#include <memory>
/* System Includes End */
/* User Includes --------------------------------------------*/
/* User Includes Begin */
#include "rclcpp/rclcpp.hpp"
#include "uros_interface/srv/es_mcmd.hpp"
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

/** * @brief  service 服務中斷函式
 	* @param request service所需輸入參數
 	* @param response service返回參數
 	* @return Node
**	**/
void add(std::shared_ptr<uros_interface::srv::ESMcmd::Request> request,
		 std::shared_ptr<uros_interface::srv::ESMcmd::Response> response)
{
	response->esm_state = -99;
	// response->sum = request->a + request->b;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request, mode: %d" " lpf: %d",request->mode,request->lpf);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sneding back reponse: [%d]\n",response->esm_state);
}

/** * @brief  Program entry point.
 	* @param argc(int) : Number of input parameters
 	* @param argv(int) : input parameters
 	* @return (int) Program Error.
**	**/
int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("esm_command_node");
	// std::shared_ptr<rclcpp::Node> nn = std::make_shared<rclcpp::Node>("add_two_ints_service");
	rclcpp::Service<uros_interface::srv::ESMcmd>::SharedPtr service
		= node->create_service<uros_interface::srv::ESMcmd>("esm_command",&add);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}

/* Program End */
/* ---------------------------------------------------------*/
/* ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ Program ⇧⇧⇧⇧⇧⇧⇧⇧⇧⇧ ---------------------------*/
/* ---------------------------------------------------------*/


/* ***** END OF 03_service_two_int_add.cpp ***** */