/** ******************************************************
	* @file		define_table.h
	* @author	Tsai,Li-chun
	******************************************************
**/

/* Define to prevent recursive inclusi ----------------------*/
#ifndef __define_table_H
#define __define_table_H

#ifdef __cplusplus
extern "C" {
#endif


/* System Includes ------------------------------------------*/
/* System Includes Begin */
/* System Includes End */
/* User Includes --------------------------------------------*/
/* User Includes Begin */
/* User Includes End */


/* Define ---------------------------------------------------*/
/* Define Begin */

#define topicname_motor_servo_switch "servo_switch"
#define topicname_multi_motor_info "multi_motor_info"
	#define topicname_motors_info "motors_info"
	#define topicname_motors_servo_state "motors_servo_state"
#define topicname_motor_position_ref "motor_position_ref"
#define topicname_toESP_theta_command "theta_command"
#define topicname_toESP_theta_feedback "theta_feedback"
#define topicname_toESP_esm_alarm "esm_alarm"


#define servicename_toESP_esm_command "esm_command"

#define motor_quantity 7

/* Define End */


/* Extern Typedef -------------------------------------------*/
/* Extern Typedef Begin */
/* Extern Typedef End */


/* Extern Variables -----------------------------------------*/
/* Extern Variables Begin */
/* Extern Variables End */


/* Function -------------------------------------------------*/
/* Function Begin */
/* Function End */


#ifdef __cplusplus
}
#endif

#endif /* __define_table_H */

/* ***** END OF define_table.H ***** */