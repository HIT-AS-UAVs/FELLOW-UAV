/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

bool stable = false, updateellipse = false, getlocalposition = false, drop = false;
int TargetNum = 0;
coordinate droptarget;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

// -------------------------------------------------------------------------------
//  计算三维距离
// -------------------------------------------------------------------------------
float Distance(float x,float y,float z,float x1,float y1,float z1)
{
    //如果是经纬度*10^7，转化为m
    if (x > 10000)
    {
        x = x*18.5/1000;
        x1 = x1*18.5/1000;
        y = y*14/1000;
        y1 = y1*14/1000;
        z = z/1000.0;
        z1 = z1/1000.0;
    }
    float Distance = fabsf(x - x1)+fabsf(y - y1)+fabsf(z - z1);
    return Distance ;
}

// -------------------------------------------------------------------------------
//  计算二维距离
// -------------------------------------------------------------------------------
float XYDistance(float x, float y, float x1, float y1)
{
    if (x > 10000) {
        x = x * 18.5 / 1000;
        x1 = x1 * 18.5 / 1000;
        y = y * 14 / 1000;
        y1 = y1 * 14 / 1000;
    }
    float Distance = fabsf(x - x1)+fabsf(y - y1);
    return Distance ;
}


// --------------------------------------------------------------------------------
// 角度转化为弧度
// --------------------------------------------------------------------------------
float D2R(uint16_t ghdg)
{
    float deg = (float)ghdg/100.0;
    float Rad = deg*3.14159/180;
    return Rad;

}


// ----------------------------------------------------------------------------------
//   设置目标点的函数
// ----------------------------------------------------------------------------------

// choose one of the next three
/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	        //MAV_FRAME_BODY_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	//sp.coordinate_frame = MAV_FRAME_BODY_NED;
	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}

// the next two need to be called after one of the above
/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


/*
 * 设置全局目标坐标点，坐标系GLOBAL_RELATIVE_ALT_INT
 */
void
set_global_position(int32_t x,int32_t y,int32_t z,mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_POSITION;
	sp.coordinate_frame =
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

	sp.lat_int   = x;
	sp.lon_int   = y;
	sp.alt   = z;
}

/*
 * 设置global_velocity
 */
void
set_global_velocity(float vx, float vy, float vz, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask =
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY ;

	sp.coordinate_frame =
	        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
}


// the next two need to be called after one of the above

/*
 * 设置指向
 */
void
set_global_yaw(float yaw, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &= MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE;
	sp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_global_yaw_rate(float yaw_rate, mavlink_set_position_target_global_int_t &sp)
{
	sp.type_mask &=
			MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_, Serial_Port *WL_port_)
{
	// initialize attributes
	write_count = 0;
	//
    WL_write_count = 0;

    //飞控连接读写线程标志
	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running

	// 机间连接读写线程标志
    WL_reading = 0;
    WL_writing = 0;

    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit   = false;  // flag to signal thread exit


	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

    WL_read   = 0;  //机间通信read
    WL_write  = 0;  //机间通信write

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object
    WL_port = WL_port_;

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Global Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_global_setpoint(mavlink_set_position_target_global_int_t set_global_point)
{
	current_global_setpoint = set_global_point;
	write_global_setpoint();
}


// ------------------------------------------------------------------------------
//	驱动舵机：<PWM_Value:1100-1900>
//	ServoId：AUX_OUT1-6 对应148-153/9-14
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
Servo_Control(float ServoId, float PWM_Value)
{
    mavlink_command_long_t ServoCom = { 0 };
    ServoCom.target_system = 01;
    ServoCom.target_component = 01;
    ServoCom.command = MAV_CMD_DO_SET_SERVO;
    ServoCom.param1 = ServoId;
    ServoCom.param2 = PWM_Value;
    ServoCom.confirmation = 0;
    mavlink_message_t RCC;
    mavlink_msg_command_long_encode(255, 190, &RCC, &ServoCom);
    // Send the message
    int ServoLen = serial_port->write_message(RCC);
    usleep(100);
    return ServoLen;
}


// ------------------------------------------------------------------------------
//   Update Local Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_local_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_local_setpoint = setpoint;
	write_local_setpoint();
}



// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
//					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
//					printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					getlocalposition = true;
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
//                    std::cout<<"local_position.x:"<<current_messages.local_position_ned.x<<std::endl
//                             <<"local_position.y:"<<current_messages.local_position_ned.y<<std::endl
//                             <<"local_position.z:"<<current_messages.local_position_ned.z<<std::endl;
                    local_position = current_messages.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
//					printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
//                    std::cout<<"lat:"<<current_messages.global_position_int.lat<<std::endl;
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                    global_position = current_messages.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}
				case MAVLINK_MSG_ID_SET_MODE:
				{
					printf("MAVLINK_MSG_ID_SET_MODE\n");
					mavlink_msg_set_mode_decode(&message, &(current_messages.set_mode));
					std::cout<<"base_mode:"<<current_messages.set_mode.base_mode<<std::endl
							 <<"custom_mode:"<<current_messages.set_mode.custom_mode<<std::endl;
					current_messages.time_stamps.setmode = get_time_usec();
					this_timestamps.setmode = current_messages.time_stamps.setmode;
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_LONG:
				{
					printf("MAVLINK_MSG_ID_COMMAND_LONG\n");
					mavlink_msg_command_long_decode(&message, &(current_messages.command_long));
					std::cout<<"command:"<<current_messages.command_long.command<<std::endl
							 <<"confirmation:"<<(float)current_messages.command_long.confirmation<<std::endl
							<<"param1:"<<current_messages.command_long.param1<<std::endl
					<<"param2:"<<current_messages.command_long.param2<<std::endl
					<<"param3:"<<current_messages.command_long.param3<<std::endl
					<<"param4:"<<current_messages.command_long.param4<<std::endl
					<<"param5:"<<current_messages.command_long.param5<<std::endl
					<<"param6:"<<current_messages.command_long.param6<<std::endl
					<<"param7:"<<current_messages.command_long.param7<<std::endl;
					current_messages.time_stamps.command_long = get_time_usec();
					this_timestamps.command_long = current_messages.time_stamps.command_long;
					break;
				}
				case MAVLINK_MSG_ID_MISSION_ITEM:
				{
					printf("MAVLINK_MSG_ID_MISSION_ITEM\n");
					mavlink_msg_mission_item_decode(&message, &(current_messages.mission_item));
					std::cout<<"frame:"<<(float)current_messages.mission_item.frame<<std::endl
							 <<"command:"<<current_messages.mission_item.command<<std::endl
							 <<"current:"<<(float)current_messages.mission_item.current<<std::endl
							 <<"param2:"<<current_messages.mission_item.param2<<std::endl
							 <<"param3:"<<current_messages.mission_item.param3<<std::endl
							 <<"param4:"<<current_messages.mission_item.param4<<std::endl
							 <<"param1:"<<current_messages.mission_item.param1<<std::endl
							 <<"autocontinue:"<<(float)current_messages.mission_item.autocontinue<<std::endl
							 <<"z:"<<current_messages.mission_item.z<<std::endl;
					current_messages.time_stamps.mission_item = get_time_usec();
					this_timestamps.mission_item = current_messages.time_stamps.mission_item;
					break;
				}
				case MAVLINK_MSG_ID_COMMAND_ACK:
                {
                    printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
                    mavlink_msg_command_ack_decode(&message, &(current_messages.command_ack));
                    std::cout<<"command:"<<(float)current_messages.command_ack.command<<std::endl
                             <<"result:"<<(float)current_messages.command_ack.result<<std::endl;
                    current_messages.time_stamps.command_ack = get_time_usec();
                    this_timestamps.command_ack = current_messages.time_stamps.command_ack;
                    break;
                }

				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					printf("MAVLINK ID PARAM_VALUE!\n");
					mavlink_msg_param_value_decode(&message,&(current_messages.param_value));
					this_timestamps.param_value = current_messages.time_stamps.param_value;
					std::cout<<"param_id:"<<current_messages.param_value.param_id<<std::endl
							 <<"param_value:"<<current_messages.param_value.param_value<<std::endl
							 <<"param_type:"<<(float)current_messages.param_value.param_type<<std::endl;
					break;
				}
				case MAVLINK_MSG_ID_STATUSTEXT:
				{
					printf("Mavlink ID statustext!\n");
					mavlink_msg_statustext_decode(&message,&(current_messages.statustext));
					this_timestamps.statustext = current_messages.time_stamps.statustext;
					std::cout<<"severity:"<<(float)current_messages.statustext.severity<<std::endl
							 <<"text:"<<current_messages.statustext.text<<std::endl;
					break;
				}

                case MAVLINK_MSG_ID_MISSION_COUNT:
                {
                    printf("mavlink id mission_count!\n");
                    mavlink_msg_mission_count_decode(&message,&(current_messages.mission_count));
                    std::cout<<"mission count :"<<current_messages.mission_count.count<<std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_MISSION_ACK:
                {
                    printf("mavlink id mission_ack!\n");
                    mavlink_msg_mission_ack_decode(&message,&(current_messages.mission_ack));
                    std::cout<<"mission_ack:"<<(float)current_messages.mission_ack.type<<std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
                {
                    printf("mavlink id mission_item_reached!");
                    mavlink_msg_mission_item_reached_decode(&message,&(current_messages.mission_item_reached));
                    std::cout<<"mission_item_reached seq :"<<current_messages.mission_item_reached.seq<<std::endl;
                    break;
                }


				default:
				{
					printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status != false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
    // do the write
    int len = serial_port->write_message(message);
    // book keep
    write_count++;
    // Done!
    return len;
}

void
Autopilot_Interface::
WL_read_messages()
{
    bool success;               // receive success flag
    bool received_all = false;  // receive only one message
    Time_Stamps this_timestamps;
    while ( !received_all and !time_to_exit )
    {
        mavlink_message_t message;
        success = WL_port->read_message(message);
        if ((message.sysid != 40)&&(message.sysid != Machine_Num))
        {
            break;
        }
        if(success)
        {
            Inter_message.sysid  = message.sysid;
            Inter_message.compid = message.compid;
            switch (message.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
//                    printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(Inter_message.heartbeat));
                    Inter_message.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = Inter_message.time_stamps.heartbeat;
                    printf("uart2 is succeed!\n");
                    break;
                }
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
//                    printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(Inter_message.local_position_ned));
                    Inter_message.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = Inter_message.time_stamps.local_position_ned;
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
//                    printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(Inter_message.global_position_int));
//                    std::cout<<"lat:"<<Inter_message.global_position_int.lat<<std::endl;
                    Inter_message.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int = Inter_message.time_stamps.global_position_int;
//                    global_position = Inter_message.global_position_int;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(Inter_message.position_target_local_ned));
                    Inter_message.time_stamps.position_target_local_ned = get_time_usec();
                    this_timestamps.position_target_local_ned = Inter_message.time_stamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(Inter_message.position_target_global_int));
                    Inter_message.time_stamps.position_target_global_int = get_time_usec();
                    this_timestamps.position_target_global_int = Inter_message.time_stamps.position_target_global_int;
                    break;
                }
                case MAVLINK_MSG_ID_MISSION_ACK:
                {
                    printf("mavlink id mission_ack!\n");
                    mavlink_msg_mission_ack_decode(&message,&(Inter_message.mission_ack));
                    std::cout<<"mission_ack:"<<(float)Inter_message.mission_ack.type<<std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG:
                {
                    printf("MAVLINK_MSG_ID_COMMAND_LONG\n");
                    mavlink_msg_command_long_decode(&message, &(Inter_message.command_long));
                    std::cout<<"command:"<<Inter_message.command_long.command<<std::endl
                             <<"confirmation:"<<(float)Inter_message.command_long.confirmation<<std::endl
                             <<"param1:"<<Inter_message.command_long.param1<<std::endl
                             <<"param2:"<<Inter_message.command_long.param2<<std::endl
                             <<"param3:"<<Inter_message.command_long.param3<<std::endl
                             <<"param4:"<<Inter_message.command_long.param4<<std::endl
                             <<"param5:"<<Inter_message.command_long.param5<<std::endl
                             <<"param6:"<<Inter_message.command_long.param6<<std::endl
                             <<"param7:"<<Inter_message.command_long.param7<<std::endl;
                    Inter_message.time_stamps.command_long = get_time_usec();
                    this_timestamps.command_long = Inter_message.time_stamps.command_long;
                    if((Inter_message.command_long.command==400)&&(Inter_message.command_long.param1 == 0))
                    {
                        mavlink_message_t disarm;
                        Inter_message.command_long.target_system = 1;
                        Inter_message.command_long.target_component = 1;
                        mavlink_msg_command_long_encode(255, 190, &disarm, &Inter_message.command_long);
                        int disarmlen = write_message(disarm);
                    }
                    if((Inter_message.command_long.command == 20))
                    {
                        mavlink_message_t RTLL;
                        Inter_message.command_long.target_system = 01;
                        Inter_message.command_long.target_component = 01;
                        mavlink_msg_command_long_encode(255,190,&RTLL,&Inter_message.command_long);
                        int RT = write_message(RTLL);
                    }
                    break;
                }

                default:
                {
                    printf("Warning, did not handle message id %i\n",message.msgid);
                    break;
                }

            }
        }
        received_all =
                this_timestamps.heartbeat                  &&
                //				this_timestamps.battery_status             &&
                //				this_timestamps.radio_status               &&
                //				this_timestamps.local_position_ned         &&
                //				this_timestamps.global_position_int        &&
                //				this_timestamps.position_target_local_ned  &&
                //				this_timestamps.position_target_global_int &&
                //				this_timestamps.highres_imu                &&
                //				this_timestamps.attitude                   &&
                this_timestamps.sys_status
                ;
        if ( WL_writing != false )
        {
            usleep(100); // look for components of batches at 5kHz
        }

    }
}

// ------------------------------------------------------------------------------
//   WL Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
WL_write_message(mavlink_message_t message)
{
    // do the write
    int len = WL_port->write_message(message);
    // book keep
    WL_write_count++;
    // Done!
    return len;
}
int
Autopilot_Interface::
Send_WL_Global_Position(int Target_machine, mavlink_global_position_int_t Target_Global_Position)
{
    mavlink_message_t Global_messgge;
    mavlink_msg_global_position_int_encode(Target_machine,Target_machine,&Global_messgge,&Target_Global_Position);
    int Glolen = WL_write_message(Global_messgge);
    while(Glolen <= 0)
    {
        printf("fail send wl message! try again! ");
        Glolen = WL_write_message(Global_messgge);
    }
        printf("send wl message succeed!");

    return Glolen;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_global_setpoint()
{
	mavlink_set_position_target_global_int_t sp = current_global_setpoint;
	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_global_int_encode(system_id, companion_id, &message, &sp);
	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	int len = write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_GLOBAL_INT \n");
	else
		printf("%lu Global_POSITION_TARGET  = [ %4f , %4f , %4f ] \n", write_count, (float)sp.lat_int, (float)sp.lon_int, sp.alt);
	std::cout<<sp.type_mask<<std::endl;
	return;
}

void
Autopilot_Interface::
write_local_setpoint()
{
	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_local_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;
	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
	// do the write
	int len = write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	else
		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, sp.x, sp.y, sp.z);
	std::cout<<sp.type_mask<<std::endl;

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ----------------------------------------------------------------------------------
//                                     设置模式
//      STABILIZE=0,    ACRO=1,     ALT_HOLD=2,AUTO=3,GUIDED=4,
//       LOITER=5,      RTL=6,          CIRCLE=7,
//       LAND=9,    DRIFT=11,       SPORT=13,
//       FLIP=14,   AUTOTUNE=15,    POSHOLD=16,
//       BRAKE=17,  HROW=18,       AVOID_ADSB=19,
//       GUIDED_NOGPS=20,            SMART_RTL=21,
// -----------------------------------------------------------------------------------
void
Autopilot_Interface::
Set_Mode(unsigned int custom)
{
    mavlink_set_mode_t Mode_enable = { 0 };
    Mode_enable.base_mode = 1;
    Mode_enable.target_system = 01;
    Mode_enable.custom_mode = custom;
    mavlink_message_t Mode_mes;
    mavlink_msg_set_mode_encode(255, 190, &Mode_mes, &Mode_enable);
    // Send the message
    int lenMode = write_message(Mode_mes);
    if (lenMode <= 0)
    {
        printf("设置模式错误!\n");
    }
    else {
        printf("成功设置模式\n");
    }

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
//		int success = toggle_offboard_control( false );

		// Check the command was written
		if (true )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status
//MAV_CMD_MISSION_START
}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
    //////////////////////自稳模式
    mavlink_set_mode_t com = { 0 };
    com.base_mode = 1;
    com.target_system = 01;
    com.custom_mode = 00;
    // Encode
    mavlink_message_t message;
    mavlink_msg_set_mode_encode(255, 190, &message, &com);
    int len = serial_port->write_message(message);
    usleep(100);
    // Done!

	Servo_Control(10,1250);

    ///////请求数据流(关闭ALL)
    mavlink_request_data_stream_t com1 = { 0 };
    com1.target_system= 01;
    com1.target_component = 01;
    com1.req_stream_id = MAV_DATA_STREAM_ALL;
    com1.req_message_rate = 2;
    com1.start_stop = 0;
    mavlink_message_t message1;
    mavlink_msg_request_data_stream_encode(255, 190, &message1, &com1);
    int len1 = serial_port->write_message(message1);
    usleep(100);

    /////////////////////////////////请求位置数据流
    mavlink_request_data_stream_t comdata = { 0 };
    comdata.req_message_rate = 5;
    comdata.req_stream_id = MAV_DATA_STREAM_POSITION;
    comdata.start_stop = 1;
    comdata.target_system = 1;
    comdata.target_component = 1;
    mavlink_message_t Rmassage;
    mavlink_msg_request_data_stream_encode(255,190,&Rmassage,&comdata);
	int Rlen;
    //重复发送确保指令收到
    for (int i = 0; i < 3; ++i)
    {
    	Rlen = serial_port->write_message(Rmassage);
        usleep(100);
    }
    usleep(20000);

    ////////////////////////////////////////解锁
    mavlink_command_long_t Armdata = { 0 };
    Armdata.target_system= 01;
    Armdata.target_component = 01;
    Armdata.command = MAV_CMD_COMPONENT_ARM_DISARM;
    Armdata.param1 = 1;
    mavlink_message_t Armmes;
    mavlink_msg_command_long_encode(255, 190, &Armmes, &Armdata);
    // Send the message
    int Armlen = serial_port->write_message(Armmes);
    usleep(100);

    //设置成AUTO模式，开始mission
    Set_Mode(03);
    sleep(1);
    Set_Mode(03);

    //////////////////////////////////开始misiion
    mavlink_command_long_t mission_start = { 0 };
    mission_start.target_system= 01;
    mission_start.target_component = 01;
    mission_start.command = 300;
    mission_start.confirmation = 1;
    mission_start.param1 = 1;
    mission_start.param2 = 8;

    mavlink_message_t Mission_starmessage;
    mavlink_msg_command_long_encode(255, 190, &Mission_starmessage, &mission_start);

    // Send the message
    int lenmission = serial_port->write_message(Mission_starmessage);
    if (lenmission <= 0)
    {
        printf("Start Mission error!\n");
    }
    else {
        printf("Start Mission!\n");
    }

    // Done!

    return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::
start()
{
	int result;
	int WL_result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
        fprintf(stderr,"ERROR: serial port not open\n");
        throw 1;
	}

	if (WL_port->status != 1)
    {
        fprintf(stderr,"ERROR: serial port not open\n");
        throw 1;
    }

	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
    WL_result = pthread_create(&WL_read,NULL,&start_WL_read_thread,this);

	if ( result ) throw result;
	if (WL_result) throw WL_result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
//	while ( not ( current_messages.time_stamps.local_position_ned &&
//				  current_messages.time_stamps.attitude            )  )
	while ( not true )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	//初始化global_position
	initial_global_position.lat_int = local_data.global_position_int.lat;
	initial_global_position.lon_int = local_data.global_position_int.lon;
	initial_global_position.alt = local_data.global_position_int.alt;
	initial_global_position.vx = local_data.global_position_int.vx;
	initial_global_position.vy = local_data.global_position_int.vy;
	initial_global_position.vz = local_data.global_position_int.vz;
	initial_global_position.yaw = local_data.global_position_int.hdg;
	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
    WL_result = pthread_create( &WL_write, NULL, &start_WL_write_thread, this );
	if ( result ) throw result;
    if ( WL_result ) throw WL_result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

    pthread_join(WL_read ,NULL);
    pthread_join(WL_write,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();

		return;
	}

}

void
Autopilot_Interface::
start_WL_read()
{

	if ( WL_reading!= 0 )
	{
		fprintf(stderr,"WL read thread already running\n");
		return;
	}
	else
	{
		WL_read_thread();

		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( writing_status != 0 )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}

void
Autopilot_Interface::
start_WL_write(void)
{
	if ( WL_writing != 0 )
	{
		fprintf(stderr,"WL write thread already running\n");
		return;
	}

	else
	{
		WL_write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}

int
Autopilot_Interface::
Throw(float yaw,int Tnum)
{
	int local_alt = -10;
	mavlink_set_position_target_local_ned_t locsp;
	set_position(  target_ellipse_position[TargetNum].x, // [m]
				   target_ellipse_position[TargetNum].y, // [m]
				   local_alt, // [m]
				   locsp);
	set_yaw(yaw,locsp);
	update_local_setpoint(locsp);

	while(((current_messages.local_position_ned.z+11) <= 0)||(XYDistance(current_messages.local_position_ned.x,current_messages.local_position_ned.y,locsp.x,locsp.y) >= 8))
	{
		set_position(  target_ellipse_position[TargetNum].x, // [m]
					   target_ellipse_position[TargetNum].y, // [m]
					   local_alt, // [m]
					   locsp);
		set_yaw(yaw,locsp);
		update_local_setpoint(locsp);
		usleep(200000);
	}



	//执行仍的过程
    drop = true;

    //给响应时间识别小圆,需加判断是否写入目标点
	while(drop)
	{
		float loc = droptarget.locx+droptarget.locy;
		if(loc!=0)
		{
			break;
		}
		else
		{
			usleep(20000);
		}
	}
	int i = 0;

	while (drop)
	{
		float locx = droptarget.locx;
		float locy = droptarget.locy;
		mavlink_local_position_ned_t pos = current_messages.local_position_ned;
		float disx = locx - pos.x;
		float disy = locy - pos.y;
		float adisx = fabsf(disx);
		float adisy	= fabsf(disy);
		if(adisx >= adisy)
		{
			set_velocity(disx/adisx,disy/adisx,0,locsp);
		}
		else
		{
			set_velocity(disx/adisy,disy/adisy,0,locsp);
		}
		set_yaw(yaw, // [rad]
				locsp);
		// SEND THE COMMAND
		update_local_setpoint(locsp);
		mavlink_local_position_ned_t locpos = current_messages.local_position_ned;

		if ((fabsf(locpos.x-droptarget.locx) < 0.5)&&(fabsf(locpos.y-droptarget.locy) < 0.5)&&((locpos.z+10.5)>= 0))
		{
			Set_Mode(05);
			sleep(2);
			Set_Mode(04);
			printf("input drop process!!!\n");
			// ------------------------------------------------------------------------------
			//	驱动舵机：<PWM_Value:1100-1900> 打开：1700、关闭：1250
			//	ServoId：AUX_OUT1-6 对应148-153/9-14
			// ------------------------------------------------------------------------------
			usleep(100000);
			int lenn = Servo_Control(11, 1700);
			Tnum = Tnum + 1;
			sleep(1);
			drop = false;
			break;
		}
		else
		{
			usleep(200000);
		}
    }
    return Tnum;
}

int
Autopilot_Interface::ThrowF(float yaw,int32_t lat,int32_t lon,int Num)
{
	mavlink_set_position_target_global_int_t glosp;
	float hight = 25;
	Set_Mode(05);
	usleep(400);
	Set_Mode(04);
	usleep(400);
	set_global_position(lat,lon,hight,glosp);
	set_global_yaw(yaw,glosp);
	update_global_setpoint(glosp);
	int Targetnum = TargetNum;
	TargetNum = Num;
	while(updateellipse)
	{
		mavlink_global_position_int_t current_global = current_messages.global_position_int;
		float distan = Distance(current_global.lat,current_global.lon,current_global.relative_alt,lat,lon,hight);
		if(distan < 5)
		{
			sleep(2);
			//updateellipse = false;
			break;
		}
		else
		{
			usleep(200000);
		}
	}
	ellipse_F[0].T_N = ellipse_F[0].F_N = ellipse_F[0].possbile = 0;
	ellipse_F[1].T_N = ellipse_F[1].F_N = ellipse_F[1].possbile = 0;
	stable = true;
	int TF = 0;
	while(stable)
	{

		sleep(1);
		TF++;
		if (TF==10)
		{
			int TplusF = target_ellipse_position[TargetNum].T_N + target_ellipse_position[TargetNum].F_N;
			if(TplusF <= 10 )
			{
				break;
			}
			else {
					continue;
			}
		}
		else
		{
			;
		}

	}
	Throw(yaw,2);
	TargetNum = Targetnum;

}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(1000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
//	writing_status = 2;

	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
	}

	// signal end
	writing_status = false;

	return;

}



// ------------------------------------------------------------------------------
//  WL Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
WL_read_thread()
{
    WL_reading = true;

    while ( ! time_to_exit )
    {
        WL_read_messages();
        usleep(1000); // Read batches at 10Hz
    }

    WL_reading = false;

    return;
}


// ------------------------------------------------------------------------------
//  WL Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
WL_write_thread()
{
    // signal startup

    WL_writing = true;

    while ( !time_to_exit )
    {
        usleep(250000);   // Stream at 4Hz
//		write_setpoint();
    }

    // signal end
    WL_writing = false;

    return;

}



// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_WL_read_thread(void *args)
{
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	autopilot_interface->start_WL_read();
}


void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}
void*
start_WL_write_thread(void *args)
{
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_WL_write();

	// done!
	return NULL;
}

// ------------------------------------------------------------------------------
//  将当前时刻看到的所有可能为目标的椭圆存放在容器中
// ------------------------------------------------------------------------------
void possible_ellipse(Autopilot_Interface& api, vector<coordinate>& ellipse_out, vector<target>& target_ellipse){
    float dis = 7;//在室外的参数圆心相距9米内都算一个圆
//	float dis = 0.05;//在室内测试用0.05
    for (auto &p:ellipse_out) {
    	float x_l, y_l;
    	realtarget(api, p, x_l, y_l);
    	p.locx = x_l;
    	p.locy = y_l;
    	if (target_ellipse.size() == 0) {
                target t;
                t.x = p.locx;
                t.y = p.locy;
                t.a = p.a;
                if (p.flag == 1)
                    t.T_N = 1;
                else if (p.flag == 0)
                    t.F_N = 1;
                else {}

                t.possbile = (float) t.T_N / (float) (t.T_N + t.F_N + 0.001);
                target_ellipse.push_back(t);
                continue;
            }
            if(stable == true || updateellipse == true) {
				for (auto i = 0; i < target_ellipse.size(); i++){
					if (abs(p.locx - target_ellipse[i].x) < dis &&
						abs(p.locy - target_ellipse[i].y) < dis) {
						target_ellipse[i].x = p.locx;
						target_ellipse[i].y = p.locy;
						target_ellipse[i].a = p.a;
						if (p.flag == 1)
							target_ellipse[i].T_N = target_ellipse[i].T_N + 1;
						else if (p.flag == 0)
							target_ellipse[i].F_N = target_ellipse[i].F_N + 1;
						else {}
						target_ellipse[i].possbile =
								(float) target_ellipse[i].T_N /
								(float) (target_ellipse[i].T_N + target_ellipse[i].F_N + 0.001);
						break;
					} else
						continue;
			}
            } else {
				for (auto i = 0; i < target_ellipse.size(); i++) {
					if (abs(p.locx - target_ellipse[i].x) < dis &&
						abs(p.locy - target_ellipse[i].y) < dis) {
						target_ellipse[i].x = p.locx;
						target_ellipse[i].y = p.locy;
						target_ellipse[i].a = p.a;
						if (p.flag == 1)
							target_ellipse[i].T_N = target_ellipse[i].T_N + 1;
						else if (p.flag == 0)
							target_ellipse[i].F_N = target_ellipse[i].F_N + 1;
						else {}
						target_ellipse[i].possbile =
								(float) target_ellipse[i].T_N / (float) (target_ellipse[i].T_N + target_ellipse[i].F_N + 0.001);
						break;
					} else if (i != (target_ellipse.size() - 1)) {
						continue;
					} else {
						target t;
						t.x = p.locx;
						t.y = p.locy;
						t.a = p.a;
						if (p.flag == 1)
							t.T_N = 1;
						else if (p.flag == 0)
							t.F_N = 1;
						else {}
						t.possbile = (float) t.T_N / (float) (t.T_N + t.F_N + 0.001);
						target_ellipse.push_back(t);
						break;
					}
				}
			}
    }
    nearellipse(api, target_ellipse);
}
void resultTF(Autopilot_Interface& api, vector<target>& ellipse_in, vector<target>& ellipse_1, vector<target>& ellipse_0){
//	float possobile = 0.5, dis = 0.05;//室内测试设置0.5，0.05， 室外待定
//	uint32_t num = 10;//室内测试设置10，室外待定
	float possobile = 0.4, dis = 7;//室外测试：识别概率大于0.4都算作T，两圆圆心相距9米内都算一个圆
	uint32_t num = 50;//室外测试：识别次数大于50次即可进行TF判断。
	if(ellipse_in.size() == 0){

	} else {
        target p = ellipse_in[0];
        if (p.possbile > possobile && p.T_N > num) {
            stable = false;
            if (ellipse_1.size() == 0) {
                p.lat = api.current_messages.global_position_int.lat;
                p.lon = api.current_messages.global_position_int.lon;
                p.num = TargetNum;
                ellipse_1.push_back(p);
            }
            for (auto t = 0; t < ellipse_1.size(); t++) {
                if ((p.x - ellipse_1[t].x) < dis && (p.y - ellipse_1[t].y) < dis)
                    break;
                else if (t != (ellipse_1.size() - 1))
                    continue;
                else {
                    p.lat = api.current_messages.global_position_int.lat;
                    p.lon = api.current_messages.global_position_int.lon;
                    p.num = TargetNum;
                    ellipse_1.push_back(p);
                    break;
                }

            }
        } else if (p.possbile < possobile && p.F_N > num) {
            stable = false;
            if (ellipse_0.size() == 0) {
                p.lat = api.current_messages.global_position_int.lat;
                p.lon = api.current_messages.global_position_int.lon;
                p.num = TargetNum;
                ellipse_0.push_back(p);
            }
            for (auto f = 0; f < ellipse_0.size(); f++) {
                if ((p.x - ellipse_0[f].x) < dis && (p.y - ellipse_0[f].y) < dis)
                    break;
                else if (f != (ellipse_0.size() - 1))
                    continue;
                else {
                    p.lat = api.current_messages.global_position_int.lat;
                    p.lon = api.current_messages.global_position_int.lon;
                    p.num = TargetNum;
                    ellipse_0.push_back(p);
                    break;
                }
            }
        }
    }
	}


void getdroptarget(Autopilot_Interface& api, coordinate& droptarget, vector<coordinate>& ellipse_out) {
    if (ellipse_out.size() != 0){
    	float dis = 7;
        sort(ellipse_out.begin(),ellipse_out.end());
        float e_x, e_y, locx, locy;
        realtarget(api, ellipse_out[0], e_x, e_y);
        locx = api.current_messages.local_position_ned.x;
        locy = api.current_messages.local_position_ned.y;
        if(abs(e_x - locx) < dis && abs(e_y - locy) < dis){
        	droptarget.locx = e_x;
        	droptarget.locy = e_y;
			cout << "target_x" << droptarget.locx << endl;
			cout << "target_y" << droptarget.locy << endl;
        } else{
			cout << "target_x" << droptarget.locx << endl;
			cout << "target_y" << droptarget.locy << endl;
        }

    } else{
		cout << "target_x" << droptarget.locx << endl;
		cout << "target_y" << droptarget.locy << endl;
    }
}

void realtarget(Autopilot_Interface& api, coordinate& cam, float& x_l, float& y_l){
    int32_t h = -api.current_messages.local_position_ned.z;
//        int32_t h = 25;//桌子高度0.74M
    uint16_t hdg = api.current_messages.global_position_int.hdg;
//        uint16_t hdg = 0;//设置机头方向为正北
    float loc_x = api.current_messages.local_position_ned.x;
    float loc_y = api.current_messages.local_position_ned.y;
    /*在相机坐标系下椭圆圆心的坐标（相机坐标系正东为x，正北为y）*/
    float x = (cam.x - cx) / fx * h;//单位为：m
    float y = -(cam.y - cy) / fy * h;
    //将相机坐标系坐标转换为以摄像头所在中心的导航坐标系下坐标（正东为y,正北为x）
    float x_r = y * cos(hdg * 3.1415926 / 180 / 100) - x * sin(hdg * 3.1415926 / 180 / 100);//单位是:m
    float y_r = x * cos(hdg * 3.1415926 / 180 / 100) + y * sin(hdg * 3.1415926 / 180 / 100);
    x_l = x_r + loc_x;
    y_l = y_r + loc_y;
//	x_l = x_r;
//	y_l = y_r;
}

void OptimizEllipse(vector<Ellipse> &ellipse_out, vector<Ellipse> &ellipses_in){
	float score = 0.7, e = 0.3;
	/***************************去掉评分不佳的椭圆********************************************/
	vector<Ellipse> e0,v1, v2;//存放去掉评分小于0.8的椭圆
	for(auto i = ellipses_in.begin(); i != ellipses_in.end(); ++i){
		if((*i)._score < score || (((*i)._a -(*i)._b) / (*i)._a) > e)
			continue;
		else
			e0.push_back(*i);
	}

	/*************延x轴方向对椭圆由小到大排序**************************/
	int n_e = e0.size();
	for (int i = 0; i < n_e - 1; i++) {
		for (int j = 0; j < n_e - 1 - i; j++) {
			if (e0[j]._xc > e0[j + 1]._xc) {
				swap(e0[j], e0[j + 1]);
			}
		}
	}


	for(auto &p:e0){
		if(v1.size() == 0){
			v1.push_back(p);
		} else{
			for(auto j = 0; j <v1.size(); j++ ){
				if (abs(v1[j]._xc - p._xc) < 20 && abs(v1[j]._yc - p._yc) < 20)
					break;
				else if( j != ( v1.size() - 1)){
					continue;
				} else{
					v1.push_back(p);
					break;
				}
			}
		}
	}
ellipse_out = v1;

}
/*将得到的圆放入vector中，并对其中数量大于一定范围的圆进行下一步处理，以滤除偶然检测出的圆*/
void filtellipse(Autopilot_Interface& api, vector<Ellipse>& ellipseok, vector<Ellipse>& ellipse_big){

	for(auto &p:ellipse_big){
		float dis = 4;
		float locx, locy;
		coordinate cam;
		cam.x = p._xc;
		cam.y = p._yc;
		realtarget(api, cam, locx, locy);
		cam.locx = locx;
		cam.locy = locy;
		if(ellipse_pre.size() == 0){
			cam.num = 1;
			ellipse_pre.push_back(cam);
			continue;
		}
		for(auto i = 0; i < ellipse_pre.size(); i++){
			if(abs(cam.locx - ellipse_pre[i].locx) < dis &&
			   abs(cam.locy - ellipse_pre[i].locy) < dis){
				ellipse_pre[i].locx = cam.locx;
				ellipse_pre[i].locy = cam.locy;
				ellipse_pre[i].x = cam.x;
				ellipse_pre[i].y = cam.y;
				ellipse_pre[i].num = ellipse_pre[i].num + 1;
				break;
			} else if(i != (ellipse_pre.size() - 1)){
				continue;
			} else{
				cam.num = 1;
				ellipse_pre.push_back(cam);
				break;
			}
			}
	}
	uint16_t totle = 0;
	for(auto &p: ellipse_pre){
		totle = totle + p.num;
//		cout<<"ellipse_pre_locx:"<<p.locx<<endl;
//		cout<<"ellipse_pre_locy:"<<p.locy<<endl;
//		cout<<"ellipse_pre_num:"<<ellipse_pre.size()<<endl;
	}

	for(auto &p: ellipse_big){
		for(auto &q: ellipse_pre){
			q.possible = q.num / (totle + 0.0001);
			float disx = (p._xc - q.x) / p._a;
			float disy = (p._yc - q.y) / p._b;
			float thresh = 0.9;//该值应小于1
			if( (disx < thresh && disy < thresh) && (q.possible > 0.1 && q.num > 5)){
				ellipseok.push_back(p);
				break;
			} else
				continue;
		}

	}
/*
	for(auto &p:ellipse_pre){
		cout<<"ellipse_pre_locx:"<<p.locx<<endl;
		cout<<"ellipse_pre_locy:"<<p.locy<<endl;
		cout<<"ellipse_pre_number:"<<p.num<<endl;
		cout<<"ellipse_pre_possible:"<<p.possible<<endl;
		cout<<"ellipse_pre_size:"<<ellipse_pre.size()<<endl;
		cout<<"ellipse_ok_size:"<<ellipseok.size()<<endl;
	}
*/
}

void nearellipse(Autopilot_Interface& api, vector<target>& target_ellipse){
    float locx = api.current_messages.local_position_ned.x;
    float locy = api.current_messages.local_position_ned.y;
    int size = target_ellipse.size();
	if(size == 0 || size == 1){

	} else {
		for (auto i = 0; i < target_ellipse.size() - 1; i++) {
			for (auto j = 0; j < target_ellipse.size() - i - 1; j++){
				float jdis = ellipsedistance(locx, locy, target_ellipse[j].x, target_ellipse[j].y);
				float jplusdis = ellipsedistance(locx, locy, target_ellipse[j + 1].x, target_ellipse[j + 1].y);
				if(jdis > jplusdis){
					swap(target_ellipse[j], target_ellipse[j+1]);
				}
			}
		}
	}


}

float ellipsedistance(float locx, float locy, float e_x, float e_y){
	float x1 = abs(locx - e_x);
	float y1 = abs(locy - e_y);
	float dis = (x1 * x1) + (y1 * y1);
	return dis;
}
