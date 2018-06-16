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
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include "mavlink/common/mavlink.h"
#include "ellipse/EllipseDetectorYaed.h"

extern bool stable, updateellipse, getlocalposition, drop;
extern int TargetNum;
extern coordinate droptarget;
extern vector<target> target_ellipse_position, ellipse_T, ellipse_F;

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_POSITION     0b0000110111111000
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_FORCE        0b0000111000111111
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT_YAW_RATE     0b0000010111111111
#define Machine_Num 1


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// helper functions
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
//void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);
//

void set_global_position(int32_t x, int32_t y, int32_t z, mavlink_set_position_target_global_int_t &sp);
void set_global_velocity(float vx, float vy, float vz, mavlink_set_position_target_global_int_t &sp);
//void set_global_acceleration(float ax, float ay, float az, mavlink_set_position_target_global_int_t &sp);
void set_global_yaw(float yaw, mavlink_set_position_target_global_int_t &sp);
void set_global_yaw_rate(float yaw_rate, mavlink_set_position_target_global_int_t &sp);

float Distance(float x,float y,float z,float x1,float y1,float z1);
float XYDistance(float x, float y, float x1, float y1);
float D2R(uint16_t ghdg);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);

void* start_WL_read_thread(void *args);
void* start_WL_write_thread(void *args);
// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t setmode;
    uint64_t command_long;
    uint64_t mission_item;
    uint64_t command_ack;
    uint64_t param_value;
    uint64_t statustext;
    uint64_t  mission_current;
    uint64_t mission_count;
    uint64_t mission_ack;

    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
        setmode = 0;
        command_long = 0;
        mission_item = 0;
        command_ack = 0;
        param_value = 0;
        statustext = 0;
        mission_current = 0;
        mission_count = 0;
        mission_ack = 0;
    }

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    //Param_Value
    mavlink_set_mode_t set_mode;

    //Command_Long
    mavlink_command_long_t command_long;

    //Mission_Item
    mavlink_mission_item_t mission_item;

    //COMMAND_ACK
    mavlink_command_ack_t command_ack;

    mavlink_param_value_t param_value;

    mavlink_statustext_t statustext;

    mavlink_mission_current_t mission_current;

    mavlink_mission_count_t mission_count;
    mavlink_mission_ack_t mission_ack;
    mavlink_mission_item_reached_t mission_item_reached;

    // System Parameters?

    // Time Stamps
    Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 */
class Autopilot_Interface
{

public:

    Autopilot_Interface();
    Autopilot_Interface(Serial_Port *serial_port_,Serial_Port *WL_port_);
    ~Autopilot_Interface();

    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;
//    uint8_t Machine_Num ;

    char WL_reading;
    char WL_writing;
    uint64_t WL_write_count;

    bool getposition = 0;

    int system_id;
    int autopilot_id;
    int companion_id;

    Mavlink_Messages current_messages;
    Mavlink_Messages Inter_message;
    mavlink_set_position_target_local_ned_t initial_position;
    mavlink_set_position_target_global_int_t initial_global_position;
    mavlink_global_position_int_t global_position;
    mavlink_local_position_ned_t local_position;

    //void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
    void update_local_setpoint(mavlink_set_position_target_local_ned_t setpoint);

    void update_global_setpoint(mavlink_set_position_target_global_int_t set_global_point);

    int Servo_Control(float ServoId, float PWM_Value);

    void read_messages();
    int  write_message(mavlink_message_t message);

    void WL_read_messages();
    int  WL_write_message(mavlink_message_t message);
    int  Send_WL_Global_Position(int Target_machine,mavlink_global_position_int_t Target_Global_Position);

    void enable_offboard_control();
    void Set_Mode(unsigned int custom);
    void disable_offboard_control();

    void start();
    void stop();

    void start_read_thread();
    void start_write_thread(void);

    void start_WL_read();
    void start_WL_write(void);

    void handle_quit( int sig );

    int Throw(float yaw,int TNum);
    int ThrowF(float yaw,int32_t lat,int32_t lon, int Num);


private:

    Serial_Port *serial_port;
    Serial_Port *WL_port;

    bool time_to_exit;

    pthread_t read_tid;
    pthread_t write_tid;

    pthread_t WL_read;
    pthread_t WL_write;

    //mavlink_set_position_target_local_ned_t current_setpoint;
    mavlink_set_position_target_local_ned_t current_local_setpoint;
    mavlink_set_position_target_global_int_t current_global_setpoint;

    void read_thread();
    void write_thread(void);

    void WL_read_thread();
    void WL_write_thread(void);


    int toggle_offboard_control( bool flag );
//	void write_setpoint();

    void write_global_setpoint();

    void write_local_setpoint();

};

/*将当前时刻看到的所有可能为目标的椭圆存放在容器中*/
void possible_ellipse(Autopilot_Interface& api, vector<coordinate>& ellipse_out, vector<target>& target_ellipse);
void resultTF(Autopilot_Interface& api, vector<target>& ellipse_in, vector<target>& ellipse_1, vector<target>& ellipse_0);
void getdroptarget(Autopilot_Interface& api, coordinate& droptarget, vector<coordinate>& ellipse_out);
void realtarget(Autopilot_Interface& api, coordinate& cam, float& x, float& y);
void OptimizEllipse(vector<Ellipse>& ellipse_out, vector<Ellipse>& ellipses_in);
void filtellipse(Autopilot_Interface& api, vector<Ellipse>& ellipseok, vector<Ellipse>& ellipse_big);
#endif // AUTOPILOT_INTERFACE_H_


