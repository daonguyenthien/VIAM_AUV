/** @file
 *    @brief MAVLink comm protocol testsuite generated from viamlab_auv.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef VIAMLAB_AUV_TESTSUITE_H
#define VIAMLAB_AUV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_viamlab_auv(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_viamlab_auv(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_viam_odometry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_ODOMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_odometry_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0
    };
    mavlink_viam_odometry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.vroll = packet_in.vroll;
        packet1.vpitch = packet_in.vpitch;
        packet1.vyaw = packet_in.vyaw;
        packet1.ax = packet_in.ax;
        packet1.ay = packet_in.ay;
        packet1.az = packet_in.az;
        packet1.aroll = packet_in.aroll;
        packet1.apitch = packet_in.apitch;
        packet1.ayaw = packet_in.ayaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_odometry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_odometry_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw , packet1.ax , packet1.ay , packet1.az , packet1.aroll , packet1.apitch , packet1.ayaw );
    mavlink_msg_viam_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_odometry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw , packet1.ax , packet1.ay , packet1.az , packet1.aroll , packet1.apitch , packet1.ayaw );
    mavlink_msg_viam_odometry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_odometry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_odometry_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw , packet1.ax , packet1.ay , packet1.az , packet1.aroll , packet1.apitch , packet1.ayaw );
    mavlink_msg_viam_odometry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_SETPOINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_setpoint_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0
    };
    mavlink_viam_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.vroll = packet_in.vroll;
        packet1.vpitch = packet_in.vpitch;
        packet1.vyaw = packet_in.vyaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_setpoint_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_setpoint_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw );
    mavlink_msg_viam_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw );
    mavlink_msg_viam_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_setpoint_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw , packet1.vx , packet1.vy , packet1.vz , packet1.vroll , packet1.vpitch , packet1.vyaw );
    mavlink_msg_viam_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_motors_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_motors_command_t packet_in = {
        963497464,45.0,73.0,101.0,129.0
    };
    mavlink_viam_motors_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.thruster_speed = packet_in.thruster_speed;
        packet1.rudder_angle = packet_in.rudder_angle;
        packet1.mass_shifter_position = packet_in.mass_shifter_position;
        packet1.piston_position = packet_in.piston_position;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_motors_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_motors_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_motors_command_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.thruster_speed , packet1.rudder_angle , packet1.mass_shifter_position , packet1.piston_position );
    mavlink_msg_viam_motors_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_motors_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.thruster_speed , packet1.rudder_angle , packet1.mass_shifter_position , packet1.piston_position );
    mavlink_msg_viam_motors_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_motors_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_motors_command_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.thruster_speed , packet1.rudder_angle , packet1.mass_shifter_position , packet1.piston_position );
    mavlink_msg_viam_motors_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_error(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_ERROR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_error_t packet_in = {
        963497464,45.0,73.0
    };
    mavlink_viam_error_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.along_track = packet_in.along_track;
        packet1.cross_track = packet_in.cross_track;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_error_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_error_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.along_track , packet1.cross_track );
    mavlink_msg_viam_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_error_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.along_track , packet1.cross_track );
    mavlink_msg_viam_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_error_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_error_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.along_track , packet1.cross_track );
    mavlink_msg_viam_error_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_thruster_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_thruster_status_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_viam_thruster_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_thruster_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_thruster_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_thruster_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_thruster_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_thruster_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_thruster_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_thruster_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_thruster_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_thruster_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_mass_shifter_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_MASS_SHIFTER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_mass_shifter_status_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_viam_mass_shifter_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_MASS_SHIFTER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_MASS_SHIFTER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_mass_shifter_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_mass_shifter_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_mass_shifter_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_mass_shifter_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_mass_shifter_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_mass_shifter_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_mass_shifter_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_mass_shifter_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_mass_shifter_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_piston_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_PISTON_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_piston_status_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_viam_piston_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_PISTON_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_PISTON_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_piston_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_piston_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_piston_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_piston_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_piston_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_piston_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_piston_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_piston_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_viam_piston_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_board_arm1_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_board_arm1_status_t packet_in = {
        963497464,45.0,73.0,101.0,53,120,187,254
    };
    mavlink_viam_board_arm1_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.altimeter_in_metres = packet_in.altimeter_in_metres;
        packet1.altimeter_in_feet = packet_in.altimeter_in_feet;
        packet1.altimeter_in_fathoms = packet_in.altimeter_in_fathoms;
        packet1.ls_piston_athead = packet_in.ls_piston_athead;
        packet1.ls_piston_attail = packet_in.ls_piston_attail;
        packet1.ls_mass_shifter_athead = packet_in.ls_mass_shifter_athead;
        packet1.ls_mass_shifter_attail = packet_in.ls_mass_shifter_attail;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm1_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_board_arm1_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm1_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.ls_piston_athead , packet1.ls_piston_attail , packet1.ls_mass_shifter_athead , packet1.ls_mass_shifter_attail , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms );
    mavlink_msg_viam_board_arm1_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm1_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.ls_piston_athead , packet1.ls_piston_attail , packet1.ls_mass_shifter_athead , packet1.ls_mass_shifter_attail , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms );
    mavlink_msg_viam_board_arm1_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_board_arm1_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm1_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.ls_piston_athead , packet1.ls_piston_attail , packet1.ls_mass_shifter_athead , packet1.ls_mass_shifter_attail , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms );
    mavlink_msg_viam_board_arm1_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_board_arm2_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_viam_board_arm2_status_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_viam_board_arm2_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.rudder_position = packet_in.rudder_position;
        packet1.rudder_speed = packet_in.rudder_speed;
        packet1.rudder_load = packet_in.rudder_load;
        packet1.rudder_voltage = packet_in.rudder_voltage;
        packet1.rudder_temperature = packet_in.rudder_temperature;
        packet1.keller_pa3_pressure = packet_in.keller_pa3_pressure;
        packet1.keller_pa3_temperature = packet_in.keller_pa3_temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm2_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_viam_board_arm2_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm2_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature );
    mavlink_msg_viam_board_arm2_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm2_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature );
    mavlink_msg_viam_board_arm2_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_viam_board_arm2_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_viam_board_arm2_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature );
    mavlink_msg_viam_board_arm2_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viamlab_auv(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_viam_odometry(system_id, component_id, last_msg);
    mavlink_test_viam_setpoint(system_id, component_id, last_msg);
    mavlink_test_viam_motors_command(system_id, component_id, last_msg);
    mavlink_test_viam_error(system_id, component_id, last_msg);
    mavlink_test_viam_thruster_status(system_id, component_id, last_msg);
    mavlink_test_viam_mass_shifter_status(system_id, component_id, last_msg);
    mavlink_test_viam_piston_status(system_id, component_id, last_msg);
    mavlink_test_viam_board_arm1_status(system_id, component_id, last_msg);
    mavlink_test_viam_board_arm2_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // VIAMLAB_AUV_TESTSUITE_H
