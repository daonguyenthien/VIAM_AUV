/** @file
 *	@brief MAVLink comm testsuite protocol generated from viamlab_auv.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "viamlab_auv.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(viamlab_auv, VIAM_ODOMETRY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_ODOMETRY packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;
    packet_in.roll = 213.0;
    packet_in.pitch = 241.0;
    packet_in.yaw = 269.0;
    packet_in.vx = 297.0;
    packet_in.vy = 325.0;
    packet_in.vz = 353.0;
    packet_in.vroll = 381.0;
    packet_in.vpitch = 409.0;
    packet_in.vyaw = 437.0;
    packet_in.ax = 465.0;
    packet_in.ay = 493.0;
    packet_in.az = 521.0;
    packet_in.aroll = 549.0;
    packet_in.apitch = 577.0;
    packet_in.ayaw = 605.0;

    mavlink::viamlab_auv::msg::VIAM_ODOMETRY packet1{};
    mavlink::viamlab_auv::msg::VIAM_ODOMETRY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.vroll, packet2.vroll);
    EXPECT_EQ(packet1.vpitch, packet2.vpitch);
    EXPECT_EQ(packet1.vyaw, packet2.vyaw);
    EXPECT_EQ(packet1.ax, packet2.ax);
    EXPECT_EQ(packet1.ay, packet2.ay);
    EXPECT_EQ(packet1.az, packet2.az);
    EXPECT_EQ(packet1.aroll, packet2.aroll);
    EXPECT_EQ(packet1.apitch, packet2.apitch);
    EXPECT_EQ(packet1.ayaw, packet2.ayaw);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_ODOMETRY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_odometry_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0, 381.0, 409.0, 437.0, 465.0, 493.0, 521.0, 549.0, 577.0, 605.0
    };

    mavlink::viamlab_auv::msg::VIAM_ODOMETRY packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;
    packet_in.roll = 213.0;
    packet_in.pitch = 241.0;
    packet_in.yaw = 269.0;
    packet_in.vx = 297.0;
    packet_in.vy = 325.0;
    packet_in.vz = 353.0;
    packet_in.vroll = 381.0;
    packet_in.vpitch = 409.0;
    packet_in.vyaw = 437.0;
    packet_in.ax = 465.0;
    packet_in.ay = 493.0;
    packet_in.az = 521.0;
    packet_in.aroll = 549.0;
    packet_in.apitch = 577.0;
    packet_in.ayaw = 605.0;

    mavlink::viamlab_auv::msg::VIAM_ODOMETRY packet2{};

    mavlink_msg_viam_odometry_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.vroll, packet2.vroll);
    EXPECT_EQ(packet_in.vpitch, packet2.vpitch);
    EXPECT_EQ(packet_in.vyaw, packet2.vyaw);
    EXPECT_EQ(packet_in.ax, packet2.ax);
    EXPECT_EQ(packet_in.ay, packet2.ay);
    EXPECT_EQ(packet_in.az, packet2.az);
    EXPECT_EQ(packet_in.aroll, packet2.aroll);
    EXPECT_EQ(packet_in.apitch, packet2.apitch);
    EXPECT_EQ(packet_in.ayaw, packet2.ayaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_SETPOINT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_SETPOINT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.vx = 213.0;
    packet_in.vy = 241.0;
    packet_in.vz = 269.0;
    packet_in.vroll = 297.0;
    packet_in.vpitch = 325.0;
    packet_in.vyaw = 353.0;

    mavlink::viamlab_auv::msg::VIAM_SETPOINT packet1{};
    mavlink::viamlab_auv::msg::VIAM_SETPOINT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.vroll, packet2.vroll);
    EXPECT_EQ(packet1.vpitch, packet2.vpitch);
    EXPECT_EQ(packet1.vyaw, packet2.vyaw);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_SETPOINT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_setpoint_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0, 325.0, 353.0
    };

    mavlink::viamlab_auv::msg::VIAM_SETPOINT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.roll = 129.0;
    packet_in.pitch = 157.0;
    packet_in.yaw = 185.0;
    packet_in.vx = 213.0;
    packet_in.vy = 241.0;
    packet_in.vz = 269.0;
    packet_in.vroll = 297.0;
    packet_in.vpitch = 325.0;
    packet_in.vyaw = 353.0;

    mavlink::viamlab_auv::msg::VIAM_SETPOINT packet2{};

    mavlink_msg_viam_setpoint_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.vroll, packet2.vroll);
    EXPECT_EQ(packet_in.vpitch, packet2.vpitch);
    EXPECT_EQ(packet_in.vyaw, packet2.vyaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_MOTORS_COMMAND)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_MOTORS_COMMAND packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.thruster_speed = 45.0;
    packet_in.rudder_angle = 73.0;
    packet_in.mass_shifter_position = 101.0;
    packet_in.piston_position = 129.0;

    mavlink::viamlab_auv::msg::VIAM_MOTORS_COMMAND packet1{};
    mavlink::viamlab_auv::msg::VIAM_MOTORS_COMMAND packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.thruster_speed, packet2.thruster_speed);
    EXPECT_EQ(packet1.rudder_angle, packet2.rudder_angle);
    EXPECT_EQ(packet1.mass_shifter_position, packet2.mass_shifter_position);
    EXPECT_EQ(packet1.piston_position, packet2.piston_position);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_MOTORS_COMMAND)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_motors_command_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0
    };

    mavlink::viamlab_auv::msg::VIAM_MOTORS_COMMAND packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.thruster_speed = 45.0;
    packet_in.rudder_angle = 73.0;
    packet_in.mass_shifter_position = 101.0;
    packet_in.piston_position = 129.0;

    mavlink::viamlab_auv::msg::VIAM_MOTORS_COMMAND packet2{};

    mavlink_msg_viam_motors_command_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.thruster_speed, packet2.thruster_speed);
    EXPECT_EQ(packet_in.rudder_angle, packet2.rudder_angle);
    EXPECT_EQ(packet_in.mass_shifter_position, packet2.mass_shifter_position);
    EXPECT_EQ(packet_in.piston_position, packet2.piston_position);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_ERROR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_ERROR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.along_track = 45.0;
    packet_in.cross_track = 73.0;

    mavlink::viamlab_auv::msg::VIAM_ERROR packet1{};
    mavlink::viamlab_auv::msg::VIAM_ERROR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.along_track, packet2.along_track);
    EXPECT_EQ(packet1.cross_track, packet2.cross_track);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_ERROR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_error_t packet_c {
         963497464, 45.0, 73.0
    };

    mavlink::viamlab_auv::msg::VIAM_ERROR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.along_track = 45.0;
    packet_in.cross_track = 73.0;

    mavlink::viamlab_auv::msg::VIAM_ERROR packet2{};

    mavlink_msg_viam_error_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.along_track, packet2.along_track);
    EXPECT_EQ(packet_in.cross_track, packet2.cross_track);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_THRUSTER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_THRUSTER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.motor_duty = 45.0;
    packet_in.motor_temp_on_chip = 73.0;
    packet_in.motor_temp_ambient = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

    mavlink::viamlab_auv::msg::VIAM_THRUSTER_STATUS packet1{};
    mavlink::viamlab_auv::msg::VIAM_THRUSTER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_THRUSTER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_thruster_status_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::viamlab_auv::msg::VIAM_THRUSTER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.motor_duty = 45.0;
    packet_in.motor_temp_on_chip = 73.0;
    packet_in.motor_temp_ambient = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

    mavlink::viamlab_auv::msg::VIAM_THRUSTER_STATUS packet2{};

    mavlink_msg_viam_thruster_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_MASS_SHIFTER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_MASS_SHIFTER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::viamlab_auv::msg::VIAM_MASS_SHIFTER_STATUS packet1{};
    mavlink::viamlab_auv::msg::VIAM_MASS_SHIFTER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_MASS_SHIFTER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_mass_shifter_status_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::viamlab_auv::msg::VIAM_MASS_SHIFTER_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::viamlab_auv::msg::VIAM_MASS_SHIFTER_STATUS packet2{};

    mavlink_msg_viam_mass_shifter_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_PISTON_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_PISTON_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::viamlab_auv::msg::VIAM_PISTON_STATUS packet1{};
    mavlink::viamlab_auv::msg::VIAM_PISTON_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_PISTON_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_piston_status_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::viamlab_auv::msg::VIAM_PISTON_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::viamlab_auv::msg::VIAM_PISTON_STATUS packet2{};

    mavlink_msg_viam_piston_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_BOARD_ARM1_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM1_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ls_piston_athead = 53;
    packet_in.ls_piston_attail = 120;
    packet_in.ls_mass_shifter_athead = 187;
    packet_in.ls_mass_shifter_attail = 254;
    packet_in.altimeter_in_metres = 45.0;
    packet_in.altimeter_in_feet = 73.0;
    packet_in.altimeter_in_fathoms = 101.0;

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM1_STATUS packet1{};
    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM1_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.ls_piston_athead, packet2.ls_piston_athead);
    EXPECT_EQ(packet1.ls_piston_attail, packet2.ls_piston_attail);
    EXPECT_EQ(packet1.ls_mass_shifter_athead, packet2.ls_mass_shifter_athead);
    EXPECT_EQ(packet1.ls_mass_shifter_attail, packet2.ls_mass_shifter_attail);
    EXPECT_EQ(packet1.altimeter_in_metres, packet2.altimeter_in_metres);
    EXPECT_EQ(packet1.altimeter_in_feet, packet2.altimeter_in_feet);
    EXPECT_EQ(packet1.altimeter_in_fathoms, packet2.altimeter_in_fathoms);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_BOARD_ARM1_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_board_arm1_status_t packet_c {
         963497464, 45.0, 73.0, 101.0, 53, 120, 187, 254
    };

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM1_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ls_piston_athead = 53;
    packet_in.ls_piston_attail = 120;
    packet_in.ls_mass_shifter_athead = 187;
    packet_in.ls_mass_shifter_attail = 254;
    packet_in.altimeter_in_metres = 45.0;
    packet_in.altimeter_in_feet = 73.0;
    packet_in.altimeter_in_fathoms = 101.0;

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM1_STATUS packet2{};

    mavlink_msg_viam_board_arm1_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.ls_piston_athead, packet2.ls_piston_athead);
    EXPECT_EQ(packet_in.ls_piston_attail, packet2.ls_piston_attail);
    EXPECT_EQ(packet_in.ls_mass_shifter_athead, packet2.ls_mass_shifter_athead);
    EXPECT_EQ(packet_in.ls_mass_shifter_attail, packet2.ls_mass_shifter_attail);
    EXPECT_EQ(packet_in.altimeter_in_metres, packet2.altimeter_in_metres);
    EXPECT_EQ(packet_in.altimeter_in_feet, packet2.altimeter_in_feet);
    EXPECT_EQ(packet_in.altimeter_in_fathoms, packet2.altimeter_in_fathoms);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(viamlab_auv, VIAM_BOARD_ARM2_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM2_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.rudder_position = 45.0;
    packet_in.rudder_speed = 73.0;
    packet_in.rudder_load = 101.0;
    packet_in.rudder_voltage = 129.0;
    packet_in.rudder_temperature = 157.0;
    packet_in.keller_pa3_pressure = 185.0;
    packet_in.keller_pa3_temperature = 213.0;

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM2_STATUS packet1{};
    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM2_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.rudder_position, packet2.rudder_position);
    EXPECT_EQ(packet1.rudder_speed, packet2.rudder_speed);
    EXPECT_EQ(packet1.rudder_load, packet2.rudder_load);
    EXPECT_EQ(packet1.rudder_voltage, packet2.rudder_voltage);
    EXPECT_EQ(packet1.rudder_temperature, packet2.rudder_temperature);
    EXPECT_EQ(packet1.keller_pa3_pressure, packet2.keller_pa3_pressure);
    EXPECT_EQ(packet1.keller_pa3_temperature, packet2.keller_pa3_temperature);
}

#ifdef TEST_INTEROP
TEST(viamlab_auv_interop, VIAM_BOARD_ARM2_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_viam_board_arm2_status_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM2_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.rudder_position = 45.0;
    packet_in.rudder_speed = 73.0;
    packet_in.rudder_load = 101.0;
    packet_in.rudder_voltage = 129.0;
    packet_in.rudder_temperature = 157.0;
    packet_in.keller_pa3_pressure = 185.0;
    packet_in.keller_pa3_temperature = 213.0;

    mavlink::viamlab_auv::msg::VIAM_BOARD_ARM2_STATUS packet2{};

    mavlink_msg_viam_board_arm2_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.rudder_position, packet2.rudder_position);
    EXPECT_EQ(packet_in.rudder_speed, packet2.rudder_speed);
    EXPECT_EQ(packet_in.rudder_load, packet2.rudder_load);
    EXPECT_EQ(packet_in.rudder_voltage, packet2.rudder_voltage);
    EXPECT_EQ(packet_in.rudder_temperature, packet2.rudder_temperature);
    EXPECT_EQ(packet_in.keller_pa3_pressure, packet2.keller_pa3_pressure);
    EXPECT_EQ(packet_in.keller_pa3_temperature, packet2.keller_pa3_temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
