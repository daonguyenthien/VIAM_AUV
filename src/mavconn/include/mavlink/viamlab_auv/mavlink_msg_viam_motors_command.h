#pragma once
// MESSAGE VIAM_MOTORS_COMMAND PACKING

#define MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND 13002

MAVPACKED(
typedef struct __mavlink_viam_motors_command_t {
 uint32_t time_boot_ms; /*< [ms] */
 float thruster_speed; /*< [rpm] */
 float rudder_angle; /*< [deg] */
 float mass_shifter_position; /*< [mm] */
 float piston_position; /*< [mm] */
}) mavlink_viam_motors_command_t;

#define MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN 20
#define MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN 20
#define MAVLINK_MSG_ID_13002_LEN 20
#define MAVLINK_MSG_ID_13002_MIN_LEN 20

#define MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC 210
#define MAVLINK_MSG_ID_13002_CRC 210



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_MOTORS_COMMAND { \
    13002, \
    "VIAM_MOTORS_COMMAND", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_motors_command_t, time_boot_ms) }, \
         { "thruster_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_motors_command_t, thruster_speed) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_motors_command_t, rudder_angle) }, \
         { "mass_shifter_position", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_motors_command_t, mass_shifter_position) }, \
         { "piston_position", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_motors_command_t, piston_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_MOTORS_COMMAND { \
    "VIAM_MOTORS_COMMAND", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_motors_command_t, time_boot_ms) }, \
         { "thruster_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_motors_command_t, thruster_speed) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_motors_command_t, rudder_angle) }, \
         { "mass_shifter_position", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_motors_command_t, mass_shifter_position) }, \
         { "piston_position", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_motors_command_t, piston_position) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_motors_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param thruster_speed [rpm] 
 * @param rudder_angle [deg] 
 * @param mass_shifter_position [mm] 
 * @param piston_position [mm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_motors_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float thruster_speed, float rudder_angle, float mass_shifter_position, float piston_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thruster_speed);
    _mav_put_float(buf, 8, rudder_angle);
    _mav_put_float(buf, 12, mass_shifter_position);
    _mav_put_float(buf, 16, piston_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN);
#else
    mavlink_viam_motors_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thruster_speed = thruster_speed;
    packet.rudder_angle = rudder_angle;
    packet.mass_shifter_position = mass_shifter_position;
    packet.piston_position = piston_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
}

/**
 * @brief Pack a viam_motors_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param thruster_speed [rpm] 
 * @param rudder_angle [deg] 
 * @param mass_shifter_position [mm] 
 * @param piston_position [mm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_motors_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float thruster_speed,float rudder_angle,float mass_shifter_position,float piston_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thruster_speed);
    _mav_put_float(buf, 8, rudder_angle);
    _mav_put_float(buf, 12, mass_shifter_position);
    _mav_put_float(buf, 16, piston_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN);
#else
    mavlink_viam_motors_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thruster_speed = thruster_speed;
    packet.rudder_angle = rudder_angle;
    packet.mass_shifter_position = mass_shifter_position;
    packet.piston_position = piston_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
}

/**
 * @brief Encode a viam_motors_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_motors_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_motors_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_motors_command_t* viam_motors_command)
{
    return mavlink_msg_viam_motors_command_pack(system_id, component_id, msg, viam_motors_command->time_boot_ms, viam_motors_command->thruster_speed, viam_motors_command->rudder_angle, viam_motors_command->mass_shifter_position, viam_motors_command->piston_position);
}

/**
 * @brief Encode a viam_motors_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_motors_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_motors_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_motors_command_t* viam_motors_command)
{
    return mavlink_msg_viam_motors_command_pack_chan(system_id, component_id, chan, msg, viam_motors_command->time_boot_ms, viam_motors_command->thruster_speed, viam_motors_command->rudder_angle, viam_motors_command->mass_shifter_position, viam_motors_command->piston_position);
}

/**
 * @brief Send a viam_motors_command message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param thruster_speed [rpm] 
 * @param rudder_angle [deg] 
 * @param mass_shifter_position [mm] 
 * @param piston_position [mm] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_motors_command_send(mavlink_channel_t chan, uint32_t time_boot_ms, float thruster_speed, float rudder_angle, float mass_shifter_position, float piston_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thruster_speed);
    _mav_put_float(buf, 8, rudder_angle);
    _mav_put_float(buf, 12, mass_shifter_position);
    _mav_put_float(buf, 16, piston_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND, buf, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
#else
    mavlink_viam_motors_command_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thruster_speed = thruster_speed;
    packet.rudder_angle = rudder_angle;
    packet.mass_shifter_position = mass_shifter_position;
    packet.piston_position = piston_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
#endif
}

/**
 * @brief Send a viam_motors_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_motors_command_send_struct(mavlink_channel_t chan, const mavlink_viam_motors_command_t* viam_motors_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_motors_command_send(chan, viam_motors_command->time_boot_ms, viam_motors_command->thruster_speed, viam_motors_command->rudder_angle, viam_motors_command->mass_shifter_position, viam_motors_command->piston_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND, (const char *)viam_motors_command, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_motors_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float thruster_speed, float rudder_angle, float mass_shifter_position, float piston_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thruster_speed);
    _mav_put_float(buf, 8, rudder_angle);
    _mav_put_float(buf, 12, mass_shifter_position);
    _mav_put_float(buf, 16, piston_position);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND, buf, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
#else
    mavlink_viam_motors_command_t *packet = (mavlink_viam_motors_command_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->thruster_speed = thruster_speed;
    packet->rudder_angle = rudder_angle;
    packet->mass_shifter_position = mass_shifter_position;
    packet->piston_position = piston_position;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND, (const char *)packet, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_MOTORS_COMMAND UNPACKING


/**
 * @brief Get field time_boot_ms from viam_motors_command message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_motors_command_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field thruster_speed from viam_motors_command message
 *
 * @return [rpm] 
 */
static inline float mavlink_msg_viam_motors_command_get_thruster_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rudder_angle from viam_motors_command message
 *
 * @return [deg] 
 */
static inline float mavlink_msg_viam_motors_command_get_rudder_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mass_shifter_position from viam_motors_command message
 *
 * @return [mm] 
 */
static inline float mavlink_msg_viam_motors_command_get_mass_shifter_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field piston_position from viam_motors_command message
 *
 * @return [mm] 
 */
static inline float mavlink_msg_viam_motors_command_get_piston_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a viam_motors_command message into a struct
 *
 * @param msg The message to decode
 * @param viam_motors_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_motors_command_decode(const mavlink_message_t* msg, mavlink_viam_motors_command_t* viam_motors_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_motors_command->time_boot_ms = mavlink_msg_viam_motors_command_get_time_boot_ms(msg);
    viam_motors_command->thruster_speed = mavlink_msg_viam_motors_command_get_thruster_speed(msg);
    viam_motors_command->rudder_angle = mavlink_msg_viam_motors_command_get_rudder_angle(msg);
    viam_motors_command->mass_shifter_position = mavlink_msg_viam_motors_command_get_mass_shifter_position(msg);
    viam_motors_command->piston_position = mavlink_msg_viam_motors_command_get_piston_position(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN;
        memset(viam_motors_command, 0, MAVLINK_MSG_ID_VIAM_MOTORS_COMMAND_LEN);
    memcpy(viam_motors_command, _MAV_PAYLOAD(msg), len);
#endif
}
