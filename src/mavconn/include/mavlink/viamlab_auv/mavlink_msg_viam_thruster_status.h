#pragma once
// MESSAGE VIAM_THRUSTER_STATUS PACKING

#define MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS 13004

MAVPACKED(
typedef struct __mavlink_viam_thruster_status_t {
 uint32_t time_boot_ms; /*< [ms] */
 float motor_duty; /*< [%] */
 float motor_temp_on_chip; /*< [oC] */
 float motor_temp_ambient; /*< [oC] */
 float motor_current; /*< [mA] */
 float motor_rspeed; /*< [rpm] */
 float motor_dspeed; /*< [rpm] */
}) mavlink_viam_thruster_status_t;

#define MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN 28
#define MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN 28
#define MAVLINK_MSG_ID_13004_LEN 28
#define MAVLINK_MSG_ID_13004_MIN_LEN 28

#define MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC 223
#define MAVLINK_MSG_ID_13004_CRC 223



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_THRUSTER_STATUS { \
    13004, \
    "VIAM_THRUSTER_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_thruster_status_t, time_boot_ms) }, \
         { "motor_duty", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_thruster_status_t, motor_duty) }, \
         { "motor_temp_on_chip", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_thruster_status_t, motor_temp_on_chip) }, \
         { "motor_temp_ambient", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_thruster_status_t, motor_temp_ambient) }, \
         { "motor_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_thruster_status_t, motor_current) }, \
         { "motor_rspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_thruster_status_t, motor_rspeed) }, \
         { "motor_dspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_thruster_status_t, motor_dspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_THRUSTER_STATUS { \
    "VIAM_THRUSTER_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_thruster_status_t, time_boot_ms) }, \
         { "motor_duty", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_thruster_status_t, motor_duty) }, \
         { "motor_temp_on_chip", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_thruster_status_t, motor_temp_on_chip) }, \
         { "motor_temp_ambient", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_thruster_status_t, motor_temp_ambient) }, \
         { "motor_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_thruster_status_t, motor_current) }, \
         { "motor_rspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_thruster_status_t, motor_rspeed) }, \
         { "motor_dspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_thruster_status_t, motor_dspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_thruster_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param motor_duty [%] 
 * @param motor_temp_on_chip [oC] 
 * @param motor_temp_ambient [oC] 
 * @param motor_current [mA] 
 * @param motor_rspeed [rpm] 
 * @param motor_dspeed [rpm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_thruster_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN);
#else
    mavlink_viam_thruster_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
}

/**
 * @brief Pack a viam_thruster_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param motor_duty [%] 
 * @param motor_temp_on_chip [oC] 
 * @param motor_temp_ambient [oC] 
 * @param motor_current [mA] 
 * @param motor_rspeed [rpm] 
 * @param motor_dspeed [rpm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_thruster_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float motor_duty,float motor_temp_on_chip,float motor_temp_ambient,float motor_current,float motor_rspeed,float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN);
#else
    mavlink_viam_thruster_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
}

/**
 * @brief Encode a viam_thruster_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_thruster_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_thruster_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_thruster_status_t* viam_thruster_status)
{
    return mavlink_msg_viam_thruster_status_pack(system_id, component_id, msg, viam_thruster_status->time_boot_ms, viam_thruster_status->motor_duty, viam_thruster_status->motor_temp_on_chip, viam_thruster_status->motor_temp_ambient, viam_thruster_status->motor_current, viam_thruster_status->motor_rspeed, viam_thruster_status->motor_dspeed);
}

/**
 * @brief Encode a viam_thruster_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_thruster_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_thruster_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_thruster_status_t* viam_thruster_status)
{
    return mavlink_msg_viam_thruster_status_pack_chan(system_id, component_id, chan, msg, viam_thruster_status->time_boot_ms, viam_thruster_status->motor_duty, viam_thruster_status->motor_temp_on_chip, viam_thruster_status->motor_temp_ambient, viam_thruster_status->motor_current, viam_thruster_status->motor_rspeed, viam_thruster_status->motor_dspeed);
}

/**
 * @brief Send a viam_thruster_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param motor_duty [%] 
 * @param motor_temp_on_chip [oC] 
 * @param motor_temp_ambient [oC] 
 * @param motor_current [mA] 
 * @param motor_rspeed [rpm] 
 * @param motor_dspeed [rpm] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_thruster_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS, buf, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
#else
    mavlink_viam_thruster_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
#endif
}

/**
 * @brief Send a viam_thruster_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_thruster_status_send_struct(mavlink_channel_t chan, const mavlink_viam_thruster_status_t* viam_thruster_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_thruster_status_send(chan, viam_thruster_status->time_boot_ms, viam_thruster_status->motor_duty, viam_thruster_status->motor_temp_on_chip, viam_thruster_status->motor_temp_ambient, viam_thruster_status->motor_current, viam_thruster_status->motor_rspeed, viam_thruster_status->motor_dspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS, (const char *)viam_thruster_status, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_thruster_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS, buf, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
#else
    mavlink_viam_thruster_status_t *packet = (mavlink_viam_thruster_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->motor_duty = motor_duty;
    packet->motor_temp_on_chip = motor_temp_on_chip;
    packet->motor_temp_ambient = motor_temp_ambient;
    packet->motor_current = motor_current;
    packet->motor_rspeed = motor_rspeed;
    packet->motor_dspeed = motor_dspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS, (const char *)packet, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_THRUSTER_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from viam_thruster_status message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_thruster_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field motor_duty from viam_thruster_status message
 *
 * @return [%] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_duty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field motor_temp_on_chip from viam_thruster_status message
 *
 * @return [oC] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_temp_on_chip(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor_temp_ambient from viam_thruster_status message
 *
 * @return [oC] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_temp_ambient(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field motor_current from viam_thruster_status message
 *
 * @return [mA] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field motor_rspeed from viam_thruster_status message
 *
 * @return [rpm] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_rspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field motor_dspeed from viam_thruster_status message
 *
 * @return [rpm] 
 */
static inline float mavlink_msg_viam_thruster_status_get_motor_dspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a viam_thruster_status message into a struct
 *
 * @param msg The message to decode
 * @param viam_thruster_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_thruster_status_decode(const mavlink_message_t* msg, mavlink_viam_thruster_status_t* viam_thruster_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_thruster_status->time_boot_ms = mavlink_msg_viam_thruster_status_get_time_boot_ms(msg);
    viam_thruster_status->motor_duty = mavlink_msg_viam_thruster_status_get_motor_duty(msg);
    viam_thruster_status->motor_temp_on_chip = mavlink_msg_viam_thruster_status_get_motor_temp_on_chip(msg);
    viam_thruster_status->motor_temp_ambient = mavlink_msg_viam_thruster_status_get_motor_temp_ambient(msg);
    viam_thruster_status->motor_current = mavlink_msg_viam_thruster_status_get_motor_current(msg);
    viam_thruster_status->motor_rspeed = mavlink_msg_viam_thruster_status_get_motor_rspeed(msg);
    viam_thruster_status->motor_dspeed = mavlink_msg_viam_thruster_status_get_motor_dspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN;
        memset(viam_thruster_status, 0, MAVLINK_MSG_ID_VIAM_THRUSTER_STATUS_LEN);
    memcpy(viam_thruster_status, _MAV_PAYLOAD(msg), len);
#endif
}
