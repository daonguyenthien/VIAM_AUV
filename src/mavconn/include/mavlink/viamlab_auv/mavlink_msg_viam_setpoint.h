#pragma once
// MESSAGE VIAM_SETPOINT PACKING

#define MAVLINK_MSG_ID_VIAM_SETPOINT 13001

MAVPACKED(
typedef struct __mavlink_viam_setpoint_t {
 uint32_t time_boot_ms; /*< [ms] */
 float x; /*< [m] */
 float y; /*< [m] */
 float z; /*< [m] */
 float roll; /*< [rad] */
 float pitch; /*< [rad] */
 float yaw; /*< [rad] */
 float vx; /*< [m/s] */
 float vy; /*< [m/s] */
 float vz; /*< [m/s] */
 float vroll; /*< [rad/s] */
 float vpitch; /*< [rad/s] */
 float vyaw; /*< [rad/s] */
}) mavlink_viam_setpoint_t;

#define MAVLINK_MSG_ID_VIAM_SETPOINT_LEN 52
#define MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN 52
#define MAVLINK_MSG_ID_13001_LEN 52
#define MAVLINK_MSG_ID_13001_MIN_LEN 52

#define MAVLINK_MSG_ID_VIAM_SETPOINT_CRC 240
#define MAVLINK_MSG_ID_13001_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_SETPOINT { \
    13001, \
    "VIAM_SETPOINT", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_setpoint_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_setpoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_setpoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_setpoint_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_setpoint_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_setpoint_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_setpoint_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_viam_setpoint_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_viam_setpoint_t, vz) }, \
         { "vroll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_viam_setpoint_t, vroll) }, \
         { "vpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_viam_setpoint_t, vpitch) }, \
         { "vyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_viam_setpoint_t, vyaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_SETPOINT { \
    "VIAM_SETPOINT", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_setpoint_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_setpoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_setpoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_setpoint_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_setpoint_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_setpoint_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_setpoint_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_setpoint_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_viam_setpoint_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_viam_setpoint_t, vz) }, \
         { "vroll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_viam_setpoint_t, vroll) }, \
         { "vpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_viam_setpoint_t, vpitch) }, \
         { "vyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_viam_setpoint_t, vyaw) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param x [m] 
 * @param y [m] 
 * @param z [m] 
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param vx [m/s] 
 * @param vy [m/s] 
 * @param vz [m/s] 
 * @param vroll [rad/s] 
 * @param vpitch [rad/s] 
 * @param vyaw [rad/s] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_float(buf, 40, vroll);
    _mav_put_float(buf, 44, vpitch);
    _mav_put_float(buf, 48, vyaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN);
#else
    mavlink_viam_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vroll = vroll;
    packet.vpitch = vpitch;
    packet.vyaw = vyaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
}

/**
 * @brief Pack a viam_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param x [m] 
 * @param y [m] 
 * @param z [m] 
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param vx [m/s] 
 * @param vy [m/s] 
 * @param vz [m/s] 
 * @param vroll [rad/s] 
 * @param vpitch [rad/s] 
 * @param vyaw [rad/s] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float x,float y,float z,float roll,float pitch,float yaw,float vx,float vy,float vz,float vroll,float vpitch,float vyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_float(buf, 40, vroll);
    _mav_put_float(buf, 44, vpitch);
    _mav_put_float(buf, 48, vyaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN);
#else
    mavlink_viam_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vroll = vroll;
    packet.vpitch = vpitch;
    packet.vyaw = vyaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
}

/**
 * @brief Encode a viam_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_setpoint_t* viam_setpoint)
{
    return mavlink_msg_viam_setpoint_pack(system_id, component_id, msg, viam_setpoint->time_boot_ms, viam_setpoint->x, viam_setpoint->y, viam_setpoint->z, viam_setpoint->roll, viam_setpoint->pitch, viam_setpoint->yaw, viam_setpoint->vx, viam_setpoint->vy, viam_setpoint->vz, viam_setpoint->vroll, viam_setpoint->vpitch, viam_setpoint->vyaw);
}

/**
 * @brief Encode a viam_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_setpoint_t* viam_setpoint)
{
    return mavlink_msg_viam_setpoint_pack_chan(system_id, component_id, chan, msg, viam_setpoint->time_boot_ms, viam_setpoint->x, viam_setpoint->y, viam_setpoint->z, viam_setpoint->roll, viam_setpoint->pitch, viam_setpoint->yaw, viam_setpoint->vx, viam_setpoint->vy, viam_setpoint->vz, viam_setpoint->vroll, viam_setpoint->vpitch, viam_setpoint->vyaw);
}

/**
 * @brief Send a viam_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param x [m] 
 * @param y [m] 
 * @param z [m] 
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param vx [m/s] 
 * @param vy [m/s] 
 * @param vz [m/s] 
 * @param vroll [rad/s] 
 * @param vpitch [rad/s] 
 * @param vyaw [rad/s] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_setpoint_send(mavlink_channel_t chan, uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_SETPOINT_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_float(buf, 40, vroll);
    _mav_put_float(buf, 44, vpitch);
    _mav_put_float(buf, 48, vyaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_SETPOINT, buf, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
#else
    mavlink_viam_setpoint_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.vroll = vroll;
    packet.vpitch = vpitch;
    packet.vyaw = vyaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a viam_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_setpoint_send_struct(mavlink_channel_t chan, const mavlink_viam_setpoint_t* viam_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_setpoint_send(chan, viam_setpoint->time_boot_ms, viam_setpoint->x, viam_setpoint->y, viam_setpoint->z, viam_setpoint->roll, viam_setpoint->pitch, viam_setpoint->yaw, viam_setpoint->vx, viam_setpoint->vy, viam_setpoint->vz, viam_setpoint->vroll, viam_setpoint->vpitch, viam_setpoint->vyaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_SETPOINT, (const char *)viam_setpoint, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_float(buf, 28, vx);
    _mav_put_float(buf, 32, vy);
    _mav_put_float(buf, 36, vz);
    _mav_put_float(buf, 40, vroll);
    _mav_put_float(buf, 44, vpitch);
    _mav_put_float(buf, 48, vyaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_SETPOINT, buf, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
#else
    mavlink_viam_setpoint_t *packet = (mavlink_viam_setpoint_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->vroll = vroll;
    packet->vpitch = vpitch;
    packet->vyaw = vyaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_VIAM_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN, MAVLINK_MSG_ID_VIAM_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_SETPOINT UNPACKING


/**
 * @brief Get field time_boot_ms from viam_setpoint message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_setpoint_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from viam_setpoint message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_setpoint_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from viam_setpoint message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_setpoint_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from viam_setpoint message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_setpoint_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from viam_setpoint message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_setpoint_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from viam_setpoint message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_setpoint_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from viam_setpoint message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_setpoint_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vx from viam_setpoint message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vy from viam_setpoint message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field vz from viam_setpoint message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vroll from viam_setpoint message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vroll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vpitch from viam_setpoint message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vpitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vyaw from viam_setpoint message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_setpoint_get_vyaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a viam_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param viam_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_setpoint_decode(const mavlink_message_t* msg, mavlink_viam_setpoint_t* viam_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_setpoint->time_boot_ms = mavlink_msg_viam_setpoint_get_time_boot_ms(msg);
    viam_setpoint->x = mavlink_msg_viam_setpoint_get_x(msg);
    viam_setpoint->y = mavlink_msg_viam_setpoint_get_y(msg);
    viam_setpoint->z = mavlink_msg_viam_setpoint_get_z(msg);
    viam_setpoint->roll = mavlink_msg_viam_setpoint_get_roll(msg);
    viam_setpoint->pitch = mavlink_msg_viam_setpoint_get_pitch(msg);
    viam_setpoint->yaw = mavlink_msg_viam_setpoint_get_yaw(msg);
    viam_setpoint->vx = mavlink_msg_viam_setpoint_get_vx(msg);
    viam_setpoint->vy = mavlink_msg_viam_setpoint_get_vy(msg);
    viam_setpoint->vz = mavlink_msg_viam_setpoint_get_vz(msg);
    viam_setpoint->vroll = mavlink_msg_viam_setpoint_get_vroll(msg);
    viam_setpoint->vpitch = mavlink_msg_viam_setpoint_get_vpitch(msg);
    viam_setpoint->vyaw = mavlink_msg_viam_setpoint_get_vyaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_VIAM_SETPOINT_LEN;
        memset(viam_setpoint, 0, MAVLINK_MSG_ID_VIAM_SETPOINT_LEN);
    memcpy(viam_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
