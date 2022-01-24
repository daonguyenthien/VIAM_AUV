#pragma once
// MESSAGE VIAM_ERROR PACKING

#define MAVLINK_MSG_ID_VIAM_ERROR 13003

MAVPACKED(
typedef struct __mavlink_viam_error_t {
 uint32_t time_boot_ms; /*< [ms] */
 float along_track; /*< [m] */
 float cross_track; /*< [m] */
}) mavlink_viam_error_t;

#define MAVLINK_MSG_ID_VIAM_ERROR_LEN 12
#define MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN 12
#define MAVLINK_MSG_ID_13003_LEN 12
#define MAVLINK_MSG_ID_13003_MIN_LEN 12

#define MAVLINK_MSG_ID_VIAM_ERROR_CRC 248
#define MAVLINK_MSG_ID_13003_CRC 248



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_ERROR { \
    13003, \
    "VIAM_ERROR", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_error_t, time_boot_ms) }, \
         { "along_track", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_error_t, along_track) }, \
         { "cross_track", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_error_t, cross_track) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_ERROR { \
    "VIAM_ERROR", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_error_t, time_boot_ms) }, \
         { "along_track", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_error_t, along_track) }, \
         { "cross_track", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_error_t, cross_track) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param along_track [m] 
 * @param cross_track [m] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_error_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float along_track, float cross_track)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, along_track);
    _mav_put_float(buf, 8, cross_track);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_ERROR_LEN);
#else
    mavlink_viam_error_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.along_track = along_track;
    packet.cross_track = cross_track;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_ERROR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
}

/**
 * @brief Pack a viam_error message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param along_track [m] 
 * @param cross_track [m] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_error_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float along_track,float cross_track)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, along_track);
    _mav_put_float(buf, 8, cross_track);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_ERROR_LEN);
#else
    mavlink_viam_error_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.along_track = along_track;
    packet.cross_track = cross_track;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_ERROR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
}

/**
 * @brief Encode a viam_error struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_error_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_error_t* viam_error)
{
    return mavlink_msg_viam_error_pack(system_id, component_id, msg, viam_error->time_boot_ms, viam_error->along_track, viam_error->cross_track);
}

/**
 * @brief Encode a viam_error struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_error_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_error_t* viam_error)
{
    return mavlink_msg_viam_error_pack_chan(system_id, component_id, chan, msg, viam_error->time_boot_ms, viam_error->along_track, viam_error->cross_track);
}

/**
 * @brief Send a viam_error message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param along_track [m] 
 * @param cross_track [m] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_error_send(mavlink_channel_t chan, uint32_t time_boot_ms, float along_track, float cross_track)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, along_track);
    _mav_put_float(buf, 8, cross_track);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ERROR, buf, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
#else
    mavlink_viam_error_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.along_track = along_track;
    packet.cross_track = cross_track;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ERROR, (const char *)&packet, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
#endif
}

/**
 * @brief Send a viam_error message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_error_send_struct(mavlink_channel_t chan, const mavlink_viam_error_t* viam_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_error_send(chan, viam_error->time_boot_ms, viam_error->along_track, viam_error->cross_track);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ERROR, (const char *)viam_error, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_ERROR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_error_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float along_track, float cross_track)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, along_track);
    _mav_put_float(buf, 8, cross_track);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ERROR, buf, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
#else
    mavlink_viam_error_t *packet = (mavlink_viam_error_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->along_track = along_track;
    packet->cross_track = cross_track;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ERROR, (const char *)packet, MAVLINK_MSG_ID_VIAM_ERROR_MIN_LEN, MAVLINK_MSG_ID_VIAM_ERROR_LEN, MAVLINK_MSG_ID_VIAM_ERROR_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_ERROR UNPACKING


/**
 * @brief Get field time_boot_ms from viam_error message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_error_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field along_track from viam_error message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_error_get_along_track(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field cross_track from viam_error message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_error_get_cross_track(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a viam_error message into a struct
 *
 * @param msg The message to decode
 * @param viam_error C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_error_decode(const mavlink_message_t* msg, mavlink_viam_error_t* viam_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_error->time_boot_ms = mavlink_msg_viam_error_get_time_boot_ms(msg);
    viam_error->along_track = mavlink_msg_viam_error_get_along_track(msg);
    viam_error->cross_track = mavlink_msg_viam_error_get_cross_track(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_ERROR_LEN? msg->len : MAVLINK_MSG_ID_VIAM_ERROR_LEN;
        memset(viam_error, 0, MAVLINK_MSG_ID_VIAM_ERROR_LEN);
    memcpy(viam_error, _MAV_PAYLOAD(msg), len);
#endif
}
