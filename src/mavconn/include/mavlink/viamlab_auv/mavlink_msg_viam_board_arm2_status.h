#pragma once
// MESSAGE VIAM_BOARD_ARM2_STATUS PACKING

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS 13008

MAVPACKED(
typedef struct __mavlink_viam_board_arm2_status_t {
 uint32_t time_boot_ms; /*< [ms] */
 float rudder_position; /*< [deg] */
 float rudder_speed; /*< [rpm] */
 float rudder_load; /*< [Nm] */
 float rudder_voltage; /*< [V] */
 float rudder_temperature; /*< [oC] */
 float keller_pa3_pressure; /*< [Pa] */
 float keller_pa3_temperature; /*< [oC] */
}) mavlink_viam_board_arm2_status_t;

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN 32
#define MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN 32
#define MAVLINK_MSG_ID_13008_LEN 32
#define MAVLINK_MSG_ID_13008_MIN_LEN 32

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC 207
#define MAVLINK_MSG_ID_13008_CRC 207



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_BOARD_ARM2_STATUS { \
    13008, \
    "VIAM_BOARD_ARM2_STATUS", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_board_arm2_status_t, time_boot_ms) }, \
         { "rudder_position", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_board_arm2_status_t, rudder_position) }, \
         { "rudder_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_board_arm2_status_t, rudder_speed) }, \
         { "rudder_load", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_board_arm2_status_t, rudder_load) }, \
         { "rudder_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_board_arm2_status_t, rudder_voltage) }, \
         { "rudder_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_board_arm2_status_t, rudder_temperature) }, \
         { "keller_pa3_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_board_arm2_status_t, keller_pa3_pressure) }, \
         { "keller_pa3_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_board_arm2_status_t, keller_pa3_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_BOARD_ARM2_STATUS { \
    "VIAM_BOARD_ARM2_STATUS", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_board_arm2_status_t, time_boot_ms) }, \
         { "rudder_position", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_board_arm2_status_t, rudder_position) }, \
         { "rudder_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_board_arm2_status_t, rudder_speed) }, \
         { "rudder_load", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_board_arm2_status_t, rudder_load) }, \
         { "rudder_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_board_arm2_status_t, rudder_voltage) }, \
         { "rudder_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_board_arm2_status_t, rudder_temperature) }, \
         { "keller_pa3_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_board_arm2_status_t, keller_pa3_pressure) }, \
         { "keller_pa3_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_board_arm2_status_t, keller_pa3_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_board_arm2_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param rudder_position [deg] 
 * @param rudder_speed [rpm] 
 * @param rudder_load [Nm] 
 * @param rudder_voltage [V] 
 * @param rudder_temperature [oC] 
 * @param keller_pa3_pressure [Pa] 
 * @param keller_pa3_temperature [oC] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_board_arm2_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN);
#else
    mavlink_viam_board_arm2_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
}

/**
 * @brief Pack a viam_board_arm2_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param rudder_position [deg] 
 * @param rudder_speed [rpm] 
 * @param rudder_load [Nm] 
 * @param rudder_voltage [V] 
 * @param rudder_temperature [oC] 
 * @param keller_pa3_pressure [Pa] 
 * @param keller_pa3_temperature [oC] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_board_arm2_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float rudder_position,float rudder_speed,float rudder_load,float rudder_voltage,float rudder_temperature,float keller_pa3_pressure,float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN);
#else
    mavlink_viam_board_arm2_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
}

/**
 * @brief Encode a viam_board_arm2_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_board_arm2_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_board_arm2_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_board_arm2_status_t* viam_board_arm2_status)
{
    return mavlink_msg_viam_board_arm2_status_pack(system_id, component_id, msg, viam_board_arm2_status->time_boot_ms, viam_board_arm2_status->rudder_position, viam_board_arm2_status->rudder_speed, viam_board_arm2_status->rudder_load, viam_board_arm2_status->rudder_voltage, viam_board_arm2_status->rudder_temperature, viam_board_arm2_status->keller_pa3_pressure, viam_board_arm2_status->keller_pa3_temperature);
}

/**
 * @brief Encode a viam_board_arm2_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_board_arm2_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_board_arm2_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_board_arm2_status_t* viam_board_arm2_status)
{
    return mavlink_msg_viam_board_arm2_status_pack_chan(system_id, component_id, chan, msg, viam_board_arm2_status->time_boot_ms, viam_board_arm2_status->rudder_position, viam_board_arm2_status->rudder_speed, viam_board_arm2_status->rudder_load, viam_board_arm2_status->rudder_voltage, viam_board_arm2_status->rudder_temperature, viam_board_arm2_status->keller_pa3_pressure, viam_board_arm2_status->keller_pa3_temperature);
}

/**
 * @brief Send a viam_board_arm2_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param rudder_position [deg] 
 * @param rudder_speed [rpm] 
 * @param rudder_load [Nm] 
 * @param rudder_voltage [V] 
 * @param rudder_temperature [oC] 
 * @param keller_pa3_pressure [Pa] 
 * @param keller_pa3_temperature [oC] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_board_arm2_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS, buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
#else
    mavlink_viam_board_arm2_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS, (const char *)&packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
#endif
}

/**
 * @brief Send a viam_board_arm2_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_board_arm2_status_send_struct(mavlink_channel_t chan, const mavlink_viam_board_arm2_status_t* viam_board_arm2_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_board_arm2_status_send(chan, viam_board_arm2_status->time_boot_ms, viam_board_arm2_status->rudder_position, viam_board_arm2_status->rudder_speed, viam_board_arm2_status->rudder_load, viam_board_arm2_status->rudder_voltage, viam_board_arm2_status->rudder_temperature, viam_board_arm2_status->keller_pa3_pressure, viam_board_arm2_status->keller_pa3_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS, (const char *)viam_board_arm2_status, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_board_arm2_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS, buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
#else
    mavlink_viam_board_arm2_status_t *packet = (mavlink_viam_board_arm2_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->rudder_position = rudder_position;
    packet->rudder_speed = rudder_speed;
    packet->rudder_load = rudder_load;
    packet->rudder_voltage = rudder_voltage;
    packet->rudder_temperature = rudder_temperature;
    packet->keller_pa3_pressure = keller_pa3_pressure;
    packet->keller_pa3_temperature = keller_pa3_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS, (const char *)packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_BOARD_ARM2_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from viam_board_arm2_status message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_board_arm2_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rudder_position from viam_board_arm2_status message
 *
 * @return [deg] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_rudder_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rudder_speed from viam_board_arm2_status message
 *
 * @return [rpm] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_rudder_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rudder_load from viam_board_arm2_status message
 *
 * @return [Nm] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_rudder_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rudder_voltage from viam_board_arm2_status message
 *
 * @return [V] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_rudder_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rudder_temperature from viam_board_arm2_status message
 *
 * @return [oC] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_rudder_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field keller_pa3_pressure from viam_board_arm2_status message
 *
 * @return [Pa] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_keller_pa3_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field keller_pa3_temperature from viam_board_arm2_status message
 *
 * @return [oC] 
 */
static inline float mavlink_msg_viam_board_arm2_status_get_keller_pa3_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a viam_board_arm2_status message into a struct
 *
 * @param msg The message to decode
 * @param viam_board_arm2_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_board_arm2_status_decode(const mavlink_message_t* msg, mavlink_viam_board_arm2_status_t* viam_board_arm2_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_board_arm2_status->time_boot_ms = mavlink_msg_viam_board_arm2_status_get_time_boot_ms(msg);
    viam_board_arm2_status->rudder_position = mavlink_msg_viam_board_arm2_status_get_rudder_position(msg);
    viam_board_arm2_status->rudder_speed = mavlink_msg_viam_board_arm2_status_get_rudder_speed(msg);
    viam_board_arm2_status->rudder_load = mavlink_msg_viam_board_arm2_status_get_rudder_load(msg);
    viam_board_arm2_status->rudder_voltage = mavlink_msg_viam_board_arm2_status_get_rudder_voltage(msg);
    viam_board_arm2_status->rudder_temperature = mavlink_msg_viam_board_arm2_status_get_rudder_temperature(msg);
    viam_board_arm2_status->keller_pa3_pressure = mavlink_msg_viam_board_arm2_status_get_keller_pa3_pressure(msg);
    viam_board_arm2_status->keller_pa3_temperature = mavlink_msg_viam_board_arm2_status_get_keller_pa3_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN? msg->len : MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN;
        memset(viam_board_arm2_status, 0, MAVLINK_MSG_ID_VIAM_BOARD_ARM2_STATUS_LEN);
    memcpy(viam_board_arm2_status, _MAV_PAYLOAD(msg), len);
#endif
}
