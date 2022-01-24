#pragma once
// MESSAGE VIAM_BOARD_ARM1_STATUS PACKING

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS 13007

MAVPACKED(
typedef struct __mavlink_viam_board_arm1_status_t {
 uint32_t time_boot_ms; /*< [ms] */
 float altimeter_in_metres; /*< [m] */
 float altimeter_in_feet; /*< [ft] */
 float altimeter_in_fathoms; /*< [fathom] */
 uint8_t ls_piston_athead; /*<  */
 uint8_t ls_piston_attail; /*<  */
 uint8_t ls_mass_shifter_athead; /*<  */
 uint8_t ls_mass_shifter_attail; /*<  */
}) mavlink_viam_board_arm1_status_t;

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN 20
#define MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN 20
#define MAVLINK_MSG_ID_13007_LEN 20
#define MAVLINK_MSG_ID_13007_MIN_LEN 20

#define MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC 50
#define MAVLINK_MSG_ID_13007_CRC 50



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_BOARD_ARM1_STATUS { \
    13007, \
    "VIAM_BOARD_ARM1_STATUS", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_board_arm1_status_t, time_boot_ms) }, \
         { "ls_piston_athead", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_viam_board_arm1_status_t, ls_piston_athead) }, \
         { "ls_piston_attail", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_viam_board_arm1_status_t, ls_piston_attail) }, \
         { "ls_mass_shifter_athead", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_viam_board_arm1_status_t, ls_mass_shifter_athead) }, \
         { "ls_mass_shifter_attail", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_viam_board_arm1_status_t, ls_mass_shifter_attail) }, \
         { "altimeter_in_metres", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_metres) }, \
         { "altimeter_in_feet", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_feet) }, \
         { "altimeter_in_fathoms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_fathoms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_BOARD_ARM1_STATUS { \
    "VIAM_BOARD_ARM1_STATUS", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_board_arm1_status_t, time_boot_ms) }, \
         { "ls_piston_athead", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_viam_board_arm1_status_t, ls_piston_athead) }, \
         { "ls_piston_attail", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_viam_board_arm1_status_t, ls_piston_attail) }, \
         { "ls_mass_shifter_athead", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_viam_board_arm1_status_t, ls_mass_shifter_athead) }, \
         { "ls_mass_shifter_attail", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_viam_board_arm1_status_t, ls_mass_shifter_attail) }, \
         { "altimeter_in_metres", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_metres) }, \
         { "altimeter_in_feet", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_feet) }, \
         { "altimeter_in_fathoms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_board_arm1_status_t, altimeter_in_fathoms) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_board_arm1_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param ls_piston_athead  
 * @param ls_piston_attail  
 * @param ls_mass_shifter_athead  
 * @param ls_mass_shifter_attail  
 * @param altimeter_in_metres [m] 
 * @param altimeter_in_feet [ft] 
 * @param altimeter_in_fathoms [fathom] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_board_arm1_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t ls_piston_athead, uint8_t ls_piston_attail, uint8_t ls_mass_shifter_athead, uint8_t ls_mass_shifter_attail, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_uint8_t(buf, 16, ls_piston_athead);
    _mav_put_uint8_t(buf, 17, ls_piston_attail);
    _mav_put_uint8_t(buf, 18, ls_mass_shifter_athead);
    _mav_put_uint8_t(buf, 19, ls_mass_shifter_attail);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN);
#else
    mavlink_viam_board_arm1_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.ls_piston_athead = ls_piston_athead;
    packet.ls_piston_attail = ls_piston_attail;
    packet.ls_mass_shifter_athead = ls_mass_shifter_athead;
    packet.ls_mass_shifter_attail = ls_mass_shifter_attail;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
}

/**
 * @brief Pack a viam_board_arm1_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param ls_piston_athead  
 * @param ls_piston_attail  
 * @param ls_mass_shifter_athead  
 * @param ls_mass_shifter_attail  
 * @param altimeter_in_metres [m] 
 * @param altimeter_in_feet [ft] 
 * @param altimeter_in_fathoms [fathom] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_board_arm1_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t ls_piston_athead,uint8_t ls_piston_attail,uint8_t ls_mass_shifter_athead,uint8_t ls_mass_shifter_attail,float altimeter_in_metres,float altimeter_in_feet,float altimeter_in_fathoms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_uint8_t(buf, 16, ls_piston_athead);
    _mav_put_uint8_t(buf, 17, ls_piston_attail);
    _mav_put_uint8_t(buf, 18, ls_mass_shifter_athead);
    _mav_put_uint8_t(buf, 19, ls_mass_shifter_attail);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN);
#else
    mavlink_viam_board_arm1_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.ls_piston_athead = ls_piston_athead;
    packet.ls_piston_attail = ls_piston_attail;
    packet.ls_mass_shifter_athead = ls_mass_shifter_athead;
    packet.ls_mass_shifter_attail = ls_mass_shifter_attail;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
}

/**
 * @brief Encode a viam_board_arm1_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_board_arm1_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_board_arm1_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_board_arm1_status_t* viam_board_arm1_status)
{
    return mavlink_msg_viam_board_arm1_status_pack(system_id, component_id, msg, viam_board_arm1_status->time_boot_ms, viam_board_arm1_status->ls_piston_athead, viam_board_arm1_status->ls_piston_attail, viam_board_arm1_status->ls_mass_shifter_athead, viam_board_arm1_status->ls_mass_shifter_attail, viam_board_arm1_status->altimeter_in_metres, viam_board_arm1_status->altimeter_in_feet, viam_board_arm1_status->altimeter_in_fathoms);
}

/**
 * @brief Encode a viam_board_arm1_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_board_arm1_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_board_arm1_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_board_arm1_status_t* viam_board_arm1_status)
{
    return mavlink_msg_viam_board_arm1_status_pack_chan(system_id, component_id, chan, msg, viam_board_arm1_status->time_boot_ms, viam_board_arm1_status->ls_piston_athead, viam_board_arm1_status->ls_piston_attail, viam_board_arm1_status->ls_mass_shifter_athead, viam_board_arm1_status->ls_mass_shifter_attail, viam_board_arm1_status->altimeter_in_metres, viam_board_arm1_status->altimeter_in_feet, viam_board_arm1_status->altimeter_in_fathoms);
}

/**
 * @brief Send a viam_board_arm1_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param ls_piston_athead  
 * @param ls_piston_attail  
 * @param ls_mass_shifter_athead  
 * @param ls_mass_shifter_attail  
 * @param altimeter_in_metres [m] 
 * @param altimeter_in_feet [ft] 
 * @param altimeter_in_fathoms [fathom] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_board_arm1_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t ls_piston_athead, uint8_t ls_piston_attail, uint8_t ls_mass_shifter_athead, uint8_t ls_mass_shifter_attail, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_uint8_t(buf, 16, ls_piston_athead);
    _mav_put_uint8_t(buf, 17, ls_piston_attail);
    _mav_put_uint8_t(buf, 18, ls_mass_shifter_athead);
    _mav_put_uint8_t(buf, 19, ls_mass_shifter_attail);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS, buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
#else
    mavlink_viam_board_arm1_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.ls_piston_athead = ls_piston_athead;
    packet.ls_piston_attail = ls_piston_attail;
    packet.ls_mass_shifter_athead = ls_mass_shifter_athead;
    packet.ls_mass_shifter_attail = ls_mass_shifter_attail;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS, (const char *)&packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
#endif
}

/**
 * @brief Send a viam_board_arm1_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_board_arm1_status_send_struct(mavlink_channel_t chan, const mavlink_viam_board_arm1_status_t* viam_board_arm1_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_board_arm1_status_send(chan, viam_board_arm1_status->time_boot_ms, viam_board_arm1_status->ls_piston_athead, viam_board_arm1_status->ls_piston_attail, viam_board_arm1_status->ls_mass_shifter_athead, viam_board_arm1_status->ls_mass_shifter_attail, viam_board_arm1_status->altimeter_in_metres, viam_board_arm1_status->altimeter_in_feet, viam_board_arm1_status->altimeter_in_fathoms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS, (const char *)viam_board_arm1_status, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_board_arm1_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t ls_piston_athead, uint8_t ls_piston_attail, uint8_t ls_mass_shifter_athead, uint8_t ls_mass_shifter_attail, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_uint8_t(buf, 16, ls_piston_athead);
    _mav_put_uint8_t(buf, 17, ls_piston_attail);
    _mav_put_uint8_t(buf, 18, ls_mass_shifter_athead);
    _mav_put_uint8_t(buf, 19, ls_mass_shifter_attail);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS, buf, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
#else
    mavlink_viam_board_arm1_status_t *packet = (mavlink_viam_board_arm1_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->altimeter_in_metres = altimeter_in_metres;
    packet->altimeter_in_feet = altimeter_in_feet;
    packet->altimeter_in_fathoms = altimeter_in_fathoms;
    packet->ls_piston_athead = ls_piston_athead;
    packet->ls_piston_attail = ls_piston_attail;
    packet->ls_mass_shifter_athead = ls_mass_shifter_athead;
    packet->ls_mass_shifter_attail = ls_mass_shifter_attail;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS, (const char *)packet, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_MIN_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_BOARD_ARM1_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from viam_board_arm1_status message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_board_arm1_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ls_piston_athead from viam_board_arm1_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_viam_board_arm1_status_get_ls_piston_athead(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field ls_piston_attail from viam_board_arm1_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_viam_board_arm1_status_get_ls_piston_attail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field ls_mass_shifter_athead from viam_board_arm1_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_viam_board_arm1_status_get_ls_mass_shifter_athead(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field ls_mass_shifter_attail from viam_board_arm1_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_viam_board_arm1_status_get_ls_mass_shifter_attail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field altimeter_in_metres from viam_board_arm1_status message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_board_arm1_status_get_altimeter_in_metres(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altimeter_in_feet from viam_board_arm1_status message
 *
 * @return [ft] 
 */
static inline float mavlink_msg_viam_board_arm1_status_get_altimeter_in_feet(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altimeter_in_fathoms from viam_board_arm1_status message
 *
 * @return [fathom] 
 */
static inline float mavlink_msg_viam_board_arm1_status_get_altimeter_in_fathoms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a viam_board_arm1_status message into a struct
 *
 * @param msg The message to decode
 * @param viam_board_arm1_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_board_arm1_status_decode(const mavlink_message_t* msg, mavlink_viam_board_arm1_status_t* viam_board_arm1_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_board_arm1_status->time_boot_ms = mavlink_msg_viam_board_arm1_status_get_time_boot_ms(msg);
    viam_board_arm1_status->altimeter_in_metres = mavlink_msg_viam_board_arm1_status_get_altimeter_in_metres(msg);
    viam_board_arm1_status->altimeter_in_feet = mavlink_msg_viam_board_arm1_status_get_altimeter_in_feet(msg);
    viam_board_arm1_status->altimeter_in_fathoms = mavlink_msg_viam_board_arm1_status_get_altimeter_in_fathoms(msg);
    viam_board_arm1_status->ls_piston_athead = mavlink_msg_viam_board_arm1_status_get_ls_piston_athead(msg);
    viam_board_arm1_status->ls_piston_attail = mavlink_msg_viam_board_arm1_status_get_ls_piston_attail(msg);
    viam_board_arm1_status->ls_mass_shifter_athead = mavlink_msg_viam_board_arm1_status_get_ls_mass_shifter_athead(msg);
    viam_board_arm1_status->ls_mass_shifter_attail = mavlink_msg_viam_board_arm1_status_get_ls_mass_shifter_attail(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN? msg->len : MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN;
        memset(viam_board_arm1_status, 0, MAVLINK_MSG_ID_VIAM_BOARD_ARM1_STATUS_LEN);
    memcpy(viam_board_arm1_status, _MAV_PAYLOAD(msg), len);
#endif
}
