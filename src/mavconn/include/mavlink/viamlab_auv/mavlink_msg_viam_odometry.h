#pragma once
// MESSAGE VIAM_ODOMETRY PACKING

#define MAVLINK_MSG_ID_VIAM_ODOMETRY 13000

MAVPACKED(
typedef struct __mavlink_viam_odometry_t {
 uint32_t time_boot_ms; /*< [ms] */
 int32_t lat; /*< [degE7] */
 int32_t lon; /*< [degE7] */
 float alt; /*< [m] */
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
 float ax; /*< [m/s^2] */
 float ay; /*< [m/s^2] */
 float az; /*< [m/s^2] */
 float aroll; /*< [rad/s^2] */
 float apitch; /*< [rad/s^2] */
 float ayaw; /*< [rad/s^2] */
}) mavlink_viam_odometry_t;

#define MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN 88
#define MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN 88
#define MAVLINK_MSG_ID_13000_LEN 88
#define MAVLINK_MSG_ID_13000_MIN_LEN 88

#define MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC 72
#define MAVLINK_MSG_ID_13000_CRC 72



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VIAM_ODOMETRY { \
    13000, \
    "VIAM_ODOMETRY", \
    22, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_odometry_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_viam_odometry_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_viam_odometry_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_odometry_t, alt) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_odometry_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_odometry_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_odometry_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_odometry_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_viam_odometry_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_viam_odometry_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_viam_odometry_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_viam_odometry_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_viam_odometry_t, vz) }, \
         { "vroll", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_viam_odometry_t, vroll) }, \
         { "vpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_viam_odometry_t, vpitch) }, \
         { "vyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_viam_odometry_t, vyaw) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_viam_odometry_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_viam_odometry_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_viam_odometry_t, az) }, \
         { "aroll", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_viam_odometry_t, aroll) }, \
         { "apitch", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_viam_odometry_t, apitch) }, \
         { "ayaw", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_viam_odometry_t, ayaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VIAM_ODOMETRY { \
    "VIAM_ODOMETRY", \
    22, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viam_odometry_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_viam_odometry_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_viam_odometry_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_viam_odometry_t, alt) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_viam_odometry_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_viam_odometry_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_viam_odometry_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_viam_odometry_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_viam_odometry_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_viam_odometry_t, yaw) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_viam_odometry_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_viam_odometry_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_viam_odometry_t, vz) }, \
         { "vroll", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_viam_odometry_t, vroll) }, \
         { "vpitch", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_viam_odometry_t, vpitch) }, \
         { "vyaw", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_viam_odometry_t, vyaw) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_viam_odometry_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_viam_odometry_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_viam_odometry_t, az) }, \
         { "aroll", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_viam_odometry_t, aroll) }, \
         { "apitch", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_viam_odometry_t, apitch) }, \
         { "ayaw", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_viam_odometry_t, ayaw) }, \
         } \
}
#endif

/**
 * @brief Pack a viam_odometry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param lat [degE7] 
 * @param lon [degE7] 
 * @param alt [m] 
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
 * @param ax [m/s^2] 
 * @param ay [m/s^2] 
 * @param az [m/s^2] 
 * @param aroll [rad/s^2] 
 * @param apitch [rad/s^2] 
 * @param ayaw [rad/s^2] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_odometry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, float alt, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw, float ax, float ay, float az, float aroll, float apitch, float ayaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, roll);
    _mav_put_float(buf, 32, pitch);
    _mav_put_float(buf, 36, yaw);
    _mav_put_float(buf, 40, vx);
    _mav_put_float(buf, 44, vy);
    _mav_put_float(buf, 48, vz);
    _mav_put_float(buf, 52, vroll);
    _mav_put_float(buf, 56, vpitch);
    _mav_put_float(buf, 60, vyaw);
    _mav_put_float(buf, 64, ax);
    _mav_put_float(buf, 68, ay);
    _mav_put_float(buf, 72, az);
    _mav_put_float(buf, 76, aroll);
    _mav_put_float(buf, 80, apitch);
    _mav_put_float(buf, 84, ayaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN);
#else
    mavlink_viam_odometry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
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
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.aroll = aroll;
    packet.apitch = apitch;
    packet.ayaw = ayaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_ODOMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
}

/**
 * @brief Pack a viam_odometry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param lat [degE7] 
 * @param lon [degE7] 
 * @param alt [m] 
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
 * @param ax [m/s^2] 
 * @param ay [m/s^2] 
 * @param az [m/s^2] 
 * @param aroll [rad/s^2] 
 * @param apitch [rad/s^2] 
 * @param ayaw [rad/s^2] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viam_odometry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat,int32_t lon,float alt,float x,float y,float z,float roll,float pitch,float yaw,float vx,float vy,float vz,float vroll,float vpitch,float vyaw,float ax,float ay,float az,float aroll,float apitch,float ayaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, roll);
    _mav_put_float(buf, 32, pitch);
    _mav_put_float(buf, 36, yaw);
    _mav_put_float(buf, 40, vx);
    _mav_put_float(buf, 44, vy);
    _mav_put_float(buf, 48, vz);
    _mav_put_float(buf, 52, vroll);
    _mav_put_float(buf, 56, vpitch);
    _mav_put_float(buf, 60, vyaw);
    _mav_put_float(buf, 64, ax);
    _mav_put_float(buf, 68, ay);
    _mav_put_float(buf, 72, az);
    _mav_put_float(buf, 76, aroll);
    _mav_put_float(buf, 80, apitch);
    _mav_put_float(buf, 84, ayaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN);
#else
    mavlink_viam_odometry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
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
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.aroll = aroll;
    packet.apitch = apitch;
    packet.ayaw = ayaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VIAM_ODOMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
}

/**
 * @brief Encode a viam_odometry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viam_odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_odometry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viam_odometry_t* viam_odometry)
{
    return mavlink_msg_viam_odometry_pack(system_id, component_id, msg, viam_odometry->time_boot_ms, viam_odometry->lat, viam_odometry->lon, viam_odometry->alt, viam_odometry->x, viam_odometry->y, viam_odometry->z, viam_odometry->roll, viam_odometry->pitch, viam_odometry->yaw, viam_odometry->vx, viam_odometry->vy, viam_odometry->vz, viam_odometry->vroll, viam_odometry->vpitch, viam_odometry->vyaw, viam_odometry->ax, viam_odometry->ay, viam_odometry->az, viam_odometry->aroll, viam_odometry->apitch, viam_odometry->ayaw);
}

/**
 * @brief Encode a viam_odometry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viam_odometry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viam_odometry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viam_odometry_t* viam_odometry)
{
    return mavlink_msg_viam_odometry_pack_chan(system_id, component_id, chan, msg, viam_odometry->time_boot_ms, viam_odometry->lat, viam_odometry->lon, viam_odometry->alt, viam_odometry->x, viam_odometry->y, viam_odometry->z, viam_odometry->roll, viam_odometry->pitch, viam_odometry->yaw, viam_odometry->vx, viam_odometry->vy, viam_odometry->vz, viam_odometry->vroll, viam_odometry->vpitch, viam_odometry->vyaw, viam_odometry->ax, viam_odometry->ay, viam_odometry->az, viam_odometry->aroll, viam_odometry->apitch, viam_odometry->ayaw);
}

/**
 * @brief Send a viam_odometry message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param lat [degE7] 
 * @param lon [degE7] 
 * @param alt [m] 
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
 * @param ax [m/s^2] 
 * @param ay [m/s^2] 
 * @param az [m/s^2] 
 * @param aroll [rad/s^2] 
 * @param apitch [rad/s^2] 
 * @param ayaw [rad/s^2] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viam_odometry_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, float alt, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw, float ax, float ay, float az, float aroll, float apitch, float ayaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, roll);
    _mav_put_float(buf, 32, pitch);
    _mav_put_float(buf, 36, yaw);
    _mav_put_float(buf, 40, vx);
    _mav_put_float(buf, 44, vy);
    _mav_put_float(buf, 48, vz);
    _mav_put_float(buf, 52, vroll);
    _mav_put_float(buf, 56, vpitch);
    _mav_put_float(buf, 60, vyaw);
    _mav_put_float(buf, 64, ax);
    _mav_put_float(buf, 68, ay);
    _mav_put_float(buf, 72, az);
    _mav_put_float(buf, 76, aroll);
    _mav_put_float(buf, 80, apitch);
    _mav_put_float(buf, 84, ayaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ODOMETRY, buf, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
#else
    mavlink_viam_odometry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
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
    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;
    packet.aroll = aroll;
    packet.apitch = apitch;
    packet.ayaw = ayaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ODOMETRY, (const char *)&packet, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
#endif
}

/**
 * @brief Send a viam_odometry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viam_odometry_send_struct(mavlink_channel_t chan, const mavlink_viam_odometry_t* viam_odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viam_odometry_send(chan, viam_odometry->time_boot_ms, viam_odometry->lat, viam_odometry->lon, viam_odometry->alt, viam_odometry->x, viam_odometry->y, viam_odometry->z, viam_odometry->roll, viam_odometry->pitch, viam_odometry->yaw, viam_odometry->vx, viam_odometry->vy, viam_odometry->vz, viam_odometry->vroll, viam_odometry->vpitch, viam_odometry->vyaw, viam_odometry->ax, viam_odometry->ay, viam_odometry->az, viam_odometry->aroll, viam_odometry->apitch, viam_odometry->ayaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ODOMETRY, (const char *)viam_odometry, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viam_odometry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, float alt, float x, float y, float z, float roll, float pitch, float yaw, float vx, float vy, float vz, float vroll, float vpitch, float vyaw, float ax, float ay, float az, float aroll, float apitch, float ayaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_float(buf, 12, alt);
    _mav_put_float(buf, 16, x);
    _mav_put_float(buf, 20, y);
    _mav_put_float(buf, 24, z);
    _mav_put_float(buf, 28, roll);
    _mav_put_float(buf, 32, pitch);
    _mav_put_float(buf, 36, yaw);
    _mav_put_float(buf, 40, vx);
    _mav_put_float(buf, 44, vy);
    _mav_put_float(buf, 48, vz);
    _mav_put_float(buf, 52, vroll);
    _mav_put_float(buf, 56, vpitch);
    _mav_put_float(buf, 60, vyaw);
    _mav_put_float(buf, 64, ax);
    _mav_put_float(buf, 68, ay);
    _mav_put_float(buf, 72, az);
    _mav_put_float(buf, 76, aroll);
    _mav_put_float(buf, 80, apitch);
    _mav_put_float(buf, 84, ayaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ODOMETRY, buf, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
#else
    mavlink_viam_odometry_t *packet = (mavlink_viam_odometry_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
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
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;
    packet->aroll = aroll;
    packet->apitch = apitch;
    packet->ayaw = ayaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VIAM_ODOMETRY, (const char *)packet, MAVLINK_MSG_ID_VIAM_ODOMETRY_MIN_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN, MAVLINK_MSG_ID_VIAM_ODOMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE VIAM_ODOMETRY UNPACKING


/**
 * @brief Get field time_boot_ms from viam_odometry message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_viam_odometry_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from viam_odometry message
 *
 * @return [degE7] 
 */
static inline int32_t mavlink_msg_viam_odometry_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from viam_odometry message
 *
 * @return [degE7] 
 */
static inline int32_t mavlink_msg_viam_odometry_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from viam_odometry message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_odometry_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from viam_odometry message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_odometry_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from viam_odometry message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_odometry_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from viam_odometry message
 *
 * @return [m] 
 */
static inline float mavlink_msg_viam_odometry_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field roll from viam_odometry message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_odometry_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pitch from viam_odometry message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_odometry_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field yaw from viam_odometry message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_viam_odometry_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vx from viam_odometry message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vy from viam_odometry message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vz from viam_odometry message
 *
 * @return [m/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vroll from viam_odometry message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vroll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vpitch from viam_odometry message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vpitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vyaw from viam_odometry message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_viam_odometry_get_vyaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field ax from viam_odometry message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_ax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field ay from viam_odometry message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_ay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field az from viam_odometry message
 *
 * @return [m/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_az(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field aroll from viam_odometry message
 *
 * @return [rad/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_aroll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field apitch from viam_odometry message
 *
 * @return [rad/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_apitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field ayaw from viam_odometry message
 *
 * @return [rad/s^2] 
 */
static inline float mavlink_msg_viam_odometry_get_ayaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Decode a viam_odometry message into a struct
 *
 * @param msg The message to decode
 * @param viam_odometry C-struct to decode the message contents into
 */
static inline void mavlink_msg_viam_odometry_decode(const mavlink_message_t* msg, mavlink_viam_odometry_t* viam_odometry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viam_odometry->time_boot_ms = mavlink_msg_viam_odometry_get_time_boot_ms(msg);
    viam_odometry->lat = mavlink_msg_viam_odometry_get_lat(msg);
    viam_odometry->lon = mavlink_msg_viam_odometry_get_lon(msg);
    viam_odometry->alt = mavlink_msg_viam_odometry_get_alt(msg);
    viam_odometry->x = mavlink_msg_viam_odometry_get_x(msg);
    viam_odometry->y = mavlink_msg_viam_odometry_get_y(msg);
    viam_odometry->z = mavlink_msg_viam_odometry_get_z(msg);
    viam_odometry->roll = mavlink_msg_viam_odometry_get_roll(msg);
    viam_odometry->pitch = mavlink_msg_viam_odometry_get_pitch(msg);
    viam_odometry->yaw = mavlink_msg_viam_odometry_get_yaw(msg);
    viam_odometry->vx = mavlink_msg_viam_odometry_get_vx(msg);
    viam_odometry->vy = mavlink_msg_viam_odometry_get_vy(msg);
    viam_odometry->vz = mavlink_msg_viam_odometry_get_vz(msg);
    viam_odometry->vroll = mavlink_msg_viam_odometry_get_vroll(msg);
    viam_odometry->vpitch = mavlink_msg_viam_odometry_get_vpitch(msg);
    viam_odometry->vyaw = mavlink_msg_viam_odometry_get_vyaw(msg);
    viam_odometry->ax = mavlink_msg_viam_odometry_get_ax(msg);
    viam_odometry->ay = mavlink_msg_viam_odometry_get_ay(msg);
    viam_odometry->az = mavlink_msg_viam_odometry_get_az(msg);
    viam_odometry->aroll = mavlink_msg_viam_odometry_get_aroll(msg);
    viam_odometry->apitch = mavlink_msg_viam_odometry_get_apitch(msg);
    viam_odometry->ayaw = mavlink_msg_viam_odometry_get_ayaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN? msg->len : MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN;
        memset(viam_odometry, 0, MAVLINK_MSG_ID_VIAM_ODOMETRY_LEN);
    memcpy(viam_odometry, _MAV_PAYLOAD(msg), len);
#endif
}
