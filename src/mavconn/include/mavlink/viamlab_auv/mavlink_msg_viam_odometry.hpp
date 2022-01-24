// MESSAGE VIAM_ODOMETRY support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_ODOMETRY message
 *
 * Odometry message.
 */
struct VIAM_ODOMETRY : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13000;
    static constexpr size_t LENGTH = 88;
    static constexpr size_t MIN_LENGTH = 88;
    static constexpr uint8_t CRC_EXTRA = 72;
    static constexpr auto NAME = "VIAM_ODOMETRY";


    uint32_t time_boot_ms; /*< [ms]  */
    int32_t lat; /*< [degE7]  */
    int32_t lon; /*< [degE7]  */
    float alt; /*< [m]  */
    float x; /*< [m]  */
    float y; /*< [m]  */
    float z; /*< [m]  */
    float roll; /*< [rad]  */
    float pitch; /*< [rad]  */
    float yaw; /*< [rad]  */
    float vx; /*< [m/s]  */
    float vy; /*< [m/s]  */
    float vz; /*< [m/s]  */
    float vroll; /*< [rad/s]  */
    float vpitch; /*< [rad/s]  */
    float vyaw; /*< [rad/s]  */
    float ax; /*< [m/s^2]  */
    float ay; /*< [m/s^2]  */
    float az; /*< [m/s^2]  */
    float aroll; /*< [rad/s^2]  */
    float apitch; /*< [rad/s^2]  */
    float ayaw; /*< [rad/s^2]  */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;
        ss << "  roll: " << roll << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  vx: " << vx << std::endl;
        ss << "  vy: " << vy << std::endl;
        ss << "  vz: " << vz << std::endl;
        ss << "  vroll: " << vroll << std::endl;
        ss << "  vpitch: " << vpitch << std::endl;
        ss << "  vyaw: " << vyaw << std::endl;
        ss << "  ax: " << ax << std::endl;
        ss << "  ay: " << ay << std::endl;
        ss << "  az: " << az << std::endl;
        ss << "  aroll: " << aroll << std::endl;
        ss << "  apitch: " << apitch << std::endl;
        ss << "  ayaw: " << ayaw << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << lat;                           // offset: 4
        map << lon;                           // offset: 8
        map << alt;                           // offset: 12
        map << x;                             // offset: 16
        map << y;                             // offset: 20
        map << z;                             // offset: 24
        map << roll;                          // offset: 28
        map << pitch;                         // offset: 32
        map << yaw;                           // offset: 36
        map << vx;                            // offset: 40
        map << vy;                            // offset: 44
        map << vz;                            // offset: 48
        map << vroll;                         // offset: 52
        map << vpitch;                        // offset: 56
        map << vyaw;                          // offset: 60
        map << ax;                            // offset: 64
        map << ay;                            // offset: 68
        map << az;                            // offset: 72
        map << aroll;                         // offset: 76
        map << apitch;                        // offset: 80
        map << ayaw;                          // offset: 84
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> lat;                           // offset: 4
        map >> lon;                           // offset: 8
        map >> alt;                           // offset: 12
        map >> x;                             // offset: 16
        map >> y;                             // offset: 20
        map >> z;                             // offset: 24
        map >> roll;                          // offset: 28
        map >> pitch;                         // offset: 32
        map >> yaw;                           // offset: 36
        map >> vx;                            // offset: 40
        map >> vy;                            // offset: 44
        map >> vz;                            // offset: 48
        map >> vroll;                         // offset: 52
        map >> vpitch;                        // offset: 56
        map >> vyaw;                          // offset: 60
        map >> ax;                            // offset: 64
        map >> ay;                            // offset: 68
        map >> az;                            // offset: 72
        map >> aroll;                         // offset: 76
        map >> apitch;                        // offset: 80
        map >> ayaw;                          // offset: 84
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
