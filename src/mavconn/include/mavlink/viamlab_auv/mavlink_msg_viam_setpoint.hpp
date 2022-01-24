// MESSAGE VIAM_SETPOINT support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_SETPOINT message
 *
 * Setpoint message.
 */
struct VIAM_SETPOINT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13001;
    static constexpr size_t LENGTH = 52;
    static constexpr size_t MIN_LENGTH = 52;
    static constexpr uint8_t CRC_EXTRA = 240;
    static constexpr auto NAME = "VIAM_SETPOINT";


    uint32_t time_boot_ms; /*< [ms]  */
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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << x;                             // offset: 4
        map << y;                             // offset: 8
        map << z;                             // offset: 12
        map << roll;                          // offset: 16
        map << pitch;                         // offset: 20
        map << yaw;                           // offset: 24
        map << vx;                            // offset: 28
        map << vy;                            // offset: 32
        map << vz;                            // offset: 36
        map << vroll;                         // offset: 40
        map << vpitch;                        // offset: 44
        map << vyaw;                          // offset: 48
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> x;                             // offset: 4
        map >> y;                             // offset: 8
        map >> z;                             // offset: 12
        map >> roll;                          // offset: 16
        map >> pitch;                         // offset: 20
        map >> yaw;                           // offset: 24
        map >> vx;                            // offset: 28
        map >> vy;                            // offset: 32
        map >> vz;                            // offset: 36
        map >> vroll;                         // offset: 40
        map >> vpitch;                        // offset: 44
        map >> vyaw;                          // offset: 48
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
