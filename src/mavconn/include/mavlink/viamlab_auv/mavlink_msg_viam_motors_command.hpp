// MESSAGE VIAM_MOTORS_COMMAND support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_MOTORS_COMMAND message
 *
 * Motors command message.
 */
struct VIAM_MOTORS_COMMAND : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13002;
    static constexpr size_t LENGTH = 20;
    static constexpr size_t MIN_LENGTH = 20;
    static constexpr uint8_t CRC_EXTRA = 210;
    static constexpr auto NAME = "VIAM_MOTORS_COMMAND";


    uint32_t time_boot_ms; /*< [ms]  */
    float thruster_speed; /*< [rpm]  */
    float rudder_angle; /*< [deg]  */
    float mass_shifter_position; /*< [mm]  */
    float piston_position; /*< [mm]  */


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
        ss << "  thruster_speed: " << thruster_speed << std::endl;
        ss << "  rudder_angle: " << rudder_angle << std::endl;
        ss << "  mass_shifter_position: " << mass_shifter_position << std::endl;
        ss << "  piston_position: " << piston_position << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << thruster_speed;                // offset: 4
        map << rudder_angle;                  // offset: 8
        map << mass_shifter_position;         // offset: 12
        map << piston_position;               // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> thruster_speed;                // offset: 4
        map >> rudder_angle;                  // offset: 8
        map >> mass_shifter_position;         // offset: 12
        map >> piston_position;               // offset: 16
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
