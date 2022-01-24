// MESSAGE VIAM_BOARD_ARM2_STATUS support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_BOARD_ARM2_STATUS message
 *
 * Board ARM2 status message.
 */
struct VIAM_BOARD_ARM2_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13008;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 32;
    static constexpr uint8_t CRC_EXTRA = 207;
    static constexpr auto NAME = "VIAM_BOARD_ARM2_STATUS";


    uint32_t time_boot_ms; /*< [ms]  */
    float rudder_position; /*< [deg]  */
    float rudder_speed; /*< [rpm]  */
    float rudder_load; /*< [Nm]  */
    float rudder_voltage; /*< [V]  */
    float rudder_temperature; /*< [oC]  */
    float keller_pa3_pressure; /*< [Pa]  */
    float keller_pa3_temperature; /*< [oC]  */


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
        ss << "  rudder_position: " << rudder_position << std::endl;
        ss << "  rudder_speed: " << rudder_speed << std::endl;
        ss << "  rudder_load: " << rudder_load << std::endl;
        ss << "  rudder_voltage: " << rudder_voltage << std::endl;
        ss << "  rudder_temperature: " << rudder_temperature << std::endl;
        ss << "  keller_pa3_pressure: " << keller_pa3_pressure << std::endl;
        ss << "  keller_pa3_temperature: " << keller_pa3_temperature << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << rudder_position;               // offset: 4
        map << rudder_speed;                  // offset: 8
        map << rudder_load;                   // offset: 12
        map << rudder_voltage;                // offset: 16
        map << rudder_temperature;            // offset: 20
        map << keller_pa3_pressure;           // offset: 24
        map << keller_pa3_temperature;        // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> rudder_position;               // offset: 4
        map >> rudder_speed;                  // offset: 8
        map >> rudder_load;                   // offset: 12
        map >> rudder_voltage;                // offset: 16
        map >> rudder_temperature;            // offset: 20
        map >> keller_pa3_pressure;           // offset: 24
        map >> keller_pa3_temperature;        // offset: 28
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
