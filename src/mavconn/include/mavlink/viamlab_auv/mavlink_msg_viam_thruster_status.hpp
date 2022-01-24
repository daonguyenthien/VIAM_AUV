// MESSAGE VIAM_THRUSTER_STATUS support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_THRUSTER_STATUS message
 *
 * Thruster status message.
 */
struct VIAM_THRUSTER_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13004;
    static constexpr size_t LENGTH = 28;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 223;
    static constexpr auto NAME = "VIAM_THRUSTER_STATUS";


    uint32_t time_boot_ms; /*< [ms]  */
    float motor_duty; /*< [%]  */
    float motor_temp_on_chip; /*< [oC]  */
    float motor_temp_ambient; /*< [oC]  */
    float motor_current; /*< [mA]  */
    float motor_rspeed; /*< [rpm]  */
    float motor_dspeed; /*< [rpm]  */


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
        ss << "  motor_duty: " << motor_duty << std::endl;
        ss << "  motor_temp_on_chip: " << motor_temp_on_chip << std::endl;
        ss << "  motor_temp_ambient: " << motor_temp_ambient << std::endl;
        ss << "  motor_current: " << motor_current << std::endl;
        ss << "  motor_rspeed: " << motor_rspeed << std::endl;
        ss << "  motor_dspeed: " << motor_dspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << motor_duty;                    // offset: 4
        map << motor_temp_on_chip;            // offset: 8
        map << motor_temp_ambient;            // offset: 12
        map << motor_current;                 // offset: 16
        map << motor_rspeed;                  // offset: 20
        map << motor_dspeed;                  // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> motor_duty;                    // offset: 4
        map >> motor_temp_on_chip;            // offset: 8
        map >> motor_temp_ambient;            // offset: 12
        map >> motor_current;                 // offset: 16
        map >> motor_rspeed;                  // offset: 20
        map >> motor_dspeed;                  // offset: 24
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
