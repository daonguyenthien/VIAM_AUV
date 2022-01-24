// MESSAGE VIAM_ERROR support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_ERROR message
 *
 * Error message.
 */
struct VIAM_ERROR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13003;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 248;
    static constexpr auto NAME = "VIAM_ERROR";


    uint32_t time_boot_ms; /*< [ms]  */
    float along_track; /*< [m]  */
    float cross_track; /*< [m]  */


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
        ss << "  along_track: " << along_track << std::endl;
        ss << "  cross_track: " << cross_track << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << along_track;                   // offset: 4
        map << cross_track;                   // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> along_track;                   // offset: 4
        map >> cross_track;                   // offset: 8
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
