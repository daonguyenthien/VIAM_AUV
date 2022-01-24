// MESSAGE VIAM_BOARD_ARM1_STATUS support class

#pragma once

namespace mavlink {
namespace viamlab_auv {
namespace msg {

/**
 * @brief VIAM_BOARD_ARM1_STATUS message
 *
 * Board ARM1 status message.
 */
struct VIAM_BOARD_ARM1_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 13007;
    static constexpr size_t LENGTH = 20;
    static constexpr size_t MIN_LENGTH = 20;
    static constexpr uint8_t CRC_EXTRA = 50;
    static constexpr auto NAME = "VIAM_BOARD_ARM1_STATUS";


    uint32_t time_boot_ms; /*< [ms]  */
    uint8_t ls_piston_athead; /*<   */
    uint8_t ls_piston_attail; /*<   */
    uint8_t ls_mass_shifter_athead; /*<   */
    uint8_t ls_mass_shifter_attail; /*<   */
    float altimeter_in_metres; /*< [m]  */
    float altimeter_in_feet; /*< [ft]  */
    float altimeter_in_fathoms; /*< [fathom]  */


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
        ss << "  ls_piston_athead: " << +ls_piston_athead << std::endl;
        ss << "  ls_piston_attail: " << +ls_piston_attail << std::endl;
        ss << "  ls_mass_shifter_athead: " << +ls_mass_shifter_athead << std::endl;
        ss << "  ls_mass_shifter_attail: " << +ls_mass_shifter_attail << std::endl;
        ss << "  altimeter_in_metres: " << altimeter_in_metres << std::endl;
        ss << "  altimeter_in_feet: " << altimeter_in_feet << std::endl;
        ss << "  altimeter_in_fathoms: " << altimeter_in_fathoms << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << altimeter_in_metres;           // offset: 4
        map << altimeter_in_feet;             // offset: 8
        map << altimeter_in_fathoms;          // offset: 12
        map << ls_piston_athead;              // offset: 16
        map << ls_piston_attail;              // offset: 17
        map << ls_mass_shifter_athead;        // offset: 18
        map << ls_mass_shifter_attail;        // offset: 19
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> altimeter_in_metres;           // offset: 4
        map >> altimeter_in_feet;             // offset: 8
        map >> altimeter_in_fathoms;          // offset: 12
        map >> ls_piston_athead;              // offset: 16
        map >> ls_piston_attail;              // offset: 17
        map >> ls_mass_shifter_athead;        // offset: 18
        map >> ls_mass_shifter_attail;        // offset: 19
    }
};

} // namespace msg
} // namespace viamlab_auv
} // namespace mavlink
