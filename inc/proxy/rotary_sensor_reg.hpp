
/**
 * @file rotary_sensor_reg.hpp
 *
 * @brief AS5047U rotary sensor registers definition
 *
 * @date 03/2024
 */

#ifndef __ROTARY_SENSOR_REG_HPP__
#define __ROTARY_SENSOR_REG_HPP__

#include <cstdint>

struct Registers {
    union Disable {
        struct __attribute__((packed)) {
            uint8_t UVW_off : 1;
            uint8_t ABI_off : 1;
            uint8_t na : 4;
            uint8_t FILTER_disable : 1;
        };

        uint8_t raw;
    };

    union Zposm {
        struct __attribute__((packed)) {
            uint8_t ZPOSM : 8;
        };

        uint8_t raw;
    };

    union Zpols {
        struct __attribute__((packed)) {
            uint8_t ZPOSL : 6;
            uint8_t Dia1_en : 1;
            uint8_t Dia2_en : 1;
        };

        uint8_t raw;
    };

    union Settings1 {
        struct __attribute__((packed)) {
            uint8_t K_max : 3;
            uint8_t K_min : 3;
            uint8_t Dia3_en : 1;
            uint8_t Dia4_en : 1;
        };

        uint8_t raw;
    };

    union Settings2 {
        struct __attribute__((packed)) {
            uint8_t IWIDTH : 1;
            uint8_t NOISESET : 1;
            uint8_t DIR : 1;
            uint8_t UVW_ABI : 1;
            uint8_t DAECDIS : 1;
            uint8_t ABI_DEC : 1;
            uint8_t Data_select : 1;
            uint8_t PWMon : 1;
        };

        uint8_t raw;
    };

    union Settings3 {
        struct __attribute__((packed)) {
            uint8_t UVWPP : 3;
            uint8_t HYS : 2;
            uint8_t ABIRES : 3;
        };

        uint8_t raw;
    };

    union Ecc {
        struct __attribute__((packed)) {
            uint8_t ECC_chsum : 7;
            uint8_t ECC_en : 1;
        };

        uint8_t raw;
    };

    static constexpr uint16_t disable_addr{0x0015};
    static constexpr uint16_t zposm_addr{0x0016};
    static constexpr uint16_t zpos_addr{0x0017};
    static constexpr uint16_t settings1_addr{0x0018};
    static constexpr uint16_t settings2_addr{0x0019};
    static constexpr uint16_t settings3_addr{0x001A};
    static constexpr uint16_t ecc_addr{0x001B};

    Disable                   disable;
    Zposm                     zposm;
    Zpols                     zpos;
    Settings1                 settings1;
    Settings2                 settings2;
    Settings3                 settings3;
    Ecc                       ecc;
};

#endif // __ROTARY_SENSOR_REG_HPP__