#ifndef asteIMU_ImuTypes_HPP
#define asteIMU_ImuTypes_HPP
#include "Fw/FPrimeBasicTypes.hpp"

namespace asteIMU {
    static constexpr U8 DEVICE_DEFAULT_ADDRESS = 0x28;

    // BNO055 Registers
    static constexpr U8 OPR_MODE_REGISTER = 0x3D;
    static constexpr U8 PWR_MODE_REGISTER = 0x3E;
    static constexpr U8 SYS_TRIGGER_REGISTER = 0x3F;
    static constexpr U8 PAGE_ID_REGISTER = 0x07;

    // Operation modes
    static constexpr U8 CONFIG_MODE = 0x00;
    static constexpr U8 NDOF_MODE = 0x0C;  // Full fusion mode

    // Power modes
    static constexpr U8 NORMAL_POWER = 0x00;

    // Reset value
    static constexpr U8 RESET_VALUE = 0x20;
    static constexpr U8 POWER_ON_VALUE = 0x00;

    // Data registers - accelerometer starts at 0x28
    static constexpr U8 DATA_BASE_REGISTER = 0x28;
    static constexpr U8 DATA_LENGTH = 6 * sizeof(U16); // accel + gyro (no temp for now)

    // Accel/Gyro config (BNO055 handles scaling internally in NDOF mode)
    static constexpr U8 ACCEL_CONFIG_2G = 0x00;
    static constexpr U8 ACCEL_CONFIG_4G = 0x01;
    static constexpr U8 ACCEL_CONFIG_8G = 0x02;
    static constexpr U8 ACCEL_CONFIG_16G = 0x03;
    static constexpr U8 GYRO_CONFIG_250DEG = 0x04;
    static constexpr U8 GYRO_CONFIG_500DEG = 0x03;
    static constexpr U8 GYRO_CONFIG_1000DEG = 0x02;
    static constexpr U8 GYRO_CONFIG_2000DEG = 0x00;

    static constexpr F32 TEMPERATURE_SCALAR = 1.0f;
    static constexpr F32 TEMPERATURE_OFFSET = 0.0f;

    struct RawImuData {
        I16 acceleration[3];
        I16 temperature;
        I16 gyroscope[3];
    };
}
#endif