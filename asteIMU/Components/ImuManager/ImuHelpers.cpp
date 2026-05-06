// ======================================================================
// \title  ImuHelpers.cpp
// \author mstarch
// \brief  cpp file for ImuManager component helper function implementations
//         Rewritten for BNO055 IMU
// ======================================================================

#include "asteIMU/Components/ImuManager/ImuManager.hpp"
#include "asteIMU/Components/ImuManager/ImuTypes.hpp"

namespace asteIMU {

Drv::I2cStatus ImuManager ::reset() {
    // Write reset bit to SYS_TRIGGER register
    U8 reset_sequence[] = {SYS_TRIGGER_REGISTER, RESET_VALUE};
    Fw::Buffer writeBuffer(reset_sequence, sizeof(reset_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::read_reset(U8& value) {
    // Read OPR_MODE register - after reset it will return to 0x00 (CONFIG_MODE)
    U8 registerAddress = OPR_MODE_REGISTER;
    Fw::Buffer writeBuffer(&registerAddress, sizeof(registerAddress));
    Fw::Buffer readBuffer(&value, sizeof(value));
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::enable() {
    // First set page 0
    U8 page_sequence[] = {PAGE_ID_REGISTER, 0x00};
    Fw::Buffer pageBuffer(page_sequence, sizeof(page_sequence));
    Fw::Buffer emptyBuffer;
    Drv::I2cStatus status = this->bus_write(pageBuffer, emptyBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }
    // Set operation mode to NDOF (full sensor fusion)
    U8 mode_sequence[] = {OPR_MODE_REGISTER, NDOF_MODE};
    Fw::Buffer writeBuffer(mode_sequence, sizeof(mode_sequence));
    Fw::Buffer readBuffer;
    return this->bus_write(writeBuffer, readBuffer);
}

Drv::I2cStatus ImuManager ::configure_device() {
    // Set power mode to normal
    U8 pwr_sequence[] = {PWR_MODE_REGISTER, NORMAL_POWER};
    Fw::Buffer writeBuffer(pwr_sequence, sizeof(pwr_sequence));
    Fw::Buffer readBuffer;
    Drv::I2cStatus status = this->bus_write(writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }
    // Set page 0
    U8 page_sequence[] = {PAGE_ID_REGISTER, 0x00};
    Fw::Buffer pageBuffer(page_sequence, sizeof(page_sequence));
    Fw::Buffer emptyBuffer;
    return this->bus_write(pageBuffer, emptyBuffer);
}

Drv::I2cStatus ImuManager::read(ImuData& imuData) {
    // Read 6 bytes of fused Euler angles starting at 0x1A:
    //   bytes 0-1: heading (yaw),  little-endian, 1/16 deg per LSB, 0-360
    //   bytes 2-3: roll,           little-endian, 1/16 deg per LSB, -90 to +90
    //   bytes 4-5: pitch,          little-endian, 1/16 deg per LSB, -180 to +180
    U8 data[EULER_DATA_LENGTH];
    U8 registerAddress = EULER_BASE_REGISTER;
 
    Fw::Buffer writeBuffer(&registerAddress, 1);
    Fw::Buffer readBuffer(data, EULER_DATA_LENGTH);
    Drv::I2cStatus status = this->bus_write(writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }

    I16 raw_heading = static_cast<I16>((static_cast<U16>(data[1]) << 8) | data[0]);
    I16 raw_roll = static_cast<I16>((static_cast<U16>(data[3]) << 8) | data[2]);
    I16 raw_pitch = static_cast<I16>((static_cast<U16>(data[5]) << 8) | data[4]);
 
    // Convert to degrees using BNO055 scale factor (1/16 deg per LSB)
    F32 yaw   = static_cast<F32>(raw_heading) * EULER_SCALE;
    F32 roll  = static_cast<F32>(raw_roll) * EULER_SCALE;
    F32 pitch = static_cast<F32>(raw_pitch) * EULER_SCALE;
 
    imuData.get_acceleration().set_x(roll);
    imuData.get_acceleration().set_y(pitch);
    imuData.get_acceleration().set_z(yaw);
    imuData.get_rotation().set_x(0.0f);
    imuData.get_rotation().set_y(0.0f);
    imuData.get_rotation().set_z(0.0f);
    imuData.set_temperature(0.0f);
 
    return status;
}

RawImuData ImuManager::deserialize_raw_data(Fw::Buffer& buffer) {
    // Parse the 6-byte Euler angle buffer into RawImuData
    const U8* data = buffer.getData();
    RawImuData raw;
    raw.heading = static_cast<I16>((static_cast<U16>(data[1]) << 8) | data[0]);
    raw.roll    = static_cast<I16>((static_cast<U16>(data[3]) << 8) | data[2]);
    raw.pitch   = static_cast<I16>((static_cast<U16>(data[5]) << 8) | data[4]);
    return raw;
}

ImuData ImuManager::convert_raw_data(const RawImuData& raw,
                                     const AccelerationRange& accelerationRange,
                                     const GyroscopeRange& gyroscopeRange) {
    // accelerationRange and gyroscopeRange are unused in NDOF mode —
    // the BNO055 handles all scaling internally.
    ImuData imuData;
    imuData.get_acceleration().set_x(static_cast<F32>(raw.roll) * EULER_SCALE);
    imuData.get_acceleration().set_y(static_cast<F32>(raw.pitch) * EULER_SCALE);
    imuData.get_acceleration().set_z(static_cast<F32>(raw.heading) * EULER_SCALE);
    imuData.get_rotation().set_x(0.0f);
    imuData.get_rotation().set_y(0.0f);
    imuData.get_rotation().set_z(0.0f);
    imuData.set_temperature(0.0f);
    return imuData;
}

U8 ImuManager ::accelerometer_range_to_register(AccelerationRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case AccelerationRange::RANGE_2G:
            registerValue = ACCEL_CONFIG_2G;
            break;
        case AccelerationRange::RANGE_4G:
            registerValue = ACCEL_CONFIG_4G;
            break;
        case AccelerationRange::RANGE_8G:
            registerValue = ACCEL_CONFIG_8G;
            break;
        case AccelerationRange::RANGE_16G:
            registerValue = ACCEL_CONFIG_16G;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

U8 ImuManager ::gyroscope_range_to_register(GyroscopeRange range) {
    U8 registerValue = 0;
    switch (range.e) {
        case GyroscopeRange::RANGE_250DEG:
            registerValue = GYRO_CONFIG_250DEG;
            break;
        case GyroscopeRange::RANGE_500DEG:
            registerValue = GYRO_CONFIG_500DEG;
            break;
        case GyroscopeRange::RANGE_1000DEG:
            registerValue = GYRO_CONFIG_1000DEG;
            break;
        case GyroscopeRange::RANGE_2000DEG:
            registerValue = GYRO_CONFIG_2000DEG;
            break;
        default:
            FW_ASSERT(0, range.e);
            break;
    }
    return registerValue;
}

}  // namespace asteIMU