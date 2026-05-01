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

Drv::I2cStatus ImuManager ::read(ImuData& imuData) {
    U8 data[DATA_LENGTH];
    U8 registerAddress = DATA_BASE_REGISTER;

    Fw::Buffer writeBuffer(&registerAddress, 1);
    Fw::Buffer readBuffer(data, DATA_LENGTH);
    Drv::I2cStatus status = this->bus_write(writeBuffer, readBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        return status;
    }
    RawImuData raw = this->deserialize_raw_data(readBuffer);

    Fw::ParamValid paramValid;
    const AccelerationRange accelerationRange = this->paramGet_ACCELEROMETER_RANGE(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));
    const GyroscopeRange gyroscopeRange = this->paramGet_GYROSCOPE_RANGE(paramValid);
    FW_ASSERT(paramValid != Fw::ParamValid::INVALID, static_cast<FwAssertArgType>(paramValid));

    imuData = this->convert_raw_data(raw, accelerationRange, gyroscopeRange);
    return status;
}

RawImuData ImuManager ::deserialize_raw_data(Fw::Buffer& buffer) {
    auto deserializer = buffer.getDeserializer();
    RawImuData raw;
    // BNO055 accelerometer data: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    deserializer.deserialize(raw.acceleration[0]);
    deserializer.deserialize(raw.acceleration[1]);
    deserializer.deserialize(raw.acceleration[2]);
    raw.temperature = 0;  // Read separately if needed
    // Gyroscope data follows accelerometer
    deserializer.deserialize(raw.gyroscope[0]);
    deserializer.deserialize(raw.gyroscope[1]);
    deserializer.deserialize(raw.gyroscope[2]);
    return raw;
}

ImuData ImuManager ::convert_raw_data(const RawImuData& raw,
                                      const AccelerationRange& accelerationRange,
                                      const GyroscopeRange& gyroscopeRange) {
    asteIMU::ImuData imuData;
    // BNO055 in NDOF mode: accel scale is 1mg/LSB = 0.001 m/s^2 per LSB
    imuData.get_acceleration().set_x(static_cast<F32>(raw.acceleration[0]) / 100.0f);
    imuData.get_acceleration().set_y(static_cast<F32>(raw.acceleration[1]) / 100.0f);
    imuData.get_acceleration().set_z(static_cast<F32>(raw.acceleration[2]) / 100.0f);
    imuData.set_temperature(0.0f);
    // BNO055 in NDOF mode: gyro scale is 1/16 deg/s per LSB
    imuData.get_rotation().set_x(static_cast<F32>(raw.gyroscope[0]) / 16.0f);
    imuData.get_rotation().set_y(static_cast<F32>(raw.gyroscope[1]) / 16.0f);
    imuData.get_rotation().set_z(static_cast<F32>(raw.gyroscope[2]) / 16.0f);
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