module asteIMU {

    @ Range of the accelerometer in G's, integer values represent the conversion factor for the raw values from
    @ the accelerometer registers to Gs
    enum AccelerationRange : U16 {
        RANGE_2G = 16384 
        RANGE_4G = 8192
        RANGE_8G = 4096
        RANGE_16G = 2048 
    }

    @ Range of the gyroscope in degrees per second, integer values represent the conversion factor for the raw values from
    @ the gyroscope registers to 10ths of a degree per second
    enum GyroscopeRange : U16 {
        RANGE_250DEG = 1310
        RANGE_500DEG = 655
        RANGE_1000DEG = 328
        RANGE_2000DEG = 164 
    }

    @ Struct representing X, Y, Z data
    struct GeometricVector3 {
        x: F32 @< X component of the vector
        y: F32 @< Y component of the vector
        z: F32 @< Z component of the vector
    }

    @ Struct representing ImuData
    struct ImuData {
        @ Accelerations from the accelerometer
        acceleration: GeometricVector3
        
        @ Angular rates from the gyroscope
        rotation: GeometricVector3

        @ Temperature in degrees Celsius
        temperature: F32
    }
}