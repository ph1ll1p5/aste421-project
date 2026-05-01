module asteIMU {

    module QueueSizes {
        constant imuManager  = 10
    }

    instance imuDriver: Drv.LinuxI2cDriver base id 0xE0006000 {
        phase Fpp.ToCpp.Phases.configComponents """
        if (not asteIMU::imuDriver.open(state.device)) {
            Fw::Logger::log("[ERROR] IMU open failed\\n");
        }
        else {
            Fw::Logger::log("[INFO] IMU open successful\\n");
        }
        """
    }
}
