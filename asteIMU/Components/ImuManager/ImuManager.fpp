module asteIMU {
    @ Imu Manager Component
    queued component ImuManager {

        @ Port for I2C bus communication
        output port busWriteRead: Drv.I2cWriteRead
        
        @ Port for I2C bus communication
        output port busWrite: Drv.I2c

        @ Scheduling port for reading from IMU and writing to telemetry
        sync input port run: Svc.Sched

        @ Telemetry channel for IMU data
        telemetry Reading: ImuData

        event AccelerometerRangeUpdated(
            newRange: AccelerationRange
        ) severity activity high format "Accelerometer range updated to {}"

        event GyroscopeRangeUpdated(
            newRange: GyroscopeRange
        ) severity activity high format "Gyroscope range updated to {}"

        event I2cError(
            address: U32,
            status: Drv.I2cStatus
        ) severity warning high format "I2C error on address {} with status {}" throttle 5

        @ Parameter for setting the accelerometer range
        param ACCELEROMETER_RANGE: AccelerationRange default AccelerationRange.RANGE_2G

        @ Parameter for setting the gyroscope range
        param GYROSCOPE_RANGE: GyroscopeRange default GyroscopeRange.RANGE_250DEG

        @ Command to force a RESET
        async command RESET()

        @ ImuSM instance
        state machine instance imuStateMachine: ImuStateMachine

        ##############################################################################
        #### Uncomment the following examples to start customizing your component ####
        ##############################################################################

        # @ Example async command
        # async command COMMAND_NAME(param_name: U32)

        # @ Example telemetry counter
        # telemetry ExampleCounter: U64

        # @ Example event
        # event ExampleStateEvent(example_state: Fw.On) severity activity high id 0 format "State set to {}"

        # @ Example port: receiving calls from the rate group
        # sync input port run: Svc.Sched

        # @ Example parameter
        # param PARAMETER_NAME: U32

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

    }
}