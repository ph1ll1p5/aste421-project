module LED {
    @ Component to receive command from Led Controller to turn on/off Led
    passive component Led {

        ##############################################################################
        #### Uncomment the following examples to start customizing your component ####
        ##############################################################################

        @ Telemetry channel to report Led on/off
        telemetry LedState: Fw.On

        @ Event to log LED on/off state change
        event LedOnOffState(onOff: Fw.On) severity activity low format "LED is {}"

        @ Port to receive on/off command for LED
        sync input port cmdInLed: Drv.GpioWrite

        @ Port to send Led on/off command to gpio driver
        output port gpioSet: Drv.GpioWrite

        # EXAMPLES #################################
        # @ Example async command
        # async command COMMAND_NAME(param_name: U32)

        # @ Example telemetry counter
        # telemetry ExampleCounter: U64

        # @ Example event
        # event ExampleStateEvent(example_state: Fw.On) severity activity high id 0 format "State set to {}"

        # @ Example parameter
        # param PARAMETER_NAME: U32
        # ############################################################

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        # Commented out, LED component does not accept ground commands directly
        #@ Enables command handling
        #import Fw.Command

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