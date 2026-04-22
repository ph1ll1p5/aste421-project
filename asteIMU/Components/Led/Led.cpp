// ======================================================================
// \title  Led.cpp
// \author phill
// \brief  cpp file for Led component implementation class
// ======================================================================

#include "asteIMU/Components/Led/Led.hpp"

namespace LED {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

Led ::Led(const char* const compName) : LedComponentBase(compName) {}

Led ::~Led() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

Drv::GpioStatus Led ::cmdInLed_handler(FwIndexType portNum, const Fw::Logic& state) {
    // Port cmdInLed received Fw::Logic --> Therefore, check for logic 'HIGH' (LED ON) or logic 'LOW' (LED OFF)
   
    //pdate and Report Telemetry
    m_state = (state == Fw::Logic::HIGH) ? Fw::On::ON : Fw::On::OFF; // If Fw::Logic is 'HIGH', set m_state 'ON' , else, m_state 'OFF'
    this->tlmWrite_LedState(m_state);

    // Port may not be connected, so check before sending output signal to GPIO
    // Forward signal to GPIO
    if (this->isConnected_gpioSet_OutputPort(0)) {
        this->gpioSet_out(0, (Fw::On::ON == this->m_state) ? Fw::Logic::HIGH : Fw::Logic::LOW);
        // Emit event
        this->log_ACTIVITY_LO_LedOnOffState(this->m_state);
        // Return GPIO status back to caller. When LedController calls cmdInLed_handler -> it will receive this back
        return Drv::GpioStatus::OP_OK; // Returns status that operation succeeded
    }

    // Emit the event --> Ties back to activity definition in .fpp file
    this->log_ACTIVITY_LO_LedOnOffState(this->m_state);

    return Drv::GpioStatus::NOT_OPENED; // Failure if Pin was never opened
}

}  // namespace LED
