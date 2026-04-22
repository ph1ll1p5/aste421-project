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
    // TODO return
    
}

}  // namespace LED
