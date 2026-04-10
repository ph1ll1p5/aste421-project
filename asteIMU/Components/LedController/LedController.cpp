// ======================================================================
// \title  LedController.cpp
// \author phill
// \brief  cpp file for LedController component implementation class
// ======================================================================

#include "asteIMU/Components/LedController/LedController.hpp"

namespace LEDController {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

LedController ::LedController(const char* const compName) : LedControllerComponentBase(compName) {}

LedController ::~LedController() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void LedController ::TODO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace LEDController
