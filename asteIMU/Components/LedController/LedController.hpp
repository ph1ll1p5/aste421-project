// ======================================================================
// \title  LedController.hpp
// \author phill
// \brief  hpp file for LedController component implementation class
// ======================================================================

#ifndef LEDController_LedController_HPP
#define LEDController_LedController_HPP

#include "asteIMU/Components/LedController/LedControllerComponentAc.hpp"

namespace LEDController {

class LedController final : public LedControllerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct LedController object
    LedController(const char* const compName  //!< The component name
    );

    //! Destroy LedController object
    ~LedController();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command TODO
    //!
    //! TODO
    void TODO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                         U32 cmdSeq            //!< The command sequence number
                         ) override;
};

}  // namespace LEDController

#endif
