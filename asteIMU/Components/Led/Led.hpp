// ======================================================================
// \title  Led.hpp
// \author phill
// \brief  hpp file for Led component implementation class
// ======================================================================

#ifndef LED_Led_HPP
#define LED_Led_HPP

#include "asteIMU/Components/Led/LedComponentAc.hpp"

namespace LED {

class Led final : public LedComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct Led object
    Led(const char* const compName  //!< The component name
    );

    //! Destroy Led object
    ~Led();

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

}  // namespace LED

#endif
