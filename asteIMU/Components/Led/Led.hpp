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
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for cmdIn
    //!
    //! Port to receive on/off command for LED
    Drv::GpioStatus cmdInLed_handler(FwIndexType portNum,  //!< The port number
                                  const Fw::Logic& state) override;

    //! Tracks the most recently commanded LED state
    Fw::On m_state = Fw::On::OFF; // stores whether the LED is ON or OFF, and start it as OFF
};

}  // namespace LED

#endif
