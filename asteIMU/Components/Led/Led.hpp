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
};

}  // namespace LED

#endif
