#pragma once
#include "infusion/hal/IGpioPin.hpp"

class GpioStub final : public IGpioPin {
public:
    void setHigh() noexcept override { state_ = true;  }
    void setLow()  noexcept override { state_ = false; }
    bool read()    noexcept override { return state_;  }
    bool state()   const noexcept   { return state_;  }
private:
    bool state_{false};
};
