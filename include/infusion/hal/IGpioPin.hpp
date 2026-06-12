#pragma once

// ---------------------------------------------------------------------------
// IGpioPin — single digital output pin abstraction.
// ---------------------------------------------------------------------------
class IGpioPin {
public:
    virtual ~IGpioPin() = default;  // LCOV_EXCL_LINE

    virtual void setHigh() noexcept = 0;
    virtual void setLow()  noexcept = 0;
    virtual bool read()    noexcept = 0;
};
