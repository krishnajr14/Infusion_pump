#pragma once
#include "infusion/hal/IPressureSensor.hpp"

class PressureSensorStub final : public IPressureSensor {
public:
    uint32_t readPressureHPa() const noexcept override { return pressure_; }
    bool     isReady()         const noexcept override { return ready_;    }

    void setPressure(uint32_t hpa) noexcept { pressure_ = hpa; }
    void setReady(bool r)          noexcept { ready_ = r;      }
    void reset()                   noexcept { pressure_ = 1013U; ready_ = true; }
private:
    uint32_t pressure_{1013U};  // standard atmospheric pressure
    bool     ready_{true};
};
