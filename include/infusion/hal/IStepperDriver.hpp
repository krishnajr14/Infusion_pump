#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// IStepperDriver
// Pure interface for TMC2209 SilentStepStick driver.
// Identical contract to syringe pump — same hardware, same interface.
//
// In the infusion pump:
//   1 microstep = 1 µL delivered  (simplified geometry assumption)
//
// Concrete implementations:
//   Production : ZephyrStepperDriver  in zephyr/main.cpp
//   Test       : StepperDriverStub    in tests/stubs/
// ---------------------------------------------------------------------------
class IStepperDriver {
public:
    virtual ~IStepperDriver() = default;  // LCOV_EXCL_LINE

    virtual void enable()                    noexcept = 0;
    virtual void disable()                   noexcept = 0;
    virtual void step()                      noexcept = 0;
    virtual void setDirection(bool forward)  noexcept = 0;
    virtual uint32_t getStepCount()    const noexcept = 0;
    virtual void     resetStepCount()        noexcept = 0;
};
