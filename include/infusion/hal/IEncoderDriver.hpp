#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// IEncoderDriver
// Pure interface for quadrature encoder on NUCLEO-F446RE.
//
// Role in infusion pump:
//   Independently measures actual shaft rotation.
//   1 encoder tick = 1 µL  (same as 1 stepper microstep)
//   VolumeTracker accumulates encoder ticks as primary volume measurement.
//   If ticks fall behind commanded steps beyond STALL_THRESHOLD → rate error.
//
// Concrete implementations:
//   Production : ZephyrEncoderDriver  in zephyr/main.cpp
//   Test       : EncoderDriverStub    in tests/stubs/
// ---------------------------------------------------------------------------
class IEncoderDriver {
public:
    virtual ~IEncoderDriver() = default;  // LCOV_EXCL_LINE

    // Returns cumulative tick count since last reset.
    virtual uint32_t getTicks() noexcept = 0;
    // Reset tick counter to zero.
    virtual void resetTicks() noexcept = 0;

    // Simulate ticks (used by Zephyr ISR; stub injects directly).
    virtual void addTicks(uint32_t ticks) noexcept = 0;
};
