#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// VolumeTracker — encoder-tick accumulator
//
// Geometry assumption (per spec):
//   1 encoder tick = 1 µL delivered
//
// Unlike the syringe pump (which used fixed-point nL arithmetic),
// this version accumulates directly in µL — no conversion needed.
//
// The encoder is the PRIMARY volume measurement source.
// Stepper step count is secondary (commanded), encoder ticks are actual.
// ---------------------------------------------------------------------------
class VolumeTracker {
public:
    VolumeTracker() noexcept = default;

    // Add encoder ticks — called from InfusionMode::tick()
    void addTicks(uint32_t ticks) noexcept;

    // Reset accumulator to zero
    void reset() noexcept;

    // Delivered volume in µL (1 tick = 1 µL)
    uint32_t volumeUL()   const noexcept;

    // Raw tick count since last reset()
    uint32_t tickCount()  const noexcept;

private:
    uint32_t accUL_{0U};
    uint32_t ticks_{0U};
};
