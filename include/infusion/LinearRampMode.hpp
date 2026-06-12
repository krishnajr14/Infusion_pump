#pragma once

#include "infusion/InfusionMode.hpp"

// ---------------------------------------------------------------------------
// LinearRampMode
// Linearly ramps delivery rate from startRateMlPerHr to targetRateMlPerHr
// over rampDurationUs microseconds, then holds at targetRate.
//
// computeTargetRate() calculates current rate based on elapsed ticks:
//
//   progress = elapsedUs / rampDurationUs   (clamped 0.0 → 1.0)
//   rate     = startRate + (targetRate - startRate) * progress
//
// Ramp profile example (startRate=1, targetRate=500, rampDuration=10s):
//
//   mL/hr
//   500 |                    ──────────────
//       |                ───/
//       |            ───/
//       |        ───/
//     1 |───────/
//       └────────────────────────────────► time (s)
//       0       5      10
//
// After ramp completes, behaves like ConstantRateMode at targetRate.
// ---------------------------------------------------------------------------
class LinearRampMode final : public InfusionMode {
public:
    LinearRampMode(IStepperDriver&   stepper,
                   IEncoderDriver&   encoder,
                   VolumeTracker&    tracker,
                   OcclusionMonitor& monitor,
                   uint32_t          targetVolumeUL,
                   float             startRateMlPerHr,
                   float             targetRateMlPerHr,
                   uint32_t          rampDurationUs) noexcept;

    // Inspection
    float    currentRate()    const noexcept;
    float    startRate()      const noexcept;
    float    targetRate()     const noexcept;
    bool     rampComplete()   const noexcept;
    uint32_t elapsedUs()      const noexcept;

    // Advance elapsed time (called by tick() or directly in tests)
    void advanceUs(uint32_t us) noexcept;

    // Reset ramp to beginning
    void resetRamp() noexcept;

public:
    float computeTargetRate() noexcept override;
    void  advanceTime(uint32_t us) noexcept override;

private:
    float    startRateMlPerHr_;
    float    targetRateMlPerHr_;
    uint32_t rampDurationUs_;
    uint32_t elapsedUs_{0U};
    float    currentRate_{0.0f};
};
