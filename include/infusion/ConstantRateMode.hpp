#pragma once

#include "infusion/InfusionMode.hpp"

// ---------------------------------------------------------------------------
// ConstantRateMode
// Delivers fluid at a fixed mL/hr rate for the entire infusion.
//
// computeTargetRate() always returns the same configured rate.
//
// Flow accuracy:
//   At 1 step = 1 µL, rate accuracy depends only on timer precision.
//   Zephyr k_sleep(K_USEC) gives ~±1 µs accuracy → well within ±5%.
//
// Example:
//   rate = 120 mL/hr
//   stepInterval = 3,600,000 / 120 = 30,000 µs = 30 ms per step
// ---------------------------------------------------------------------------
class ConstantRateMode final : public InfusionMode {
public:
    ConstantRateMode(IStepperDriver&   stepper,
                     IEncoderDriver&   encoder,
                     VolumeTracker&    tracker,
                     OcclusionMonitor& monitor,
                     uint32_t          targetVolumeUL,
                     float             rateMlPerHr) noexcept;

    // Set a new rate at runtime (takes effect next run() call).
    void setRate(float rateMlPerHr) noexcept;
    float getRate() const noexcept;

public:
    float computeTargetRate() noexcept override;

private:
    float rateMlPerHr_;
};
