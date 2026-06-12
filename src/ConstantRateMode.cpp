#include "infusion/ConstantRateMode.hpp"

// ---------------------------------------------------------------------------
ConstantRateMode::ConstantRateMode(IStepperDriver&   stepper,
                                    IEncoderDriver&   encoder,
                                    VolumeTracker&    tracker,
                                    OcclusionMonitor& monitor,
                                    uint32_t          targetVolumeUL,
                                    float             rateMlPerHr) noexcept
    : InfusionMode(stepper, encoder, tracker, monitor, targetVolumeUL)
    , rateMlPerHr_(rateMlPerHr)
{}

// ---------------------------------------------------------------------------
// computeTargetRate — always returns the same configured rate.
// ---------------------------------------------------------------------------
float ConstantRateMode::computeTargetRate() noexcept {
    return rateMlPerHr_;
}

// ---------------------------------------------------------------------------
void ConstantRateMode::setRate(float rateMlPerHr) noexcept {
    rateMlPerHr_ = rateMlPerHr;
}

float ConstantRateMode::getRate() const noexcept {
    return rateMlPerHr_;
}
