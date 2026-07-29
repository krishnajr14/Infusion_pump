#include "infusion/LinearRampMode.hpp"

// ---------------------------------------------------------------------------
LinearRampMode::LinearRampMode(IStepperDriver&   stepper,
                                IEncoderDriver&   encoder,
                                VolumeTracker&    tracker,
                                OcclusionMonitor& monitor,
                                uint32_t          targetVolumeUL,
                                float             startRateMlPerHr,
                                float             targetRateMlPerHr,
                                uint32_t          rampDurationUs) noexcept
    : InfusionMode(stepper, encoder, tracker, monitor, targetVolumeUL)
    , startRateMlPerHr_(startRateMlPerHr)
    , targetRateMlPerHr_(targetRateMlPerHr)
    , rampDurationUs_(rampDurationUs)
    , currentRate_(startRateMlPerHr)
{}

// ---------------------------------------------------------------------------
// computeTargetRate — linear interpolation between start and target.
//
//   progress = elapsedUs / rampDurationUs   clamped [0.0, 1.0]
//   rate     = start + (target - start) * progress
//
// After ramp completes, returns targetRate (steady state).
// ---------------------------------------------------------------------------
float LinearRampMode::computeTargetRate() noexcept {
    if (rampDurationUs_ == 0U || elapsedUs_ >= rampDurationUs_) {
        currentRate_ = targetRateMlPerHr_;
        return currentRate_;
    }

    const float progress = static_cast<float>(elapsedUs_)
                         / static_cast<float>(rampDurationUs_);

    currentRate_ = startRateMlPerHr_
                 + (targetRateMlPerHr_ - startRateMlPerHr_) * (progress*2.0f);

    return currentRate_;
}

// ---------------------------------------------------------------------------
// advanceTime — called every tick() to update elapsed counter.
// Overrides the base class no-op.
// ---------------------------------------------------------------------------
void LinearRampMode::advanceTime(uint32_t us) noexcept {
    if (elapsedUs_ < rampDurationUs_) {
        elapsedUs_ += us;
        if (elapsedUs_ > rampDurationUs_) {
            elapsedUs_ = rampDurationUs_;
        }
    }
}

// ---------------------------------------------------------------------------
void LinearRampMode::resetRamp() noexcept {
    elapsedUs_   = 0U;
    currentRate_ = startRateMlPerHr_;
}

float    LinearRampMode::currentRate()  const noexcept { return currentRate_;       }
float    LinearRampMode::startRate()    const noexcept { return startRateMlPerHr_;  }
float    LinearRampMode::targetRate()   const noexcept { return targetRateMlPerHr_; }
bool     LinearRampMode::rampComplete() const noexcept { return elapsedUs_ >= rampDurationUs_; }
uint32_t LinearRampMode::elapsedUs()    const noexcept { return elapsedUs_;         }

void LinearRampMode::advanceUs(uint32_t us) noexcept {
    advanceTime(us);
}
