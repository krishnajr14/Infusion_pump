#include "infusion/InfusionMode.hpp"

// ---------------------------------------------------------------------------
InfusionMode::InfusionMode(IStepperDriver&   stepper,
                            IEncoderDriver&   encoder,
                            VolumeTracker&    tracker,
                            OcclusionMonitor& monitor,
                            uint32_t          targetVolumeUL) noexcept
    : stepper_(stepper)
    , encoder_(encoder)
    , tracker_(tracker)
    , monitor_(monitor)
    , targetVolumeUL_(targetVolumeUL)
    , activeMode_(this)
{}

// ---------------------------------------------------------------------------
// run() — Template Method skeleton. Called once per logical cycle.
// Order is fixed: compute rate → apply rate → check alarms.
// ---------------------------------------------------------------------------
// Inside run()
void InfusionMode::run() noexcept {
    /* LCOV_EXCL_START */
    if (!running_ || complete_) {
        return;
    }
    /* LCOV_EXCL_STOP */
    // Step 1: polymorphic rate computation (overridden per mode)
    float rate = activeMode_->computeTargetRate();

    // Step 2: shared — convert rate to step interval
    applyRate(rate);

    // Step 3: shared + mode-specific alarm checks
    checkAlarms();
}

// ---------------------------------------------------------------------------
// tick() — called every 200 µs from Zephyr thread.
// Counts elapsed time and fires step pulses at the correct interval.
// Also polls the occlusion monitor and accumulates encoder ticks.
// ---------------------------------------------------------------------------
// Inside tick()
void InfusionMode::tick() noexcept {
    /* LCOV_EXCL_START */
    if (!running_ || complete_) {
        return;
    }
    /* LCOV_EXCL_STOP */

    // Poll pressure sensor every tick
    monitor_.poll();

    // Accumulate encoder ticks into volume tracker
    const uint32_t newTicks = encoder_.getTicks();
    encoder_.resetTicks();
    tracker_.addTicks(newTicks);

    // Step motor when interval has elapsed
    if (stepIntervalUs_ > 0U) {
        ticksSinceStep_ += TICK_US;
        if (ticksSinceStep_ >= stepIntervalUs_) {
            ticksSinceStep_ = 0U;
            stepper_.step();
        }
    }

    // Update ramp progress for LinearRampMode
    activeMode_->advanceTime(TICK_US);
}

// ---------------------------------------------------------------------------
void InfusionMode::start() noexcept {
    running_        = true;
    complete_       = false;
    ticksSinceStep_ = 0U;
    stepper_.enable();
    stepper_.setDirection(true);
    monitor_.resetBaseline();
    tracker_.reset();
    encoder_.resetTicks();
}

void InfusionMode::stop() noexcept {
    running_ = false;
    stepper_.disable();
}

void InfusionMode::pause() noexcept {
    running_ = false;
    stepper_.disable();
}

void InfusionMode::resume() noexcept {
    running_ = true;
    stepper_.enable();
}

// ---------------------------------------------------------------------------
// switchMode — swap active mode without stopping infusion.
// New mode's computeTargetRate() takes effect on next run() call.
// ---------------------------------------------------------------------------
void InfusionMode::switchMode(InfusionMode* newMode) noexcept {
    if (newMode != nullptr) {
        activeMode_ = newMode;
    }
}

// ---------------------------------------------------------------------------
bool InfusionMode::isRunning()  const noexcept { return running_;  }
bool InfusionMode::isComplete() const noexcept { return complete_; }
uint32_t InfusionMode::stepIntervalUs() const noexcept { return stepIntervalUs_; }

// ---------------------------------------------------------------------------
// applyRate — converts mL/hr to inter-step delay in µs.
//
// Derivation (1 step = 1 µL):
//   steps/hr  = rate_mLperHr × 1000
//   steps/sec = rate_mLperHr × 1000 / 3600
//   µs/step   = 3,600,000 / rate_mLperHr
//
// Clamps rate to [1, 500] mL/hr per IEC 60601-2-24.
// ---------------------------------------------------------------------------
void InfusionMode::applyRate(float mLperHr) noexcept {
    if (mLperHr < MIN_RATE_ML_HR) { mLperHr = MIN_RATE_ML_HR; }
    if (mLperHr > MAX_RATE_ML_HR) { mLperHr = MAX_RATE_ML_HR; }

    stepIntervalUs_ = static_cast<uint32_t>(3'600'000.0f / mLperHr);
}

// ---------------------------------------------------------------------------
// checkAlarms — volume completion + occlusion + mode-specific checks.
// ---------------------------------------------------------------------------
void InfusionMode::checkAlarms() noexcept {
    // Volume complete check
    if (tracker_.volumeUL() >= targetVolumeUL_) {
        complete_ = true;
        running_  = false;
        stepper_.disable();
    }

    // Occlusion: stop motor if blocked
    if (monitor_.isOccluded()) {
        running_ = false;
        stepper_.disable();
    }

    // Mode-specific checks (override in subclass if needed)
    activeMode_->onCheckAlarms();
}

// ---------------------------------------------------------------------------
// advanceTime — default no-op. LinearRampMode overrides this.
// ---------------------------------------------------------------------------
void InfusionMode::advanceTime(uint32_t /*us*/) noexcept {}
