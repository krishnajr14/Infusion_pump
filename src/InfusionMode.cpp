#include "infusion/InfusionMode.hpp"

// Conditional include to maintain zero-dependency rule on host machine
#ifdef __ZEPHYR__
#include <zephyr/sys/printk.h>
#else
// Mock printk for native host tests so compilation doesn't crash
#define printk(...) (void)0
#endif

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
void InfusionMode::run() noexcept {
    /* LCOV_EXCL_START */
    if (!running_ || complete_) {
        return;
    }

    monitor_.poll();

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
// InfusionMode.cpp — tick(): remove the poll call and pollDivider_ block entirely
// ---------------------------------------------------------------------------
// tick() — called every 200 µs from a Zephyr k_timer ISR.
// Counts elapsed time and fires step pulses at the correct interval.
// Accumulates encoder ticks into the volume tracker.
// TEMPORARY: instrumented to diagnose why tracker_.addTicks() isn't
// producing any visible volume — remove the #ifdef block once resolved.
// ---------------------------------------------------------------------------
void InfusionMode::tick() noexcept {
    if (!running_ || complete_) {
        return;
    }

    const uint32_t newTicks = encoder_.getTicks();
    encoder_.resetTicks();
    tracker_.addTicks(newTicks);

#ifdef __ZEPHYR__
    static uint32_t debug_ctr = 0U;
    if (newTicks > 0U && (++debug_ctr % 50U == 0U)) {
        // Throttled — this runs from a 200µs ISR, an unthrottled printk
        // here would reintroduce the exact blocking-in-ISR problem we
        // already fixed for the pressure sensor.
        printk("[DBG] newTicks=%u accUL=%u\r\n", newTicks, tracker_.volumeUL());
    }
#endif

    if (stepIntervalUs_ > 0U) {
        ticksSinceStep_ += TICK_US;
        if (ticksSinceStep_ >= stepIntervalUs_) {
            ticksSinceStep_ = 0U;
            stepper_.step();
        }
    }

    activeMode_->advanceTime(TICK_US);
}

// ---------------------------------------------------------------------------
void InfusionMode::start() noexcept {
    printk("=== Infusion Pump Started ===\n");
    ticksSinceStep_ = 0U;
    stepper_.enable();
    stepper_.setDirection(true);
    monitor_.resetBaseline();
    tracker_.reset();
    encoder_.resetTicks();
    running_        = true;
    complete_       = false;
}

void InfusionMode::stop() noexcept {
    printk("=== Infusion Pump Stopped ===\n");
    running_ = false;
    stepper_.disable();
}

void InfusionMode::pause() noexcept {
    printk("=== Infusion Pump Paused ===\n");
    running_ = false;
    stepper_.disable();
}

void InfusionMode::resume() noexcept {
    printk("=== Infusion Pump Resumed ===\n");
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
// ---------------------------------------------------------------------------
void InfusionMode::applyRate(float mLperHr) noexcept {
    if (mLperHr <= 0.0f) {
        stepIntervalUs_ = 0U;   // true 0 Hz, bypasses MIN_RATE floor
        return;
    }
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