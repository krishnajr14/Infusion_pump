#pragma once

#include "infusion/hal/IStepperDriver.hpp"
#include "infusion/hal/IEncoderDriver.hpp"
#include "infusion/VolumeTracker.hpp"
#include "infusion/OcclusionMonitor.hpp"
#include <cstdint>

// ---------------------------------------------------------------------------
// InfusionMode — abstract base class, Template Method pattern
//
// Fixed skeleton (run()):
//   1. computeTargetRate()  ← OVERRIDE in each concrete mode
//   2. applyRate()          ← shared: converts mL/hr → step interval
//   3. checkAlarms()        ← shared: volume complete + occlusion
//
// Flow rate formula (1 step = 1 µL):
//   µs/step = 3,600,000 / rate_mLperHr
//
// Accuracy target: ±5% over 1–500 mL/hr (IEC 60601-2-24)
//
// Concrete subclasses:
//   ConstantRateMode  — fixed rate
//   LinearRampMode    — linearly ramps startRate → targetRate
//
// Mode-switch without restart:
//   switchMode(newMode) swaps the active polymorphic target.
//   New mode's computeTargetRate() takes effect on next run() call.
// ---------------------------------------------------------------------------
class InfusionMode {
public:
    InfusionMode(IStepperDriver&   stepper,
                 IEncoderDriver&   encoder,
                 VolumeTracker&    tracker,
                 OcclusionMonitor& monitor,
                 uint32_t          targetVolumeUL) noexcept;

    virtual ~InfusionMode() = default;  // LCOV_EXCL_LINE

    // Template Method — fixed skeleton.
    void run()    noexcept;

    // Tick — called every 200 µs from Zephyr thread.
    void tick()   noexcept;

    // Lifecycle control
    void start()  noexcept;
    void stop()   noexcept;
    void pause()  noexcept;
    void resume() noexcept;

    // Mode-switch without restart.
    void switchMode(InfusionMode* newMode) noexcept;

    // Inspection
    bool     isRunning()      const noexcept;
    bool     isComplete()     const noexcept;
    uint32_t stepIntervalUs() const noexcept;

protected:
    // ── Template Method hooks ─────────────────────────────────────────────

    // Returns target rate in mL/hr for this tick. MUST override.
    virtual float computeTargetRate() noexcept = 0;

    // Called by tick() to advance internal time counter.
    // Default: no-op. LinearRampMode overrides to update elapsed time.
    virtual void advanceTime(uint32_t us) noexcept;

    // Optional mode-specific alarm checks beyond occlusion.
    virtual void onCheckAlarms() noexcept {}

    // Subclass access to hardware
    IStepperDriver&   stepper()  noexcept { return stepper_;  }
    IEncoderDriver&   encoder()  noexcept { return encoder_;  }
    VolumeTracker&    tracker()  noexcept { return tracker_;  }
    OcclusionMonitor& monitor()  noexcept { return monitor_;  }

private:
    IStepperDriver&   stepper_;
    IEncoderDriver&   encoder_;
    VolumeTracker&    tracker_;
    OcclusionMonitor& monitor_;

    uint32_t targetVolumeUL_;
    uint32_t stepIntervalUs_{0U};
    uint32_t ticksSinceStep_{0U};
    uint16_t pollDivider_{0U};
    static constexpr uint16_t POLL_EVERY_N_TICKS = 50U; // 100us tick * 2 * 50 = 10ms cadence
    uint16_t startSettleTicks_{0U};
    static constexpr uint16_t START_SETTLE_TICKS = 25U;  // 25 * 200us = 5ms grace window — tune to your actual observed transient duration


    bool running_{false};
    bool complete_{false};

    InfusionMode* activeMode_{nullptr};

    // Private Template Method steps
    void applyRate(float mLperHr) noexcept;
    void checkAlarms()            noexcept;

    static constexpr float    MIN_RATE_ML_HR = 1.0f;
    static constexpr float    MAX_RATE_ML_HR = 500.0f;
    static constexpr uint32_t TICK_US        = 200U;
};
