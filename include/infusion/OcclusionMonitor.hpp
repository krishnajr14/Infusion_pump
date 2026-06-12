#pragma once

#include "infusion/hal/IPressureSensor.hpp"
#include "infusion/hal/IAlarmObserver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------------
// OcclusionMonitor
// Polls LPS22HB every tick. Raises OCCLUSION alarm when pressure exceeds
// (baseline + thresholdHPa). Clears alarm when pressure drops back below
// threshold. Notifies registered IAlarmObserver instances.
//
// IEC 60601-2-24 requirement:
//   Occlusion alarm must fire within the response time defined in Table 101.
//   With a 200 µs tick rate, alarm latency is well within spec.
//
// Baseline is captured on first call to poll() after reset().
// This auto-zeroes for different atmospheric pressures.
//
// Max observers: 4 (UART + LED + buzzer + spare)
// ---------------------------------------------------------------------------

inline constexpr size_t MAX_OCCLUSION_OBSERVERS = 4U;
inline constexpr uint32_t DEFAULT_THRESHOLD_HPA  = 50U;  // +50 hPa above baseline

class OcclusionMonitor {
public:
    OcclusionMonitor(IPressureSensor& sensor,
                     uint32_t         thresholdHPa = DEFAULT_THRESHOLD_HPA) noexcept;

    // Register an alarm observer. Returns false if table full.
    bool registerObserver(IAlarmObserver* obs) noexcept;

    // Poll sensor — call every firmware tick.
    // Raises/clears OCCLUSION alarm on threshold crossing.
    void poll() noexcept;

    // Reset baseline — call before starting a new infusion.
    void resetBaseline() noexcept;

    // Inspection
    bool     isOccluded()      const noexcept;
    uint32_t baselineHPa()     const noexcept;
    uint32_t lastPressureHPa() const noexcept;
    uint8_t  observerCount()   const noexcept;

private:
    IPressureSensor& sensor_;
    uint32_t         thresholdHPa_;
    uint32_t         baselineHPa_{0U};
    uint32_t         lastPressureHPa_{0U};
    bool             baselineCaptured_{false};
    bool             occluded_{false};

    std::array<IAlarmObserver*, MAX_OCCLUSION_OBSERVERS> observers_{};
    uint8_t observerCount_{0U};

    void raiseAlarm() noexcept;
    void clearAlarm() noexcept;
};
