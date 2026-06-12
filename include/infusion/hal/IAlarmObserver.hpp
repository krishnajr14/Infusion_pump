#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// AlarmType — IEC 60601-2-24 alarm codes for infusion pump
// ---------------------------------------------------------------------------
enum class AlarmType : uint8_t {
    OCCLUSION        = 0,   // Downstream pressure exceeded threshold
    VOLUME_COMPLETE  = 1,   // Target volume fully delivered
    RATE_ERROR       = 2    // Encoder ticks diverged from commanded steps
};

// ---------------------------------------------------------------------------
// IAlarmObserver
// Pure interface. Implement once per notification channel (UART, LED).
// Register with OcclusionMonitor or InfusionMode alarm handling.
// Never allocates — use static globals or placement-new.
// ---------------------------------------------------------------------------
class IAlarmObserver {
public:
    virtual ~IAlarmObserver() = default;  // LCOV_EXCL_LINE

    virtual void onAlarm(AlarmType type)        noexcept = 0;
    virtual void onAlarmCleared(AlarmType type) noexcept = 0;
};
