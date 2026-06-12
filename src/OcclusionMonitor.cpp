#include "infusion/OcclusionMonitor.hpp"

// ---------------------------------------------------------------------------
OcclusionMonitor::OcclusionMonitor(IPressureSensor& sensor,
                                   uint32_t         thresholdHPa) noexcept
    : sensor_(sensor)
    , thresholdHPa_(thresholdHPa)
{
    observers_.fill(nullptr);
}

// ---------------------------------------------------------------------------
bool OcclusionMonitor::registerObserver(IAlarmObserver* obs) noexcept {
    if (obs == nullptr || observerCount_ >= MAX_OCCLUSION_OBSERVERS) {
        return false;
    }
    observers_[observerCount_++] = obs;
    return true;
}

// ---------------------------------------------------------------------------
// poll() — call every firmware tick (200 µs)
//
// First call after reset: captures baseline pressure.
// Subsequent calls: checks if current pressure exceeds baseline + threshold.
// Raises alarm on rising edge, clears on falling edge (hysteresis-free).
// ---------------------------------------------------------------------------
void OcclusionMonitor::poll() noexcept {
    if (!sensor_.isReady()) {
        return;
    }

    lastPressureHPa_ = sensor_.readPressureHPa();

    if (!baselineCaptured_) {
        baselineHPa_      = lastPressureHPa_;
        baselineCaptured_ = true;
        return;
    }

    const bool overThreshold =
        (lastPressureHPa_ >= baselineHPa_ + thresholdHPa_);

    if (overThreshold && !occluded_) {
        raiseAlarm();
    } else if (!overThreshold && occluded_) {
        clearAlarm();
    }
}

// ---------------------------------------------------------------------------
void OcclusionMonitor::resetBaseline() noexcept {
    baselineCaptured_ = false;
    baselineHPa_      = 0U;
    occluded_         = false;
}

// ---------------------------------------------------------------------------
bool OcclusionMonitor::isOccluded() const noexcept {
    return occluded_;
}

uint32_t OcclusionMonitor::baselineHPa() const noexcept {
    return baselineHPa_;
}

uint32_t OcclusionMonitor::lastPressureHPa() const noexcept {
    return lastPressureHPa_;
}

uint8_t OcclusionMonitor::observerCount() const noexcept {
    return observerCount_;
}

// ---------------------------------------------------------------------------
void OcclusionMonitor::raiseAlarm() noexcept {
    occluded_ = true;
    for (uint8_t i = 0U; i < observerCount_; ++i) {
        observers_[i]->onAlarm(AlarmType::OCCLUSION);
    }
}

void OcclusionMonitor::clearAlarm() noexcept {
    occluded_ = false;
    for (uint8_t i = 0U; i < observerCount_; ++i) {
        observers_[i]->onAlarmCleared(AlarmType::OCCLUSION);
    }
}
