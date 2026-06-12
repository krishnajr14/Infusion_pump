#include "infusion/VolumeTracker.hpp"

void VolumeTracker::addTicks(uint32_t ticks) noexcept {
    ticks_  += ticks;
    accUL_  += ticks;   // 1 tick = 1 µL
}

void VolumeTracker::reset() noexcept {
    ticks_ = 0U;
    accUL_ = 0U;
}

uint32_t VolumeTracker::volumeUL() const noexcept {
    return accUL_;
}

uint32_t VolumeTracker::tickCount() const noexcept {
    return ticks_;
}
