#pragma once
#include "infusion/hal/IAlarmObserver.hpp"
#include <array>
#include <cstdint>

inline constexpr size_t MAX_ALARM_LOG = 16U;

class AlarmObserverStub final : public IAlarmObserver {
public:
    void onAlarm(AlarmType type) noexcept override {
        if (raiseCount_ < MAX_ALARM_LOG)
            raisedLog_[raiseCount_++] = type;
    }
    void onAlarmCleared(AlarmType type) noexcept override {
        if (clearCount_ < MAX_ALARM_LOG)
            clearedLog_[clearCount_++] = type;
    }
    uint8_t   raiseCount()  const noexcept { return raiseCount_;  }
    uint8_t   clearCount()  const noexcept { return clearCount_;  }
    AlarmType lastRaised()  const noexcept { return raisedLog_[raiseCount_ - 1U]; }
    AlarmType lastCleared() const noexcept { return clearedLog_[clearCount_ - 1U]; }
    void reset() noexcept {
        raiseCount_ = 0U; clearCount_ = 0U;
        raisedLog_.fill(AlarmType::OCCLUSION);
        clearedLog_.fill(AlarmType::OCCLUSION);
    }
private:
    std::array<AlarmType, MAX_ALARM_LOG> raisedLog_{};
    std::array<AlarmType, MAX_ALARM_LOG> clearedLog_{};
    uint8_t raiseCount_{0U};
    uint8_t clearCount_{0U};
};
