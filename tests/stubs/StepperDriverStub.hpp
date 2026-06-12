#pragma once
#include "infusion/hal/IStepperDriver.hpp"

class StepperDriverStub final : public IStepperDriver {
public:
    void enable()  noexcept override { enabled_ = true;  }
    void disable() noexcept override { enabled_ = false; }
    void step()    noexcept override { ++stepCount_; }
    void setDirection(bool fwd) noexcept override { forward_ = fwd; }
    uint32_t getStepCount()  const noexcept override { return stepCount_; }
    void     resetStepCount()      noexcept override { stepCount_ = 0U;  }

    bool     isEnabled()  const noexcept { return enabled_;   }
    bool     isForward()  const noexcept { return forward_;   }
    uint32_t steps()      const noexcept { return stepCount_; }
    void resetAll() noexcept {
        enabled_ = false; forward_ = true; stepCount_ = 0U;
    }
private:
    bool     enabled_{false};
    bool     forward_{true};
    uint32_t stepCount_{0U};
};
