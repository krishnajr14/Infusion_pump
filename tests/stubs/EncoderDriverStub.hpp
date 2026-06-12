#pragma once
#include "infusion/hal/IEncoderDriver.hpp"

class EncoderDriverStub final : public IEncoderDriver {
public:
    uint32_t getTicks() const noexcept override { return ticks_; }
    void resetTicks()         noexcept override { ticks_ = 0U;   }
    void addTicks(uint32_t t) noexcept override { ticks_ += t;   }

    void reset() noexcept { ticks_ = 0U; }
private:
    uint32_t ticks_{0U};
};
