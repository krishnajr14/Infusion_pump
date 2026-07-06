/*
 * main.cpp — Zephyr firmware entry point for Infusion Pump
 *
 * THIS IS THE ONLY FILE THAT INCLUDES ZEPHYR HEADERS.
 * All business logic in include/ and src/ has zero Zephyr dependencies.
 *
 * Hardware: NUCLEO-F446RE 
 * - TMC2209 stepper driver  (STEP/DIR/EN via GPIO)
 * - Quadrature encoder      (TIM2 in encoder mode)
 * - LPS22HB pressure sensor (I²C1)
 * - LED alarm indicator     (PA5)
 * - UART console            (USART2 via ST-LINK VCP)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_gpio.h>

#include "infusion/ConstantRateMode.hpp"
#include "infusion/LinearRampMode.hpp"
#include "infusion/VolumeTracker.hpp"
#include "infusion/OcclusionMonitor.hpp"
#include "infusion/hal/IStepperDriver.hpp"
#include "infusion/hal/IEncoderDriver.hpp"
#include "infusion/hal/IPressureSensor.hpp"
#include "infusion/hal/IAlarmObserver.hpp"

#include <stdio.h>   // Required for sscanf
#include <string.h>  // Required for strncmp, memset

// ============================================================
// Concrete HAL implementations
// ============================================================

class ZephyrStepperDriver final : public IStepperDriver {
public:
    ZephyrStepperDriver(const struct gpio_dt_spec step,
                        const struct gpio_dt_spec dir,
                        const struct gpio_dt_spec en) noexcept
        : step_(step), dir_(dir), en_(en) {}

    void enable()  noexcept override { gpio_pin_set_dt(&en_, 1); }
    void disable() noexcept override { gpio_pin_set_dt(&en_, 0); }
    void step()    noexcept override {
        gpio_pin_set_dt(&step_, 1);
        k_busy_wait(2U);
        gpio_pin_set_dt(&step_, 0);
        ++stepCount_;
    }
    void setDirection(bool fwd) noexcept override {
        gpio_pin_set_dt(&dir_, fwd ? 1 : 0);
    }
    uint32_t getStepCount()  const noexcept override { return stepCount_; }
    void     resetStepCount()      noexcept override { stepCount_ = 0U;  }

private:
    struct gpio_dt_spec step_, dir_, en_;
    uint32_t stepCount_{0U};
};

class ZephyrEncoderDriver final : public IEncoderDriver {
public:
    ZephyrEncoderDriver() noexcept {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        __DSB();

        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_1);

        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_1);

        TIM2->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_TIE);

        TIM2->SMCR  &= ~(TIM_SMCR_SMS); 
        TIM2->SMCR  |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1); 
        
        TIM2->CCMR1 |= (TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1);

        TIM2->CCER  |= TIM_CCER_CC1P;

        TIM2->CR1   |= TIM_CR1_CEN;
        TIM2->CNT    = 0U;
        lastFetchCnt_ = 0U;
    }

    uint32_t getTicks() const noexcept override {
        uint32_t current_cnt = TIM2->CNT;
        uint32_t delta = 0U;

        if (current_cnt >= lastFetchCnt_) {
            delta = current_cnt - lastFetchCnt_;
        } else {
            delta = (0xFFFFFFFFU - lastFetchCnt_) + current_cnt + 1U;
        }

        // Catch edge mechanical rollbacks or startup transients
        if (delta > 2000000000U) {
            return 0U;
        }

        // 1.5x geometric ratio scale down to 1:1 volume tracking resolution
        return (delta * 2U) / 3U;
    }

    void resetTicks() noexcept override {
        lastFetchCnt_ = TIM2->CNT;
    }

    void addTicks(uint32_t t) noexcept override {
        lastFetchCnt_ -= (t * 3U) / 2U;
    }

private:
    mutable uint32_t lastFetchCnt_{0U}; 
};

class ZephyrPressureSensor final : public IPressureSensor {
public:
    explicit ZephyrPressureSensor(const struct device* dev) noexcept
        : dev_(dev) {}

    uint32_t readPressureHPa() const noexcept override {
        struct sensor_value val{};
        struct sensor_value attr;
        attr.val1 = 10; 
        attr.val2 = 0;
        sensor_attr_set(dev_, SENSOR_CHAN_PRESS, SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);

        if (sensor_sample_fetch(dev_) < 0) return 0U;
        if (sensor_channel_get(dev_, SENSOR_CHAN_PRESS, &val) < 0) return 0U;
        return static_cast<uint32_t>(val.val1 * 10 + val.val2 / 100000);
    }
    bool isReady() const noexcept override { return device_is_ready(dev_); }
private:
    const struct device* dev_;
};

class UartAlarmObserver final : public IAlarmObserver {
public:
    explicit UartAlarmObserver(const struct device* uart) noexcept
        : uart_(uart) {}

    void onAlarm(AlarmType type) noexcept override {
        const char* msg = nullptr;
        switch (type) {
            case AlarmType::OCCLUSION:       msg = "ALARM:OCCLUSION\r\n";       break;
            case AlarmType::VOLUME_COMPLETE: msg = "ALARM:VOLUME_COMPLETE\r\n"; break;
            case AlarmType::RATE_ERROR:      msg = "ALARM:RATE_ERROR\r\n";      break;
            default:                         msg = "ALARM:UNKNOWN\r\n";          break;
        }
        txStr(msg);
    }
    void onAlarmCleared(AlarmType /*type*/) noexcept override { txStr("ALARM:CLEARED\r\n"); }
private:
    const struct device* uart_;
    void txStr(const char* s) noexcept { while (s && *s) uart_poll_out(uart_, *s++); }
};

class LedAlarmObserver final : public IAlarmObserver {
public:
    explicit LedAlarmObserver(const struct gpio_dt_spec led) noexcept
        : led_(led) {}
    void onAlarm(AlarmType /*type*/) noexcept override { gpio_pin_set_dt(&led_, 1); }
    void onAlarmCleared(AlarmType /*type*/) noexcept override { gpio_pin_set_dt(&led_, 0); }
private:
    struct gpio_dt_spec led_;
};

// ============================================================
// Static Storage Area — Zero Heap
// ============================================================
static constexpr float    DEFAULT_RATE_ML_HR  = 120.0f;
static constexpr float    RAMP_START_ML_HR    = 1.0f;
static constexpr float    RAMP_TARGET_ML_HR   = 120.0f;
static constexpr uint32_t RAMP_DURATION_US    = 60'000'000U;
static constexpr uint32_t TARGET_VOLUME_UL    = 500'000U; 

static VolumeTracker g_tracker;

static uint8_t buf_stepper  [sizeof(ZephyrStepperDriver)]  alignas(ZephyrStepperDriver);
static uint8_t buf_encoder  [sizeof(ZephyrEncoderDriver)]  alignas(ZephyrEncoderDriver);
static uint8_t buf_pressure [sizeof(ZephyrPressureSensor)] alignas(ZephyrPressureSensor);
static uint8_t buf_uartObs  [sizeof(UartAlarmObserver)]    alignas(UartAlarmObserver);
static uint8_t buf_ledObs   [sizeof(LedAlarmObserver)]     alignas(LedAlarmObserver);
static uint8_t buf_monitor  [sizeof(OcclusionMonitor)]     alignas(OcclusionMonitor);
static uint8_t buf_constant [sizeof(ConstantRateMode)]     alignas(ConstantRateMode);
static uint8_t buf_ramp     [sizeof(LinearRampMode)]       alignas(LinearRampMode);

static ZephyrStepperDriver* g_stepper  = nullptr;
static ZephyrEncoderDriver* g_encoder  = nullptr;
static OcclusionMonitor* g_monitor  = nullptr;
static ConstantRateMode* g_constant = nullptr;
static LinearRampMode* g_ramp     = nullptr;
static InfusionMode* g_active   = nullptr;

// ============================================================
// Core Timing Threads
// ============================================================
K_THREAD_STACK_DEFINE(infusion_stack, 1024);
static struct k_thread infusion_thread;

static void infusion_tick_fn(void*, void*, void*) {
    while (true) {
        if (g_active != nullptr) {
            g_active->run();  
            g_active->tick();
        }
        k_sleep(K_USEC(200U));
    }
}

K_THREAD_STACK_DEFINE(uart_stack, 512);
static struct k_thread uart_thread;

static void uart_rx_fn(void* arg, void*, void*) {
    const struct device* uart = static_cast<const struct device*>(arg);
    
    // Fixed-size stack buffer for heap-free command processing
    static char cmd_buf[32];
    static size_t cmd_idx = 0U;
    
    uint8_t byte = 0U;

    while (true) {
        if (uart_poll_in(uart, &byte) == 0 && g_active != nullptr) {
            // Echo back character immediately for a responsive user terminal experience
            uart_poll_out(uart, byte);

            // Handle Backspace (\b or DEL) to allow correcting typos in the serial console
            if (byte == '\b' || byte == 127U) {
                if (cmd_idx > 0U) {
                    --cmd_idx;
                    // Send terminal escape sequence to clear character visually
                    uart_poll_out(uart, ' ');
                    uart_poll_out(uart, '\b');
                }
                continue;
            }

            // Detect String Line Termination (\r or \n)
            if (byte == '\r' || byte == '\n') {
                if (cmd_idx > 0U) {
                    cmd_buf[cmd_idx] = '\0'; // Enforce strict null-termination

                    // Clear mechanical rollback offsets immediately before parsing system states
                    g_encoder->resetTicks();

                    // --------------------------------------------------------
                    // COMMAND PARSER ENGINE
                    // --------------------------------------------------------
                    
                    // 1. DYNAMIC RATE MODIFICATION: SET_RATE <val>
                    if (strncmp(cmd_buf, "SET_RATE ", 9) == 0) {
                        float parsed_rate = 0.0f;
                        if (sscanf(cmd_buf + 9, "%f", &parsed_rate) == 1) {
                            g_constant->setRate(parsed_rate);
                            // Cast the target value to an integer to satisfy strict non-promoted compilation checks
                            printk("\r\n>> Parameter Confirmed: Rate adjusted to %d mL/hr\r\n", static_cast<int32_t>(parsed_rate));                        } else {
                            printk("\r\n>> Parser Error: Invalid rate format. Usage: SET_RATE <value>\r\n");
                        }
                    }
                    // 2. SYSTEM CONTROL: START
                    else if (strncmp(cmd_buf, "START", 5) == 0) {
                        g_tracker.reset();
                        g_stepper->resetStepCount();
                        g_encoder->resetTicks();
                        g_active->start();
                        printk("\r\n>> System State: START applied safely\r\n");
                    }
                    // 3. SYSTEM CONTROL: STOP
                    else if (strncmp(cmd_buf, "STOP", 4) == 0) {
                        g_active->stop();
                        g_encoder->resetTicks();
                        printk("\r\n>> System State: STOP applied safely\r\n");
                    }
                    // 4. SYSTEM CONTROL: PAUSE
                    else if (strncmp(cmd_buf, "PAUSE", 5) == 0) {
                        g_active->pause();
                        g_encoder->resetTicks();
                        printk("\r\n>> System State: PAUSE applied safely\r\n");
                    }
                    // 5. SYSTEM CONTROL: RESUME
                    else if (strncmp(cmd_buf, "RESUME", 6) == 0) {
                        g_encoder->resetTicks();
                        g_active->resume();
                        g_encoder->resetTicks();
                        printk("\r\n>> System State: RESUME applied safely\r\n");
                    }
                    // 6. MODE CONTROL: MODE_CONSTANT
                    else if (strncmp(cmd_buf, "MODE_CONSTANT", 13) == 0) {
                        if (g_active != g_constant) {
                            g_encoder->resetTicks();
                            g_active->switchMode(g_constant);
                            g_active = g_constant;
                            g_encoder->resetTicks();
                            printk("\r\n>> Mode Changed: CONSTANT RATE ACTIVE\r\n");
                        }
                    }
                    // 7. MODE CONTROL: MODE_RAMP
                    else if (strncmp(cmd_buf, "MODE_RAMP", 9) == 0) {
                        if (g_active != g_ramp) {
                            g_encoder->resetTicks();
                            g_ramp->resetRamp();
                            g_active->switchMode(g_ramp);
                            g_active = g_ramp;
                            g_encoder->resetTicks();
                            printk("\r\n>> Mode Changed: LINEAR RAMP ACTIVE\r\n");
                        }
                    }
                    else {
                        printk("\r\n>> Parser Error: Unknown Command Signature ['%s']\r\n", cmd_buf);
                    }

                    // Flush encoder boundary to protect the upcoming high-speed tick frame
                    g_encoder->resetTicks();
                    
                    // Reset buffer index for next command string entry
                    cmd_idx = 0U;
                    printk("\r\ninfusion_pump> "); // Print line prompt
                } else {
                    // Empty newline typed, just print prompt newline
                    printk("\r\ninfusion_pump> ");
                }
                continue;
            }

            // Append incoming character if there is space remaining in the stack segment
            if (cmd_idx < (sizeof(cmd_buf) - 1U)) {
                cmd_buf[cmd_idx++] = static_cast<char>(byte);
            } else {
                // Buffer Overflow Guard: Flush line automatically if boundary is crossed
                cmd_idx = 0U;
                printk("\r\n>> Parser Error: Command string too long. Buffer flushed.\r\ninfusion_pump> ");
            }
        }
        k_yield();
    }
}

// ============================================================
// Main Initialization Engine
// ============================================================
int main(void) {
    printk("=== Infusion Pump System Online ===\n");
    const struct device* uart    = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    const struct device* lps22hb = DEVICE_DT_GET(DT_CHILD(DT_NODELABEL(i2c1), lps22hb_5c));

    __ASSERT(device_is_ready(uart),    "UART Interface Error");
    __ASSERT(device_is_ready(lps22hb), "LPS22HB Sensor Error");

    static const struct gpio_dt_spec step_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(step_pin), gpios);
    static const struct gpio_dt_spec dir_spec  = GPIO_DT_SPEC_GET(DT_NODELABEL(dir_pin), gpios);
    static const struct gpio_dt_spec en_spec   = GPIO_DT_SPEC_GET(DT_NODELABEL(en_pin), gpios);
    static const struct gpio_dt_spec led_spec  = GPIO_DT_SPEC_GET(DT_NODELABEL(alarm_led), gpios);

    gpio_pin_configure_dt(&step_spec, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_spec,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&en_spec,   GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_spec,  GPIO_OUTPUT_INACTIVE);

    g_stepper  = new (buf_stepper)  ZephyrStepperDriver{step_spec, dir_spec, en_spec};
    g_encoder  = new (buf_encoder)  ZephyrEncoderDriver{};
    auto* pres = new (buf_pressure) ZephyrPressureSensor{lps22hb};
    auto* uObs = new (buf_uartObs)  UartAlarmObserver{uart};
    auto* lObs = new (buf_ledObs)   LedAlarmObserver{led_spec};

    g_monitor  = new (buf_monitor)  OcclusionMonitor{*pres, 150U}; // Restored target safety window
    g_monitor->registerObserver(uObs);
    g_monitor->registerObserver(lObs);

    g_constant = new (buf_constant) ConstantRateMode{*g_stepper, *g_encoder, g_tracker, *g_monitor, TARGET_VOLUME_UL, DEFAULT_RATE_ML_HR};
    g_ramp     = new (buf_ramp)     LinearRampMode{*g_stepper, *g_encoder, g_tracker, *g_monitor, TARGET_VOLUME_UL, RAMP_START_ML_HR, RAMP_TARGET_ML_HR, RAMP_DURATION_US};

    g_active = g_constant;  

    k_thread_create(&infusion_thread, infusion_stack, K_THREAD_STACK_SIZEOF(infusion_stack), infusion_tick_fn, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_create(&uart_thread, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack), uart_rx_fn, const_cast<struct device*>(uart), nullptr, nullptr, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);

    while (true) {
        if (g_active != nullptr && !g_active->isRunning() && g_stepper->getStepCount() > 0 && g_stepper->getStepCount() < 100000U) {
            g_active->resume(); 
        }

        uint32_t current_pressure = pres->readPressureHPa();
        uint32_t commanded_steps  = g_stepper->getStepCount();
        uint32_t raw_timer_cnt    = TIM2->CNT; 
        uint32_t volume_tracked   = g_tracker.volumeUL();
        bool     is_running       = g_active->isRunning();
        int32_t  rate_integer     = static_cast<int32_t>(g_constant->getRate());

        // Calculate Tracking Accuracy Percentage (Ideal ratio: 3 Ticks / 2 Steps = 1.5)
        uint32_t accuracy_pct = 100U;
        if (commanded_steps > 0U) {
            // Expected ticks scaled up by 100 to maintain high precision integer math
            uint32_t expected_ticks_x100 = (commanded_steps * 150U);
            uint32_t actual_ticks_x100   = (raw_timer_cnt * 100U);

            if (actual_ticks_x100 >= expected_ticks_x100) {
                // Prevent overflowing beyond 100% if mechanical jitter adds a tiny noise edge
                accuracy_pct = 100U;
            } else {
                accuracy_pct = (actual_ticks_x100 * 100U) / expected_ticks_x100;
            }
        } else if (raw_timer_cnt > 0U) {
            // Jitter while stopped
            accuracy_pct = 0U;
        }

        printk("Status: %s | Target: %d mL/hr | Pressure: %u hPa | Steps: %u | TIM2->CNT: %u | Accuracy: %u%% | Vol: %u uL\n",
               is_running ? "RUNNING" : "STOPPED", rate_integer, current_pressure, commanded_steps, raw_timer_cnt, accuracy_pct, volume_tracked);

        k_msleep(1000);
    }
}