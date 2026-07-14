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
 * - Separate Alarm LED      (PC7)
 * - Alarm Buzzer            (PC6)
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
#include <new>       // Required for placement new

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
        dashboardOffset_ = 0U;
    }

    // ZephyrEncoderDriver::getTicks(), temporary instrumentation
    uint32_t getTicks() noexcept override {   // no longer const — must mutate remainder state
        uint32_t current_cnt = TIM2->CNT;
        uint32_t delta = 0U;
        bool isReverse = false;

        if (current_cnt >= lastFetchCnt_) {
            delta = current_cnt - lastFetchCnt_;
        } else {
            delta = lastFetchCnt_ - current_cnt;
            isReverse = true;
        }

        constexpr uint32_t MAX_PLAUSIBLE_DELTA = 1000U;
        if (delta > MAX_PLAUSIBLE_DELTA) {
            return 0U;   // real noise/reversal — correctly discarded, not a truncation loss
        }

        // Scale by 2/3 without losing the remainder: accumulate the numerator
        // across calls instead of dividing away whatever doesn't fit each time.
        scaledRemainder_ += (delta * 2U);
        uint32_t wholeTicks = scaledRemainder_ / 3U;
        scaledRemainder_   %= 3U;

        return wholeTicks;
    }

    void resetTicks() noexcept override {
        lastFetchCnt_ = TIM2->CNT;
        //scaledRemainder_ = 0U; 
    }

    void addTicks(uint32_t t) noexcept override {
        lastFetchCnt_ -= (t * 3U) / 2U;
    }

    void resetDashboardCounter() noexcept {
        dashboardOffset_ = TIM2->CNT;
    }

    uint32_t getDashboardCount() const noexcept {
        uint32_t current_cnt = TIM2->CNT;
        if (current_cnt >= dashboardOffset_) {
            return current_cnt - dashboardOffset_;
        } else {
            return (0xFFFFFFFFU - dashboardOffset_) + current_cnt + 1U;
        }
    }

private:
    mutable uint32_t lastFetchCnt_{0U}; 
    uint32_t dashboardOffset_{0U}; 
    uint32_t scaledRemainder_{0U};
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

// PROFESSIONAL ADDITION: Independent Buzzer Alarm Observer Module
class BuzzerAlarmObserver final : public IAlarmObserver {
public:
    explicit BuzzerAlarmObserver(const struct gpio_dt_spec buzzer) noexcept
        : buzzer_(buzzer) {}
    void onAlarm(AlarmType /*type*/) noexcept override { gpio_pin_set_dt(&buzzer_, 1); }
    void onAlarmCleared(AlarmType /*type*/) noexcept override { gpio_pin_set_dt(&buzzer_, 0); }
private:
    struct gpio_dt_spec buzzer_;
};

// ============================================================
// Static Storage Area — Zero Heap
// ============================================================
static constexpr float    DEFAULT_RATE_ML_HR  = 120.0f;
static constexpr float    RAMP_START_ML_HR    = 0.0f;
static constexpr float    RAMP_TARGET_ML_HR   = 120.0f;
static constexpr uint32_t RAMP_DURATION_US    = 3'600'000'000U;
static constexpr uint32_t TARGET_VOLUME_UL    = 500'000U; 

static VolumeTracker g_tracker;

static uint8_t buf_stepper  [sizeof(ZephyrStepperDriver)]  alignas(ZephyrStepperDriver);
static uint8_t buf_encoder  [sizeof(ZephyrEncoderDriver)]  alignas(ZephyrEncoderDriver);
static uint8_t buf_pressure [sizeof(ZephyrPressureSensor)] alignas(ZephyrPressureSensor);
static uint8_t buf_uartObs  [sizeof(UartAlarmObserver)]    alignas(UartAlarmObserver);
static uint8_t buf_ledObs   [sizeof(LedAlarmObserver)]     alignas(LedAlarmObserver);
static uint8_t buf_buzzerObs[sizeof(BuzzerAlarmObserver)]  alignas(BuzzerAlarmObserver); // Added buffer allocation
static uint8_t buf_monitor  [sizeof(OcclusionMonitor)]     alignas(OcclusionMonitor);
static uint8_t buf_constant [sizeof(ConstantRateMode)]     alignas(ConstantRateMode);
static uint8_t buf_ramp     [sizeof(LinearRampMode)]       alignas(LinearRampMode);

static ZephyrStepperDriver* g_stepper  = nullptr;
static ZephyrEncoderDriver* g_encoder  = nullptr;
static OcclusionMonitor* g_monitor  = nullptr;
static ConstantRateMode* g_constant = nullptr;
static LinearRampMode* g_ramp     = nullptr;
static InfusionMode* g_active   = nullptr;

static char   g_cmd_buf[32] = {0};
static size_t g_cmd_idx     = 0U;

// ============================================================
// Core Timing Threads
// ============================================================
K_THREAD_STACK_DEFINE(infusion_stack, 1024);
static struct k_thread infusion_thread;

static struct k_timer tick_timer;

static void tick_timer_handler(struct k_timer*) {
    if (g_active != nullptr) {
        g_active->tick();
    }
}

K_THREAD_STACK_DEFINE(infusion_run_stack, 1024);
static struct k_thread infusion_run_thread;

static void infusion_run_fn(void*, void*, void*) {
    while (true) {
        if (g_active != nullptr) {
            g_active->run();
        }
        k_msleep(20); 
    }
}

K_THREAD_STACK_DEFINE(uart_stack, 512);
static struct k_thread uart_thread;

static void uart_rx_fn(void* arg, void*, void*) {
    const struct device* uart = static_cast<const struct device*>(arg);
    uint8_t byte = 0U;

    while (true) {
        if (uart_poll_in(uart, &byte) == 0 && g_active != nullptr) {

            if (byte == '\r' || byte == '\n') {
                if (g_cmd_idx > 0U) {
                    g_cmd_buf[g_cmd_idx] = '\0';

                    if (strncmp(g_cmd_buf, "SET_RATE ", 9) == 0) {
                        int32_t parsed_rate_int = 0;
                        if (sscanf(g_cmd_buf + 9, "%d", &parsed_rate_int) == 1) {
                            if (parsed_rate_int >= 1 && parsed_rate_int <= 500) {
                                g_constant->setRate(static_cast<float>(parsed_rate_int));
                                printk("\r\n>> Parameter Confirmed: Rate adjusted to %d mL/hr\r\n", parsed_rate_int);
                            } else {
                                printk("\r\n>> Safety Guard: Rate must be between 1 and 500 mL/hr\r\n");
                            }
                        } else {
                            printk("\r\n>> Parser Error: Invalid rate format. Usage: SET_RATE <integer>\r\n");
                        }
                    }
                    else if (strncmp(g_cmd_buf, "MODE_CONSTANT", 13) == 0) {
                        if (g_active != g_constant) {
                            g_active->switchMode(g_constant);
                            g_active = g_constant;
                            printk("\r\n>> Mode Changed: CONSTANT RATE ACTIVE\r\n");
                        }
                    }
                    else if (strncmp(g_cmd_buf, "MODE_RAMP", 9) == 0) {
                        if (g_active != g_ramp) {
                            g_ramp->resetRamp();
                            g_active->switchMode(g_ramp);
                            g_active = g_ramp;
                            printk("\r\n>> Mode Changed: LINEAR RAMP ACTIVE\r\n");
                        }
                    }
                    else if (strncmp(g_cmd_buf, "START", 5) == 0) {
                        if (g_active == g_ramp) { g_ramp->resetRamp(); }
                        g_tracker.reset();
                        g_stepper->resetStepCount();
                        g_encoder->resetTicks();          // the ONE legitimate reset in this whole function
                        g_encoder->resetDashboardCounter();
                        g_active->start();
                        printk("\r\n>> System State: START applied safely\r\n");
                    }
                    else if (strncmp(g_cmd_buf, "STOP", 4) == 0) {
                        g_active->stop();
                        printk("\r\n>> System State: STOP applied safely\r\n");
                    }
                    else if (strncmp(g_cmd_buf, "PAUSE", 5) == 0) {
                        g_active->pause();
                        printk("\r\n>> System State: PAUSE applied safely\r\n");
                    }
                    else if (strncmp(g_cmd_buf, "RESUME", 6) == 0) {
                        g_active->resume();
                        printk("\r\n>> System State: RESUME applied safely\r\n");
                    }
                    else if (strncmp(g_cmd_buf, "ALARM_CLEAR", 11) == 0) {
                        g_monitor->poll();   // force a fresh read right now, don't wait for the next run() cycle
                        if (g_monitor->isOccluded()) {
                            printk("\r\n>> Alarm Clear Rejected: Occlusion still present\r\n");
                        } else {
                            g_active->resume();
                            printk("\r\n>> Alarm Cleared: System Resumed\r\n");
                        }
                    }
                    else {
                        printk("\r\n>> Parser Error: Unknown Command Signature ['%s']\r\n", g_cmd_buf);
                    }

                    g_cmd_idx = 0U;
                    memset(g_cmd_buf, 0, sizeof(g_cmd_buf));
                    printk("\r\ninfusion_pump> ");
                }
                continue;
            }

            if (byte >= 32U && byte <= 126U) {
                uart_poll_out(uart, byte);
            }

            if (byte == '\b' || byte == 127U) {
                if (g_cmd_idx > 0U) {
                    --g_cmd_idx;
                    g_cmd_buf[g_cmd_idx] = '\0';
                    uart_poll_out(uart, '\b');
                    uart_poll_out(uart, ' ');
                    uart_poll_out(uart, '\b');
                }
                continue;
            }

            if (byte >= 32U && byte <= 126U) {
                if (g_cmd_idx < (sizeof(g_cmd_buf) - 1U)) {
                    g_cmd_buf[g_cmd_idx++] = static_cast<char>(byte);
                } else {
                    g_cmd_idx = 0U;
                    memset(g_cmd_buf, 0, sizeof(g_cmd_buf));
                    printk("\r\n>> Parser Error: Buffer Overflow. Flushed.\r\ninfusion_pump> ");
                }
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

    static const struct gpio_dt_spec step_spec   = GPIO_DT_SPEC_GET(DT_NODELABEL(step_pin), gpios);
    static const struct gpio_dt_spec dir_spec    = GPIO_DT_SPEC_GET(DT_NODELABEL(dir_pin), gpios);
    static const struct gpio_dt_spec en_spec     = GPIO_DT_SPEC_GET(DT_NODELABEL(en_pin), gpios);
    static const struct gpio_dt_spec led_spec    = GPIO_DT_SPEC_GET(DT_NODELABEL(alarm_led), gpios);
    static const struct gpio_dt_spec buzzer_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(alarm_buzzer), gpios); // Added Buzzer spec

    gpio_pin_configure_dt(&step_spec,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_spec,    GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&en_spec,     GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_spec,    GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&buzzer_spec, GPIO_OUTPUT_INACTIVE); // Configured Buzzer pin

    g_stepper  = new (buf_stepper)  ZephyrStepperDriver{step_spec, dir_spec, en_spec};
    g_encoder  = new (buf_encoder)  ZephyrEncoderDriver{};
    auto* pres = new (buf_pressure) ZephyrPressureSensor{lps22hb};
    auto* uObs = new (buf_uartObs)  UartAlarmObserver{uart};
    auto* lObs = new (buf_ledObs)   LedAlarmObserver{led_spec};
    auto* bObs = new (buf_buzzerObs) BuzzerAlarmObserver{buzzer_spec}; // Instantiated Buzzer Observer

    g_monitor  = new (buf_monitor)  OcclusionMonitor{*pres, 50U}; 
    g_monitor->registerObserver(uObs);
    g_monitor->registerObserver(lObs);
    g_monitor->registerObserver(bObs); // Registered Buzzer to the notification list

    g_constant = new (buf_constant) ConstantRateMode{*g_stepper, *g_encoder, g_tracker, *g_monitor, TARGET_VOLUME_UL, DEFAULT_RATE_ML_HR};
    g_ramp     = new (buf_ramp)     LinearRampMode{*g_stepper, *g_encoder, g_tracker, *g_monitor, TARGET_VOLUME_UL, RAMP_START_ML_HR, RAMP_TARGET_ML_HR, RAMP_DURATION_US};

    g_active = g_constant;  

    // Core Motor Real-Time Tick Interrupter (Priority 3)
    k_timer_init(&tick_timer, tick_timer_handler, nullptr);
    k_timer_start(&tick_timer, K_USEC(200), K_USEC(200));

    // Business Logic Engine (Priority 5)
    k_thread_create(&infusion_run_thread, infusion_run_stack, K_THREAD_STACK_SIZEOF(infusion_run_stack), 
                    infusion_run_fn, nullptr, nullptr, nullptr, 
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

    // Shell CLI Input Monitor (Priority 7)
    k_thread_create(&uart_thread, uart_stack, K_THREAD_STACK_SIZEOF(uart_stack), 
                    uart_rx_fn, const_cast<struct device*>(uart), nullptr, nullptr, 
                    K_PRIO_PREEMPT(7), 0, K_NO_WAIT);

    printk("\033[2J\033[H");
    printk("=============================================================\r\n");
    printk("       INFUSION PUMP EMBEDDED SYSTEM REAL-TIME DASHBOARD      \r\n");
    printk("=============================================================\r\n\n");

    while (true) {
        uint32_t current_pressure = g_monitor->lastPressureHPa();
        uint32_t commanded_steps  = g_stepper->getStepCount();
        uint32_t volume_tracked   = g_tracker.volumeUL();
        bool     is_running       = g_active->isRunning();
        int32_t  rate_integer     = static_cast<int32_t>(g_constant->getRate());

        uint32_t relative_encoder_cnt = (g_encoder->getDashboardCount() * 2U) / 3U; 

        uint32_t error_pct    = 0U;
        uint32_t missed_steps = 0U;

        if (commanded_steps > 0U) {
            if (commanded_steps > relative_encoder_cnt) {
                missed_steps = commanded_steps - relative_encoder_cnt;
                error_pct    = (missed_steps * 100U) / commanded_steps;
            } else if (relative_encoder_cnt > commanded_steps) {
                missed_steps = relative_encoder_cnt - commanded_steps;
                error_pct    = (missed_steps * 100U) / commanded_steps;
            }
        } else if (relative_encoder_cnt > 0U) {
            missed_steps = relative_encoder_cnt;
            error_pct    = 100U;
        }

        printk("\r\n=============================================================\r\n");
        printk("       INFUSION PUMP EMBEDDED SYSTEM REAL-TIME DASHBOARD      \r\n");
        printk("=============================================================\r\n");
        printk("[System Status] : %s\r\n", is_running ? "RUNNING" : "STOPPED");
        printk("[Target Rate]   : %d mL/hr\r\n", rate_integer);
        printk("[Coil Pressure] : %u hPa\r\n", current_pressure);
        printk("[Stepper Steps] : %u\r\n", commanded_steps);
        printk("[TIM2 Encoder]  : %u\r\n", relative_encoder_cnt);
        printk("[Missed Steps]  : %u (Err: %u%%)\r\n", missed_steps, error_pct);
        printk("[Total Volume]  : %u uL\r\n", volume_tracked);
        printk("-------------------------------------------------------------\r\n");
        
        printk("infusion_pump> %s", g_cmd_buf);

        k_msleep(1000);
    }
}