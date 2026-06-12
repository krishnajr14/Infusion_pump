#include <gtest/gtest.h>
#include "infusion/OcclusionMonitor.hpp"
#include "PressureSensorStub.hpp"
#include "AlarmObserverStub.hpp"

static PressureSensorStub g_sensor;
static AlarmObserverStub  g_obs1;
static AlarmObserverStub  g_obs2;
static AlarmObserverStub  g_obs3;
static AlarmObserverStub  g_obs4;
static AlarmObserverStub  g_obs5;  // overflow test

class OcclusionMonitorTest : public ::testing::Test {
protected:
    OcclusionMonitor* mon = nullptr;
    uint8_t monBuf[sizeof(OcclusionMonitor)] alignas(OcclusionMonitor){};

    void SetUp() override {
        g_sensor.reset();
        g_obs1.reset(); g_obs2.reset();
        g_obs3.reset(); g_obs4.reset(); g_obs5.reset();
        mon = new (monBuf) OcclusionMonitor{g_sensor, 50U};
    }
};

// ── Observer registration ─────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, Register_Success) {
    EXPECT_TRUE(mon->registerObserver(&g_obs1));
    EXPECT_EQ(mon->observerCount(), 1U);
}

TEST_F(OcclusionMonitorTest, Register_Null_Fails) {
    EXPECT_FALSE(mon->registerObserver(nullptr));
    EXPECT_EQ(mon->observerCount(), 0U);
}

TEST_F(OcclusionMonitorTest, Register_TableFull_Fails) {
    mon->registerObserver(&g_obs1);
    mon->registerObserver(&g_obs2);
    mon->registerObserver(&g_obs3);
    mon->registerObserver(&g_obs4);
    EXPECT_FALSE(mon->registerObserver(&g_obs5));
    EXPECT_EQ(mon->observerCount(), 4U);
}

// ── Baseline capture ──────────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, FirstPoll_CapturesBaseline) {
    g_sensor.setPressure(1013U);
    mon->poll();
    EXPECT_EQ(mon->baselineHPa(), 1013U);
    EXPECT_FALSE(mon->isOccluded());
}

TEST_F(OcclusionMonitorTest, SensorNotReady_NoPoll) {
    g_sensor.setReady(false);
    mon->poll();
    EXPECT_EQ(mon->baselineHPa(), 0U);   // baseline never captured
}

// ── Occlusion detection ───────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, PressureBelowThreshold_NoAlarm) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();   // capture baseline = 1013
    g_sensor.setPressure(1040U);   // +27 hPa < threshold(50)
    mon->poll();
    EXPECT_FALSE(mon->isOccluded());
    EXPECT_EQ(g_obs1.raiseCount(), 0U);
}

TEST_F(OcclusionMonitorTest, PressureAtThreshold_RaisesAlarm) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();   // baseline = 1013
    g_sensor.setPressure(1063U);   // +50 hPa = threshold exactly
    mon->poll();
    EXPECT_TRUE(mon->isOccluded());
    EXPECT_EQ(g_obs1.raiseCount(), 1U);
    EXPECT_EQ(g_obs1.lastRaised(), AlarmType::OCCLUSION);
}

TEST_F(OcclusionMonitorTest, PressureAboveThreshold_RaisesAlarm) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();
    g_sensor.setPressure(1100U);   // +87 hPa >> threshold
    mon->poll();
    EXPECT_TRUE(mon->isOccluded());
    EXPECT_EQ(g_obs1.raiseCount(), 1U);
}

TEST_F(OcclusionMonitorTest, AlarmRaised_Idempotent_NoDoubleNotify) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();
    g_sensor.setPressure(1100U);
    mon->poll();   // raises alarm
    mon->poll();   // still occluded — no second notify
    EXPECT_EQ(g_obs1.raiseCount(), 1U);
}

TEST_F(OcclusionMonitorTest, PressureDrops_ClearsAlarm) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();
    g_sensor.setPressure(1100U);
    mon->poll();   // occlusion raised
    g_sensor.setPressure(1020U);   // drops below threshold
    mon->poll();   // occlusion cleared
    EXPECT_FALSE(mon->isOccluded());
    EXPECT_EQ(g_obs1.clearCount(), 1U);
    EXPECT_EQ(g_obs1.lastCleared(), AlarmType::OCCLUSION);
}

TEST_F(OcclusionMonitorTest, TwoObservers_BothNotified) {
    mon->registerObserver(&g_obs1);
    mon->registerObserver(&g_obs2);
    g_sensor.setPressure(1013U);
    mon->poll();
    g_sensor.setPressure(1100U);
    mon->poll();
    EXPECT_EQ(g_obs1.raiseCount(), 1U);
    EXPECT_EQ(g_obs2.raiseCount(), 1U);
}

// ── resetBaseline ─────────────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, ResetBaseline_ClearsOcclusionState) {
    mon->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    mon->poll();
    g_sensor.setPressure(1100U);
    mon->poll();   // occluded
    mon->resetBaseline();
    EXPECT_FALSE(mon->isOccluded());
    // After reset, next poll recaptures baseline
    g_sensor.setPressure(1100U);
    mon->poll();   // new baseline = 1100
    EXPECT_EQ(mon->baselineHPa(), 1100U);
    EXPECT_FALSE(mon->isOccluded());   // pressure == baseline, not over threshold
}

// ── lastPressureHPa ───────────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, LastPressure_UpdatedOnEachPoll) {
    g_sensor.setPressure(1013U);
    mon->poll();
    EXPECT_EQ(mon->lastPressureHPa(), 1013U);
    g_sensor.setPressure(1050U);
    mon->poll();
    EXPECT_EQ(mon->lastPressureHPa(), 1050U);
}

// ── Default threshold ─────────────────────────────────────────────────────
TEST_F(OcclusionMonitorTest, DefaultThreshold_50HPa) {
    // Use default constructor threshold
    uint8_t buf[sizeof(OcclusionMonitor)] alignas(OcclusionMonitor);
    OcclusionMonitor* m = new (buf) OcclusionMonitor{g_sensor};
    m->registerObserver(&g_obs1);
    g_sensor.setPressure(1013U);
    m->poll();
    g_sensor.setPressure(1062U);   // +49 hPa — just below default threshold
    m->poll();
    EXPECT_FALSE(m->isOccluded());
    g_sensor.setPressure(1063U);   // +50 hPa — at threshold
    m->poll();
    EXPECT_TRUE(m->isOccluded());
}
