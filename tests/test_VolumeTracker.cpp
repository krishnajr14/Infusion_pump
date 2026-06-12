#include <gtest/gtest.h>
#include "infusion/VolumeTracker.hpp"

TEST(VolumeTracker, Initial_AllZero) {
    VolumeTracker vt;
    EXPECT_EQ(vt.volumeUL(),  0U);
    EXPECT_EQ(vt.tickCount(), 0U);
}

TEST(VolumeTracker, OneTick_OneUL) {
    VolumeTracker vt;
    vt.addTicks(1U);
    EXPECT_EQ(vt.volumeUL(),  1U);
    EXPECT_EQ(vt.tickCount(), 1U);
}

TEST(VolumeTracker, BulkTicks_CorrectVolume) {
    VolumeTracker vt;
    vt.addTicks(500U);
    EXPECT_EQ(vt.volumeUL(),  500U);
    EXPECT_EQ(vt.tickCount(), 500U);
}

TEST(VolumeTracker, MultipleCalls_Cumulative) {
    VolumeTracker vt;
    vt.addTicks(100U);
    vt.addTicks(200U);
    vt.addTicks(300U);
    EXPECT_EQ(vt.volumeUL(),  600U);
    EXPECT_EQ(vt.tickCount(), 600U);
}

TEST(VolumeTracker, Reset_ClearsAll) {
    VolumeTracker vt;
    vt.addTicks(1000U);
    vt.reset();
    EXPECT_EQ(vt.volumeUL(),  0U);
    EXPECT_EQ(vt.tickCount(), 0U);
}

TEST(VolumeTracker, ZeroTicks_NoChange) {
    VolumeTracker vt;
    vt.addTicks(100U);
    const uint32_t before = vt.volumeUL();
    vt.addTicks(0U);
    EXPECT_EQ(vt.volumeUL(), before);
}

TEST(VolumeTracker, LargeAccumulation_10mL) {
    VolumeTracker vt;
    vt.addTicks(10000U);   // 10000 µL = 10 mL
    EXPECT_EQ(vt.volumeUL(), 10000U);
}

TEST(VolumeTracker, ResetThenAccumulate) {
    VolumeTracker vt;
    vt.addTicks(500U);
    vt.reset();
    vt.addTicks(250U);
    EXPECT_EQ(vt.volumeUL(),  250U);
    EXPECT_EQ(vt.tickCount(), 250U);
}

TEST(VolumeTracker, MaxRate_500mLhr_60min) {
    VolumeTracker vt;
    // 500 mL/hr × 60 min = 500,000 µL
    vt.addTicks(500'000U);
    EXPECT_EQ(vt.volumeUL(), 500'000U);
}
