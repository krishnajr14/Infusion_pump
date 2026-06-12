#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// IPressureSensor
// Pure interface for LPS22HB barometric pressure sensor (I²C).
//
// Role in infusion pump:
//   Mounted on downstream tube. Normal infusion = low stable pressure.
//   Blocked tube = pressure rises sharply above occlusion threshold.
//   OcclusionMonitor polls this interface every tick.
//
// Pressure units: hPa (hectopascals)
//   Normal downstream pressure : ~1013 hPa (atmospheric baseline)
//   Occlusion threshold        : configurable, default +50 hPa above baseline
//
// Concrete implementations:
//   Production : ZephyrPressureSensor  in zephyr/main.cpp  (I²C read)
//   Test       : PressureSensorStub    in tests/stubs/
// ---------------------------------------------------------------------------
class IPressureSensor {
public:
    virtual ~IPressureSensor() = default;  // LCOV_EXCL_LINE

    // Read current pressure in hPa. Returns 0 on sensor fault.
    virtual uint32_t readPressureHPa() const noexcept = 0;

    // Returns true if sensor is responding on I²C bus.
    virtual bool isReady() const noexcept = 0;
};
