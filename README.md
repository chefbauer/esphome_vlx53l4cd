# ESPHome VL53L4CD

> Created with Claude Sonnet 4.6 — it works!!

External ESPHome component for the **VL53L4CD** Time-of-Flight distance sensor by ST Microelectronics.

Based on the [Pololu VL53L4CD Arduino Library](https://github.com/pololu/vl53l4cd-arduino), which in turn is built on the ST ULD API (STSW-IMG026).

---

## Sensor Overview

| Property            | Value                                   |
|---------------------|-----------------------------------------|
| Interface           | I²C                                     |
| Default address     | `0x29`                                  |
| Measurement range   | 1 mm – 1300 mm                          |
| Timing budget       | 10 ms – 200 ms                          |
| Supply voltage      | 2.6 V – 3.5 V (I/O 1.8 V or 2.8 V)    |

---

## Installation

Add the external component to your ESPHome YAML configuration:

```yaml
external_components:
  - source: github://your-user/esphome_vlx53l4cd
    components: [vl53l4cd]
```

---

## Configuration

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: vl53l4cd
    name: "Distance"
    address: 0x29              # optional, default: 0x29
    timing_budget_ms: 50       # optional, default: 50 (10–200 ms)
    inter_measurement_ms: 0    # optional, default: 0 (continuous mode)
    timeout_ms: 500            # optional, default: 500 ms
    update_interval: 500ms     # optional, default: 60s
```

### Parameters

| Parameter               | Type    | Default  | Description |
|-------------------------|---------|----------|-------------|
| `timing_budget_ms`      | int     | `50`     | Measurement duration per reading in ms (10–200). Longer budget = more accurate. |
| `inter_measurement_ms`  | int     | `0`      | Pause between measurements in ms. `0` = continuous mode (no pause). Must be greater than `timing_budget_ms` if != 0. |
| `timeout_ms`            | int     | `500`    | Timeout for I²C operations and measurement wait loop in ms. |
| `update_interval`       | time    | `60s`    | Standard ESPHome update interval. |
| `address`               | hex     | `0x29`   | I²C address of the sensor. |

---

## Measurement Modes

### Continuous Mode (`inter_measurement_ms: 0`)
The sensor measures continuously without any pause between readings. ESPHome reads the latest value at the configured `update_interval`.

### Autonomous Low-Power Mode (`inter_measurement_ms > timing_budget_ms`)
The sensor measures at fixed intervals and sleeps in between. Saves power when less frequent measurements are needed.

---

## Component Architecture

```
components/vl53l4cd/
├── __init__.py        ← empty Python package marker
├── sensor.py          ← YAML schema, validation & code generation
├── vl53l4cd.h         ← C++ class declaration
└── vl53l4cd.cpp       ← full implementation
```

### Initialization Flow (`setup()`)

1. Check model ID — register `0x010F` must return `0xEBAA`
2. Wait for firmware boot — register `0x00E5` must be `0x03`
3. Configure I/O voltage — registers `0x2D`–`0x2F` (2V8 mode)
4. Write default config block — 88 bytes (reg. `0x30`–`0x87`) in a single I²C transaction
5. Run VHV calibration
6. Set timing (`RANGE_CONFIG_A` / `RANGE_CONFIG_B` / `INTERMEASUREMENT_MS`)
7. Start continuous measurement

### Measurement Flow

- **`update()`** — Sets a flag that a new value is needed (non-blocking)
- **`loop()`** — Polls `data_ready_()` (interrupt pin status via I²C), reads 15 bytes from register `0x0089` when ready, and publishes the value in meters

---

## References

- [Pololu VL53L4CD Arduino Library](https://github.com/pololu/vl53l4cd-arduino)
- [ESPHome VL53L1X Component (template)](https://github.com/ccutrer/esphome/tree/vl53l1x/esphome/components/vl53l1x)
- [ST VL53L4CD Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html)
