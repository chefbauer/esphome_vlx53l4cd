# ESPHome VL53L4CD

Externe ESPHome-Komponente für den **VL53L4CD** Time-of-Flight Distanzsensor von ST Microelectronics.

Basiert auf der [Pololu VL53L4CD Arduino-Bibliothek](https://github.com/pololu/vl53l4cd-arduino), die ihrerseits auf dem ST ULD API (STSW-IMG026) aufbaut.

---

## Sensor-Überblick

| Eigenschaft         | Wert                        |
|---------------------|-----------------------------|
| Schnittstelle       | I²C                         |
| Standard-Adresse    | `0x29`                      |
| Messbereich         | 1 mm – 1300 mm              |
| Timing Budget       | 10 ms – 200 ms              |
| Versorgungsspannung | 2.6 V – 3.5 V (I/O 1.8 V oder 2.8 V) |

---

## Installation

Externe Komponente in der ESPHome-YAML-Konfiguration einbinden:

```yaml
external_components:
  - source: github://dein-user/esphome_vlx53l4cd
    components: [vl53l4cd]
```

---

## Konfiguration

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: vl53l4cd
    name: "Distanz"
    address: 0x29              # optional, Standard: 0x29
    timing_budget_ms: 50       # optional, Standard: 50 (10–200 ms)
    inter_measurement_ms: 0    # optional, Standard: 0 (Continuous-Mode)
    timeout_ms: 500            # optional, Standard: 500 ms
    update_interval: 500ms     # optional, Standard: 60s
```

### Parameter

| Parameter               | Typ     | Standard | Beschreibung |
|-------------------------|---------|----------|--------------|
| `timing_budget_ms`      | int     | `50`     | Messdauer pro Messung in ms (10–200). Längeres Budget = genauere Messung. |
| `inter_measurement_ms`  | int     | `0`      | Pause zwischen Messungen in ms. `0` = Continuous-Mode (keine Pause). Muss größer als `timing_budget_ms` sein wenn != 0. |
| `timeout_ms`            | int     | `500`    | Timeout für I²C-Operationen und Messwarte-Schleife in ms. |
| `update_interval`       | Zeit    | `60s`    | ESPHome-Standard-Update-Intervall. |
| `address`               | hex     | `0x29`   | I²C-Adresse des Sensors. |

---

## Messmodi

### Continuous-Mode (`inter_measurement_ms: 0`)
Der Sensor misst kontinuierlich ohne Pause zwischen den Messungen. ESPHome liest den neuesten Wert im konfigurierten `update_interval`.

### Autonomer Low-Power-Mode (`inter_measurement_ms > timing_budget_ms`)
Der Sensor misst in festgelegten Abständen und schläft dazwischen. Spart Strom bei weniger häufigen Messungen.

---

## Architektur der Komponente

```
components/vl53l4cd/
├── __init__.py        ← leerer Python-Package-Marker
├── sensor.py          ← YAML-Schema, Validierung & Code-Generierung
├── vl53l4cd.h         ← C++ Klassendeklaration
└── vl53l4cd.cpp       ← Vollständige Implementierung
```

### Initialisierungsablauf (`setup()`)

1. Model-ID prüfen — Register `0x010F` muss `0xEBAA` liefern
2. Firmware-Boot abwarten — Register `0x00E5` muss `0x03` sein
3. I/O-Spannung konfigurieren — Register `0x2D`–`0x2F` (2V8-Mode)
4. Default-Konfigurationsblock schreiben — 88 Bytes (Reg. `0x30`–`0x87`) in einer I²C-Transaktion
5. VHV-Kalibrierung durchführen
6. Timing setzen (`RANGE_CONFIG_A` / `RANGE_CONFIG_B` / `INTERMEASUREMENT_MS`)
7. Continuous-Messung starten

### Mess-Ablauf

- **`update()`** — Setzt ein Flag, dass ein neuer Messwert benötigt wird (nicht-blockierend)
- **`loop()`** — Prüft `data_ready_()` (Interrupt-Pin-Status via I²C), liest bei Bereitschaft 15 Bytes ab Register `0x0089` und publiziert den Wert in Metern

---

## Quellen

- [Pololu VL53L4CD Arduino Library](https://github.com/pololu/vl53l4cd-arduino)
- [ESPHome VL53L1X Komponente (Template)](https://github.com/ccutrer/esphome/tree/vl53l1x/esphome/components/vl53l1x)
- [ST VL53L4CD Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html)
