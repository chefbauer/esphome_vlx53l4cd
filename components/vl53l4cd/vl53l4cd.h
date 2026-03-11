#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l4cd {

class VL53L4CDSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_timing_budget_ms(uint8_t ms) { timing_budget_ms_ = ms; }
  void set_inter_measurement_ms(uint32_t ms) { inter_measurement_ms_ = ms; }
  void set_timeout_ms(uint16_t ms) { timeout_ms_ = ms; }

 protected:
  // Register addresses from VL53L4CD ULD API
  enum RegAddr : uint16_t {
    SOFT_RESET                           = 0x0000,
    I2C_SLAVE_DEVICE_ADDRESS             = 0x0001,
    VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
    RANGE_OFFSET_MM                      = 0x001E,
    INNER_OFFSET_MM                      = 0x0020,
    OUTER_OFFSET_MM                      = 0x0022,
    GPIO_HV_MUX_CTRL                     = 0x0030,
    GPIO_TIO_HV_STATUS                   = 0x0031,
    RANGE_CONFIG_A                       = 0x005E,
    RANGE_CONFIG_B                       = 0x0061,
    RANGE_CONFIG_SIGMA_THRESH            = 0x0064,
    MIN_COUNT_RATE_RTN_LIMIT_MCPS        = 0x0066,
    INTERMEASUREMENT_MS                  = 0x006C,
    SYSTEM_INTERRUPT_CLEAR               = 0x0086,
    SYSTEM_START                         = 0x0087,
    RESULT_RANGE_STATUS                  = 0x0089,
    RESULT_OSC_CALIBRATE_VAL             = 0x00DE,
    FIRMWARE_SYSTEM_STATUS               = 0x00E5,
    IDENTIFICATION_MODEL_ID              = 0x010F,
  };

  bool init_sensor_();
  bool set_range_timing_();
  bool data_ready_();
  void clear_interrupt_();
  void start_continuous_();
  void stop_continuous_();
  void read_and_publish_();

  uint8_t read_byte_(uint16_t reg);
  uint16_t read_word_(uint16_t reg);
  uint32_t read_dword_(uint16_t reg);
  void write_byte_(uint16_t reg, uint8_t value);
  void write_word_(uint16_t reg, uint16_t value);
  void write_dword_(uint16_t reg, uint32_t value);

  uint8_t timing_budget_ms_{50};
  uint32_t inter_measurement_ms_{0};
  uint16_t timeout_ms_{500};

  bool initialized_{false};
  bool waiting_for_data_{false};
  uint32_t update_requested_at_{0};
};

}  // namespace vl53l4cd
}  // namespace esphome
