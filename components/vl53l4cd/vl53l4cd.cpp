// ESPHome external component for the VL53L4CD time-of-flight distance sensor.
// Based on the Pololu VL53L4CD Arduino library (https://github.com/pololu/vl53l4cd-arduino)
// which itself is based on ST's VL53L4CD Ultra Lite Driver (ULD) API (STSW-IMG026).

#include "vl53l4cd.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vl53l4cd {

static const char *const TAG = "vl53l4cd";

// Range status lookup table from ULD API (index = raw status, value = translated status)
// 0 = valid, 255 = invalid/unknown
static const uint8_t STATUS_RTN[24] = {255, 255, 255, 5, 2, 4, 1, 7, 3,
                                        0,   255, 255, 9, 13, 255, 255, 255, 255,
                                        10,  6,   255, 255, 11, 12};

// Default register configuration from VL53L4CD ULD API for registers 0x30–0x87.
// Written in one block after manually setting 0x2D–0x2F.
static const uint8_t VL53L4CD_DEFAULT_CONFIG[] = {
    0x11, /* 0x30: active-low interrupt polarity + bits 3:0 = 0x1 */
    0x02, /* 0x31: interrupt on data ready */
    0x00, /* 0x32 */
    0x02, /* 0x33 */
    0x08, /* 0x34 */
    0x00, /* 0x35 */
    0x08, /* 0x36 */
    0x10, /* 0x37 */
    0x01, /* 0x38 */
    0x01, /* 0x39 */
    0x00, /* 0x3a */
    0x00, /* 0x3b */
    0x00, /* 0x3c */
    0x00, /* 0x3d */
    0xff, /* 0x3e */
    0x00, /* 0x3f */
    0x0F, /* 0x40 */
    0x00, /* 0x41 */
    0x00, /* 0x42 */
    0x00, /* 0x43 */
    0x00, /* 0x44 */
    0x00, /* 0x45 */
    0x20, /* 0x46: new sample ready interrupt mode */
    0x0b, /* 0x47 */
    0x00, /* 0x48 */
    0x00, /* 0x49 */
    0x02, /* 0x4a */
    0x14, /* 0x4b */
    0x21, /* 0x4c */
    0x00, /* 0x4d */
    0x00, /* 0x4e */
    0x05, /* 0x4f */
    0x00, /* 0x50 */
    0x00, /* 0x51 */
    0x00, /* 0x52 */
    0x00, /* 0x53 */
    0xc8, /* 0x54 */
    0x00, /* 0x55 */
    0x00, /* 0x56 */
    0x38, /* 0x57 */
    0xff, /* 0x58 */
    0x01, /* 0x59 */
    0x00, /* 0x5a */
    0x08, /* 0x5b */
    0x00, /* 0x5c */
    0x00, /* 0x5d */
    0x01, /* 0x5e: RANGE_CONFIG_A high byte (overwritten by set_range_timing_) */
    0xcc, /* 0x5f: RANGE_CONFIG_A low byte */
    0x07, /* 0x60 */
    0x01, /* 0x61: RANGE_CONFIG_B high byte (overwritten by set_range_timing_) */
    0xf1, /* 0x62: RANGE_CONFIG_B low byte */
    0x05, /* 0x63 */
    0x00, /* 0x64: sigma threshold MSB (default 90 mm) */
    0xa0, /* 0x65: sigma threshold LSB */
    0x00, /* 0x66: min count rate MSB */
    0x80, /* 0x67: min count rate LSB */
    0x08, /* 0x68 */
    0x38, /* 0x69 */
    0x00, /* 0x6a */
    0x00, /* 0x6b */
    0x00, /* 0x6c: inter-measurement period (overwritten by set_range_timing_) */
    0x00, /* 0x6d */
    0x0f, /* 0x6e */
    0x89, /* 0x6f */
    0x00, /* 0x70 */
    0x00, /* 0x71 */
    0x00, /* 0x72: distance threshold high MSB */
    0x00, /* 0x73 */
    0x00, /* 0x74: distance threshold low MSB */
    0x00, /* 0x75 */
    0x00, /* 0x76 */
    0x01, /* 0x77 */
    0x07, /* 0x78 */
    0x05, /* 0x79 */
    0x06, /* 0x7a */
    0x06, /* 0x7b */
    0x00, /* 0x7c */
    0x00, /* 0x7d */
    0x02, /* 0x7e */
    0xc7, /* 0x7f */
    0xff, /* 0x80 */
    0x9B, /* 0x81 */
    0x00, /* 0x82 */
    0x00, /* 0x83 */
    0x00, /* 0x84 */
    0x01, /* 0x85 */
    0x00, /* 0x86: SYSTEM_INTERRUPT_CLEAR */
    0x00, /* 0x87: SYSTEM_START (stopped; VHV start written separately) */
};

// ─── setup ───────────────────────────────────────────────────────────────────

void VL53L4CDSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VL53L4CD '%s'...", this->name_.c_str());

  if (!this->init_sensor_()) {
    this->mark_failed();
    return;
  }

  initialized_ = true;
  ESP_LOGD(TAG, "'%s' setup complete", this->name_.c_str());
}

bool VL53L4CDSensor::init_sensor_() {
  // Verify model ID (datasheet: 0xEBAA at register 0x010F)
  uint16_t model_id = read_word_(IDENTIFICATION_MODEL_ID);
  if (model_id != 0xEBAA) {
    ESP_LOGE(TAG, "'%s' - wrong model ID 0x%04X (expected 0xEBAA)", this->name_.c_str(), model_id);
    return false;
  }

  // Wait for firmware boot (status = 0x03)
  uint32_t start = millis();
  while (read_byte_(FIRMWARE_SYSTEM_STATUS) != 0x03) {
    if (millis() - start > timeout_ms_) {
      ESP_LOGE(TAG, "'%s' - timeout waiting for firmware boot", this->name_.c_str());
      return false;
    }
    App.feed_wdt();
    delay(1);
  }

  // Write I/O voltage and fast-mode-plus config for registers 0x2D–0x2F
  write_byte_(0x2D, 0x00);  // fast_mode_plus = false
  write_byte_(0x2E, 0x01);  // io_2v8 = true (pull-up to AVDD / 2V8)
  write_byte_(0x2F, 0x01);  // io_2v8 = true

  // Write default config block 0x30–0x87 in one I2C transaction
  this->write_register16(0x0030, VL53L4CD_DEFAULT_CONFIG, sizeof(VL53L4CD_DEFAULT_CONFIG));

  // Trigger VHV calibration
  write_byte_(SYSTEM_START, 0x40);
  start = millis();
  while (!data_ready_()) {
    if (millis() - start > timeout_ms_) {
      ESP_LOGE(TAG, "'%s' - timeout waiting for VHV calibration", this->name_.c_str());
      return false;
    }
    App.feed_wdt();
    delay(1);
  }

  clear_interrupt_();
  stop_continuous_();

  // Finish VHV setup
  write_byte_(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  write_byte_(0x000B, 0x00);
  write_word_(0x0024, 0x0500);

  // Apply user-configured timing
  if (!set_range_timing_()) {
    ESP_LOGE(TAG, "'%s' - failed to set range timing", this->name_.c_str());
    return false;
  }

  // Start continuous measurements
  start_continuous_();
  return true;
}

// ─── timing ──────────────────────────────────────────────────────────────────

bool VL53L4CDSensor::set_range_timing_() {
  if (timing_budget_ms_ < 10 || timing_budget_ms_ > 200) {
    ESP_LOGE(TAG, "timing_budget_ms must be 10–200 ms (got %u)", timing_budget_ms_);
    return false;
  }
  if (inter_measurement_ms_ != 0 && inter_measurement_ms_ <= timing_budget_ms_) {
    ESP_LOGE(TAG, "inter_measurement_ms must be 0 or > timing_budget_ms");
    return false;
  }

  uint16_t osc_frequency = read_word_(0x0006);
  if (osc_frequency == 0) {
    ESP_LOGE(TAG, "'%s' - osc_frequency is 0", this->name_.c_str());
    return false;
  }

  uint32_t timing_budget_us = (uint32_t) timing_budget_ms_ * 1000;
  uint32_t macro_period_us = ((uint32_t) 2304 * (0x40000000 / osc_frequency)) >> 6;

  if (inter_measurement_ms_ == 0) {
    // Continuous mode
    write_dword_(INTERMEASUREMENT_MS, 0);
    timing_budget_us -= 2500;
  } else {
    // Autonomous low-power mode
    uint16_t clock_pll = read_word_(RESULT_OSC_CALIBRATE_VAL) & 0x3FF;
    uint32_t inter_meas = (uint32_t)(1.055f * inter_measurement_ms_ * clock_pll);
    write_dword_(INTERMEASUREMENT_MS, inter_meas);
    timing_budget_us -= 4300;
    timing_budget_us /= 2;
  }

  timing_budget_us <<= 12;

  // Encode RANGE_CONFIG_A
  uint32_t tmp = (macro_period_us * 16) >> 6;
  uint32_t ls = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
  uint8_t ms = 0;
  while (ls > 0xFF) {
    ls >>= 1;
    ms++;
  }
  write_word_(RANGE_CONFIG_A, ((uint16_t) ms << 8) | (uint8_t) ls);

  // Encode RANGE_CONFIG_B
  tmp = (macro_period_us * 12) >> 6;
  ls = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
  ms = 0;
  while (ls > 0xFF) {
    ls >>= 1;
    ms++;
  }
  write_word_(RANGE_CONFIG_B, ((uint16_t) ms << 8) | (uint8_t) ls);

  return true;
}

// ─── continuous control ───────────────────────────────────────────────────────

void VL53L4CDSensor::start_continuous_() {
  if (read_dword_(INTERMEASUREMENT_MS) == 0) {
    write_byte_(SYSTEM_START, 0x21);  // continuous mode
  } else {
    write_byte_(SYSTEM_START, 0x40);  // autonomous low-power mode
  }
}

void VL53L4CDSensor::stop_continuous_() { write_byte_(SYSTEM_START, 0x80); }

// ─── update / loop ────────────────────────────────────────────────────────────

void VL53L4CDSensor::update() {
  if (!initialized_) return;
  waiting_for_data_ = true;
  update_requested_at_ = millis();
}

void VL53L4CDSensor::loop() {
  if (!initialized_ || !waiting_for_data_) return;

  if (millis() - update_requested_at_ > timeout_ms_) {
    ESP_LOGW(TAG, "'%s' - timeout waiting for measurement", this->name_.c_str());
    this->publish_state(NAN);
    waiting_for_data_ = false;
    return;
  }

  if (!data_ready_()) return;

  read_and_publish_();
  waiting_for_data_ = false;
}

// ─── measurement reading ──────────────────────────────────────────────────────

bool VL53L4CDSensor::data_ready_() {
  // Interrupt is active low: bit 0 of GPIO_TIO_HV_STATUS = 0 means data ready
  return (read_byte_(GPIO_TIO_HV_STATUS) & 0x01) == 0;
}

void VL53L4CDSensor::clear_interrupt_() { write_byte_(SYSTEM_INTERRUPT_CLEAR, 0x01); }

void VL53L4CDSensor::read_and_publish_() {
  clear_interrupt_();

  // Block-read 15 bytes from 0x0089 (RESULT_RANGE_STATUS) through 0x0097 (RESULT_DISTANCE low)
  uint8_t buf[15];
  this->read_register16(RESULT_RANGE_STATUS, buf, sizeof(buf));

  // Translate raw status to ULD status code
  uint8_t raw_status = buf[0] & 0x1F;
  uint8_t status = (raw_status < 24) ? STATUS_RTN[raw_status] : 255;

  if (status != 0) {
    ESP_LOGD(TAG, "'%s' - invalid range status %u (raw %u)", this->name_.c_str(), status, raw_status);
    this->publish_state(NAN);
    return;
  }

  uint16_t range_mm = ((uint16_t) buf[13] << 8) | buf[14];
  float range_m = range_mm / 1000.0f;

  ESP_LOGD(TAG, "'%s' - distance %.3f m", this->name_.c_str(), range_m);
  this->publish_state(range_m);
}

// ─── dump config ──────────────────────────────────────────────────────────────

void VL53L4CDSensor::dump_config() {
  LOG_SENSOR("", "VL53L4CD", this);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Timing Budget: %u ms", timing_budget_ms_);
  if (inter_measurement_ms_ == 0) {
    ESP_LOGCONFIG(TAG, "  Mode: Continuous");
  } else {
    ESP_LOGCONFIG(TAG, "  Inter-Measurement: %u ms (autonomous LP mode)", inter_measurement_ms_);
  }
  ESP_LOGCONFIG(TAG, "  Timeout: %u ms", timeout_ms_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup failed!");
  }
}

// ─── low-level I2C helpers ────────────────────────────────────────────────────

uint8_t VL53L4CDSensor::read_byte_(uint16_t reg) {
  uint8_t val = 0;
  this->read_register16(reg, &val, 1);
  return val;
}

uint16_t VL53L4CDSensor::read_word_(uint16_t reg) {
  uint8_t buf[2] = {};
  this->read_register16(reg, buf, 2);
  return ((uint16_t) buf[0] << 8) | buf[1];
}

uint32_t VL53L4CDSensor::read_dword_(uint16_t reg) {
  uint8_t buf[4] = {};
  this->read_register16(reg, buf, 4);
  return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint16_t) buf[2] << 8) | buf[3];
}

void VL53L4CDSensor::write_byte_(uint16_t reg, uint8_t value) {
  this->write_register16(reg, &value, 1);
}

void VL53L4CDSensor::write_word_(uint16_t reg, uint16_t value) {
  uint8_t buf[2] = {(uint8_t)(value >> 8), (uint8_t) value};
  this->write_register16(reg, buf, 2);
}

void VL53L4CDSensor::write_dword_(uint16_t reg, uint32_t value) {
  uint8_t buf[4] = {
      (uint8_t)(value >> 24),
      (uint8_t)(value >> 16),
      (uint8_t)(value >> 8),
      (uint8_t) value,
  };
  this->write_register16(reg, buf, 4);
}

}  // namespace vl53l4cd
}  // namespace esphome
