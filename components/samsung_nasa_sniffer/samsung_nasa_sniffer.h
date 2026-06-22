#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>

namespace esphome {
namespace samsung_nasa_sniffer {

static const char *const TAG = "samsung_nasa_sniffer";

static const uint8_t NASA_START = 0x32;
static const uint8_t NASA_END = 0x34;
static const uint16_t NASA_MAX_SIZE = 512;

class SamsungNasaSniffer : public Component, public uart::UARTDevice {
 public:
  explicit SamsungNasaSniffer(uart::UARTComponent *parent);

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_last_packet_sensor(text_sensor::TextSensor *sensor) { this->last_packet_sensor_ = sensor; }
  void set_total_packets_sensor(sensor::Sensor *sensor) { this->total_packets_sensor_ = sensor; }
  void set_valid_packets_sensor(sensor::Sensor *sensor) { this->valid_packets_sensor_ = sensor; }
  void set_invalid_packets_sensor(sensor::Sensor *sensor) { this->invalid_packets_sensor_ = sensor; }

 protected:
  void reset_buffer_();
  bool validate_packet_(uint16_t size);
  void log_packet_(uint16_t size);
  uint16_t crc16_(const uint8_t *data, uint16_t length) const;

  std::vector<uint8_t> buffer_;
  uint16_t index_{0};
  uint16_t expected_size_{0};
  bool have_size_{false};

  uint32_t total_packets_{0};
  uint32_t valid_packets_{0};
  uint32_t invalid_packets_{0};

  text_sensor::TextSensor *last_packet_sensor_{nullptr};
  sensor::Sensor *total_packets_sensor_{nullptr};
  sensor::Sensor *valid_packets_sensor_{nullptr};
  sensor::Sensor *invalid_packets_sensor_{nullptr};
};

}  // namespace samsung_nasa_sniffer
}  // namespace esphome
