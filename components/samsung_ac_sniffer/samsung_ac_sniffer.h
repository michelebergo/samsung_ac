#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>

namespace esphome {
namespace samsung_ac_sniffer {

static const char *const TAG = "samsung_ac_sniffer";

// Frame layout (same physical layer as samsung_ac)
static const uint8_t MSG_START = 0x32;
static const uint8_t MSG_END = 0x34;
static const uint8_t MSG_LENGTH = 14;
static const uint8_t CHECKSUM_LENGTH = 11;

class SamsungAcSniffer : public Component, public uart::UARTDevice {
 public:
  explicit SamsungAcSniffer(uart::UARTComponent *parent);

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
  bool validate_frame_();
  void log_frame_();
  uint8_t checksum_(const uint8_t *data, uint8_t length) const;

  std::vector<uint8_t> buffer_;
  uint8_t index_{0};

  uint32_t total_packets_{0};
  uint32_t valid_packets_{0};
  uint32_t invalid_packets_{0};

  text_sensor::TextSensor *last_packet_sensor_{nullptr};
  sensor::Sensor *total_packets_sensor_{nullptr};
  sensor::Sensor *valid_packets_sensor_{nullptr};
  sensor::Sensor *invalid_packets_sensor_{nullptr};
};

}  // namespace samsung_ac_sniffer
}  // namespace esphome
