#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

namespace esphome {
namespace samsung_ac {

// Message structure constants
static const uint8_t SAMSUNG_MSG_START_POS = 0;
static const uint8_t SAMSUNG_MSG_SOURCE_POS = 1;
static const uint8_t SAMSUNG_MSG_DESTINATION_POS = 2;
static const uint8_t SAMSUNG_MSG_COMMAND_POS = 3;
static const uint8_t SAMSUNG_MSG_DATA1_POS = 4;
static const uint8_t SAMSUNG_MSG_DATA2_POS = 5;
static const uint8_t SAMSUNG_MSG_DATA3_POS = 6;
static const uint8_t SAMSUNG_MSG_DATA4_POS = 7;
static const uint8_t SAMSUNG_MSG_DATA5_POS = 8;
static const uint8_t SAMSUNG_MSG_DATA6_POS = 9;
static const uint8_t SAMSUNG_MSG_DATA7_POS = 10;
static const uint8_t SAMSUNG_MSG_DATA8_POS = 11;
static const uint8_t SAMSUNG_MSG_CHECKSUM_POS = 12;
static const uint8_t SAMSUNG_MSG_END_POS = 13;

static const uint8_t SAMSUNG_CHECKSUM_LENGTH = 11;
static const uint8_t SAMSUNG_DATA_LENGTH = 8;
static const uint8_t SAMSUNG_MSG_LENGTH = 14;

static const uint8_t SAMSUNG_MSG_START = 0x32;
static const uint8_t SAMSUNG_MSG_END = 0x34;
static const uint8_t SAMSUNG_SOURCE_ADDRESS = 0x84;
static const uint8_t SAMSUNG_DESTINATION_ADDRESS = 0x20;

class SamsungAcComponent : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  explicit SamsungAcComponent(uart::UARTComponent *parent);

  // ESPHome lifecycle methods
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  bool receiving_recently_() const;
  void schedule_send_cmd_a0_();  

  // Exposed config
  uint8_t unit{0};
  uint8_t source_address{0};

  // Exposed sensors/binary
  sensor::Sensor *blade_position{nullptr};
  binary_sensor::BinarySensor *quiet_mode_sensor{nullptr};
  binary_sensor::BinarySensor *reset_clean_filter_msg{nullptr};
  binary_sensor::BinarySensor *quiet_mode{nullptr};

  // Control hooks
  void set_blade_position(uint8_t position);
  void set_quiet_mode(bool quiet);

 protected:
  // Internal helpers
  optional<bool> check_byte_() const;
  void parse_data(const std::vector<uint8_t> &data);
  void send_cmd_a0_(void);
  void write_command_(const uint8_t cmd, const uint8_t *cmd_data, uint8_t cmd_data_length);
  uint8_t samsung_ac_checksum_(const uint8_t *command_data, uint8_t length) const;

  std::vector<uint8_t> data_{std::vector<uint8_t>(SAMSUNG_MSG_LENGTH)};
  uint8_t data_index_{0};

  // State flags
  bool is_dirty{false};
  bool set_blade_position_{false};
  bool set_quiet_mode_{false};
  uint32_t last_rx_time_{0};
};

}  // namespace samsung_ac
}  // namespace esphome
