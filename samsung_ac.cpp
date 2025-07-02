#include "esphome/components/samsung_ac/samsung_ac.h"
#include "esphome/core/log.h"

namespace esphome {
namespace samsung_ac {

static const char *const TAG = "samsung_ac";

// Costruttore
SamsungAcComponent::SamsungAcComponent(uart::UARTComponent *parent)
    : uart::UARTDevice(parent), data_(14) {}

void SamsungAcComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Samsung AC...");
}

void SamsungAcComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Samsung AC:");
  LOG_CLIMATE("  ", "Samsung AC Climate", this);
  ESP_LOGCONFIG(TAG, "  Unit: %u", this->unit);
  ESP_LOGCONFIG(TAG, "  Source Address: 0x%02X", this->source_address);
  this->check_uart_settings(2400,1,uart::UART_CONFIG_PARITY_EVEN,8);
}

climate::ClimateTraits SamsungAcComponent::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_AUTO,
  });

  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
      climate::CLIMATE_SWING_BOTH,
  });

  traits.set_supports_current_temperature(true);

  // 👇 Add this line to force 1°C step
  traits.set_visual_temperature_step(1.0f);

  return traits;
}

void SamsungAcComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->data_[this->data_index_] = byte;
    this->last_rx_time_ = millis();  // Update last RX time

    auto check = this->check_byte_();
    if (!check.has_value()) {
      if (this->data_index_ == SAMSUNG_MSG_END_POS) {
        ESP_LOGV(TAG, "Data received %i", this->data_[this->data_index_]);
        this->parse_data(this->data_);
      }
      this->data_index_ = 0;
    } else if (!*check) {
      ESP_LOGV(TAG, "Invalid byte %i", this->data_index_);
      this->data_index_ = 0;
    } else {
      this->data_index_++;
    }
  }
}

float SamsungAcComponent::get_setup_priority() const {
  return setup_priority::DATA;
}

optional<bool> SamsungAcComponent::check_byte_() const {
  uint8_t index = this->data_index_;
  uint8_t byte = this->data_[index];

  if (index == SAMSUNG_MSG_START_POS)
    return byte == SAMSUNG_MSG_START;

  if (index < SAMSUNG_MSG_CHECKSUM_POS)
    return true;

  if (index == SAMSUNG_MSG_CHECKSUM_POS) {
    uint8_t checksum = samsung_ac_checksum_(this->data_.data() + 1, SAMSUNG_CHECKSUM_LENGTH);
    return checksum == byte;
  }

  if (index == SAMSUNG_MSG_END_POS && byte == SAMSUNG_MSG_END)
    return {};

  return false;
}

void SamsungAcComponent::control(const climate::ClimateCall &call) {
  // Update internal state from Home Assistant call
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();

  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  if (call.get_fan_mode().has_value())
    this->fan_mode = *call.get_fan_mode();

  if (call.get_swing_mode().has_value())
    this->swing_mode = *call.get_swing_mode();

  if (call.get_preset().has_value())
    this->preset = *call.get_preset();

  this->is_dirty = true;

  this->publish_state();
}

bool SamsungAcComponent::receiving_recently_() const {
  return millis() - this->last_rx_time_ < 300;  // or whatever idle time is safe
}

void SamsungAcComponent::schedule_send_cmd_a0_() {
  uint32_t since = millis() - this->last_rx_time_;
  uint32_t delay_time = (since < 300) ? (300 - since) : 5;

  ESP_LOGD(TAG, "Scheduling A0 in %ums (last RX %ums ago)", delay_time, since);

  this->cancel_timeout("send_cmd_a0");
  this->set_timeout("send_cmd_a0", delay_time, [this]() {
    uint32_t now = millis();
    uint32_t since_last_rx = now - this->last_rx_time_;
    if (since_last_rx < 300) {
      ESP_LOGW(TAG, "Bus still active (%ums since last RX), deferring A0", since_last_rx);
      this->schedule_send_cmd_a0_();  // retry again
    } else {
      this->send_cmd_a0_();
      ESP_LOGW(TAG, "Sent A0 after %ums of bus idle", since_last_rx);
    }
  });
}


void SamsungAcComponent::parse_data(const std::vector<uint8_t> &data) {
  if (data.size() != SAMSUNG_MSG_LENGTH) {
    ESP_LOGW(TAG, "Invalid packet size: %d", data.size());
    return;
  }

  /* ESP_LOGV(TAG, "Raw packet:");
  for (size_t i = 0; i < data.size(); i++) {
    ESP_LOGV(TAG, "[%02d] = 0x%02X", i, data[i]);
  } */
  
  std::string packet_log = "Raw packet: ";
  for (size_t i = 0; i < data.size(); i++) {
  char byte_str[6];
  snprintf(byte_str, sizeof(byte_str), "%02X ", data[i]);
  packet_log += byte_str;
  }
  ESP_LOGV(TAG, "%s", packet_log.c_str());


   if (this->is_dirty) {
            if (this->data_[SAMSUNG_MSG_COMMAND_POS] == 0xD1) {
                // The master(wired wall controller) is sending periodically commands to the slave and a broadcast message. 
                // without receiving any answer from the slave and that’s the time window where to send our control 
                // command(broadcast command sent to destination address 0xAD). So we need a routine checking if 
                // the master is sending a broadcast message and when the message is ended we can send our control command emulating the master.
                this->cancel_timeout("send_cmd_a0");
                this->set_timeout("send_cmd_a0", 50, [this]() { this->send_cmd_a0_(); });
                //this->send_cmd_a0_();
                this->is_dirty = false;
                ESP_LOGW(TAG, "sending command A0");
            }

            // we cannot update variables that are ready to be submitted to the slave
            // so we wait until we receive a 0xAD message from the master and we've 
            // taken our timewindow to send our variables to the slave device
            return;
    }

    /* // After setting a target or changing mode
    if (this->is_dirty) {
        this->cancel_timeout("send_cmd_a0");
        this->set_timeout("send_cmd_a0", 50, [this]() {
        if (millis() - this->last_rx_time_ > 50) {
            this->send_cmd_a0_();
            ESP_LOGW(TAG, "Sending command A0");
        } else {
            ESP_LOGW(TAG, "Skipped A0 due to recent UART RX");
        }
        });
    this->is_dirty = false;
    } */


    // for other messages we are only interested in slave replies
    if (this->data_[SAMSUNG_MSG_DESTINATION_POS] == SAMSUNG_DESTINATION_ADDRESS) {
        return;
    }
  
    switch (this->data_[SAMSUNG_MSG_COMMAND_POS]) {
    
        case 0x52: {
            
            const uint8_t *d = &data[SAMSUNG_MSG_DATA1_POS];  // points to data[4]

            // Interpret temperatures
            uint8_t set_temp_raw = d[0] & 0x7F;
            uint8_t room_temp_raw = d[1] & 0x7F;
            uint8_t air_temp_raw = d[2] & 0x7F;

            this->target_temperature = set_temp_raw - 55.0f;
            this->current_temperature = room_temp_raw - 55.0f;
            float output_air_temperature = air_temp_raw - 55.0f;

            // Fan mode
            uint8_t fan_speed = d[3] & 0x07;
            switch (fan_speed) {
            case 0:
                this->fan_mode = climate::CLIMATE_FAN_AUTO;
                break;
            case 2:
                this->fan_mode = climate::CLIMATE_FAN_LOW;
                break;
            case 4:
                this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
                break;
            case 5:
                this->fan_mode = climate::CLIMATE_FAN_HIGH;
                break;
            default:
                this->fan_mode = climate::CLIMATE_FAN_AUTO;
                break;
            }

            // Blade swing mode (if you want to map it)
            uint8_t swing_bits = (d[3] & 0xF8) >> 3;
            if (swing_bits == 0x1A)
            this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
            else
            this->swing_mode = climate::CLIMATE_SWING_OFF;

            // Power state
            bool power_on = (d[4] & 0x80) != 0;

            // Operating mode
            this->mode = power_on ? climate::CLIMATE_MODE_COOL : climate::CLIMATE_MODE_OFF;

            ESP_LOGD(TAG, "Parsed 0x52 reply:");
            ESP_LOGD(TAG, "  Target temp: %.1f °C", this->target_temperature);
            ESP_LOGD(TAG, "  Room temp: %.1f °C", this->current_temperature);
            ESP_LOGD(TAG, "  Output air temp: %.1f °C", output_air_temperature);
            ESP_LOGD(TAG, "  Power: %s", power_on ? "ON" : "OFF");
            ESP_LOGD(TAG, "  Fan mode: %s", 
                this->fan_mode == climate::CLIMATE_FAN_AUTO   ? "AUTO" :
                this->fan_mode == climate::CLIMATE_FAN_LOW    ? "LOW" :
                this->fan_mode == climate::CLIMATE_FAN_MEDIUM ? "MEDIUM" :
                this->fan_mode == climate::CLIMATE_FAN_HIGH   ? "HIGH" :
                "UNKNOWN");

            ESP_LOGD(TAG, "  Swing mode: %s", 
                this->swing_mode == climate::CLIMATE_SWING_OFF      ? "OFF" :
                this->swing_mode == climate::CLIMATE_SWING_VERTICAL ? "VERTICAL" :
                "UNKNOWN");
        // TODO:
            // | 5 | bit 3 - 0 : 1 = wired control, 2 = remote control |
            // |   | bit 4 : 1 = defrost on |
            // | 6 | bit 4 : 1 = filter needs cleaning |

        } break;

        case 0x53: {
            switch (this->data_[SAMSUNG_MSG_DATA5_POS]) {
            case 0x1a:
                this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
                break;
            default:
                this->swing_mode = climate::CLIMATE_SWING_OFF;
                break;
            }

        } break;

        case 0x54: 
            
        break;

        case 0x64:

            // TODO: | 2 | command and reply : bit 0 : used temperature for regulation, 0 = use internal temperature, 1 = use wired remote temperature
            // wired_remote_temperature = ((this->data_[PROTOCOL_DATA3_POS] << 8) + this->data_[PROTOCOL_DATA4_POS] - 553) / 10.0f;
            // unit_temperature = ((this->data_[PROTOCOL_DATA5_POS] << 8) + this->data_[PROTOCOL_DATA6_POS] - 553) / 10.0f;

        break;

    }

    this->publish_state();
}


void SamsungAcComponent::send_cmd_a0_(void) {
  uint8_t cmd[SAMSUNG_DATA_LENGTH] = { 0 };

  // turn on by default
  cmd[4] = 0xf4;

  switch (this->mode) {
    case climate::CLIMATE_MODE_OFF:
      cmd[4] = 0xc4;
      break;
    case climate::CLIMATE_MODE_AUTO:
      cmd[3] |= (0 & 3);
      break;
    case climate::CLIMATE_MODE_COOL:
      cmd[3] |= (1 & 3);
      break;
    case climate::CLIMATE_MODE_HEAT:
      cmd[3] |= (4 & 3);
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      cmd[3] |= (3 & 3);
      break;
    case climate::CLIMATE_MODE_DRY:
      cmd[3] |= (2 & 3);
      break;
    default:
      break;
  }

  switch (this->swing_mode) {
    case climate::CLIMATE_SWING_OFF:
      cmd[0] |= 0x1f;
      break;
    case climate::CLIMATE_SWING_VERTICAL:
      cmd[0] |= 0x1a;
      break;
    case climate::CLIMATE_SWING_BOTH:
    case climate::CLIMATE_SWING_HORIZONTAL:
    default:
      break;
  }

  switch (this->fan_mode.value()) {
    case climate::CLIMATE_FAN_AUTO:
      cmd[2] |= (0 << 5);
      break;
    case climate::CLIMATE_FAN_LOW:
      cmd[2] |= (2 << 5);
      break;
    case climate::CLIMATE_FAN_MEDIUM:
      cmd[2] |= (4 << 5);
      break;
    case climate::CLIMATE_FAN_HIGH:
      cmd[2] |= (5 << 5);
      break;
    default:
      break;
  }

  //cmd[2] |= (((uint8_t) round(this->target_temperature)) & 0xf) - 9;
  cmd[2] |= ((uint8_t) round(this->target_temperature)) & 0x1F;

  if (this->set_blade_position_) {
    cmd[6] |= ((uint8_t) this->blade_position->state) & 7;
    cmd[6] |= (1 << 4);
    this->set_blade_position_ = false;
  }

  if (this->set_quiet_mode_ && this->quiet_mode->state == true) {
    cmd[6] |= (1 << 5);
    this->set_quiet_mode_ = false;
  }

  if (this->reset_clean_filter_msg != nullptr && this->reset_clean_filter_msg->state == true) {
    cmd[3] |= (1 << 5);
    this->reset_clean_filter_msg->publish_state(false);
  }

  if (this->preset.value() == climate::CLIMATE_PRESET_SLEEP) {
    cmd[0] |= (1 << 5);
  }
  
  this->write_command_(0xa0, cmd, sizeof(cmd));
}

void SamsungAcComponent::write_command_(const uint8_t cmd, const uint8_t *cmd_data, uint8_t cmd_data_length) {
  uint8_t msg[SAMSUNG_MSG_LENGTH] = {0};
  msg[SAMSUNG_MSG_START_POS] = SAMSUNG_MSG_START;
  msg[SAMSUNG_MSG_SOURCE_POS] = SAMSUNG_SOURCE_ADDRESS;
  msg[SAMSUNG_MSG_DESTINATION_POS] = SAMSUNG_DESTINATION_ADDRESS;
  msg[SAMSUNG_MSG_COMMAND_POS] = cmd;
  memcpy(msg + SAMSUNG_MSG_DATA1_POS, cmd_data, std::min(cmd_data_length, SAMSUNG_DATA_LENGTH));
  msg[SAMSUNG_MSG_CHECKSUM_POS] = samsung_ac_checksum_(msg + 1, SAMSUNG_CHECKSUM_LENGTH);
  msg[SAMSUNG_MSG_END_POS] = SAMSUNG_MSG_END;
  this->write_array(msg, sizeof(msg));
   // Log full hex packet in one line
  char log_line[128];
  char *ptr = log_line;
  for (int i = 0; i < SAMSUNG_MSG_LENGTH; i++) {
    ptr += sprintf(ptr, "%02X ", msg[i]);
  }
  *ptr = '\0';  // Null-terminate
  ESP_LOGD(TAG, "Sending packet: %s", log_line);
}

uint8_t SamsungAcComponent::samsung_ac_checksum_(const uint8_t *command_data, uint8_t length) const {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++)
    checksum ^= command_data[i];
  return checksum;
}

void SamsungAcComponent::set_blade_position(uint8_t position) {
  if (this->blade_position != nullptr) {
    this->set_blade_position_ = true;
    this->blade_position->publish_state(position);
    this->is_dirty = true;
  }
}

void SamsungAcComponent::set_quiet_mode(bool quiet) {
  if (this->quiet_mode_sensor != nullptr) {
  this->set_quiet_mode_ = true;
  this->quiet_mode_sensor->publish_state(quiet);
  this->is_dirty = true;
}
}

}  // namespace samsung_ac
}  // namespace esphome
