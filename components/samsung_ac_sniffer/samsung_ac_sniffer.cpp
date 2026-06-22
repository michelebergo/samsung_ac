#include "samsung_ac_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace samsung_ac_sniffer {

SamsungAcSniffer::SamsungAcSniffer(uart::UARTComponent *parent)
    : uart::UARTDevice(parent), buffer_(MSG_LENGTH) {}

void SamsungAcSniffer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Samsung AC sniffer...");
}

void SamsungAcSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "Samsung AC Sniffer:");
  this->check_uart_settings(2400, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

float SamsungAcSniffer::get_setup_priority() const {
  return setup_priority::DATA;
}

void SamsungAcSniffer::reset_buffer_() {
  this->index_ = 0;
  this->buffer_.assign(MSG_LENGTH, 0);
}

uint8_t SamsungAcSniffer::checksum_(const uint8_t *data, uint8_t length) const {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < length; i++) {
    cs ^= data[i];
  }
  return cs;
}

bool SamsungAcSniffer::validate_frame_() {
  if (this->buffer_[0] != MSG_START) return false;
  if (this->buffer_[MSG_LENGTH - 1] != MSG_END) return false;
  uint8_t cs = this->checksum_(this->buffer_.data() + 1, CHECKSUM_LENGTH);
  return cs == this->buffer_[MSG_LENGTH - 2];
}

void SamsungAcSniffer::log_frame_() {
  char hex[MSG_LENGTH * 3 + 1];
  for (uint8_t i = 0; i < MSG_LENGTH; i++) {
    snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02X ", this->buffer_[i]);
  }
  hex[MSG_LENGTH * 3 - 1] = '\0';

  uint8_t src = this->buffer_[1];
  uint8_t dst = this->buffer_[2];
  uint8_t cmd = this->buffer_[3];

  ESP_LOGI(TAG, "PKT %s | src=0x%02X dst=0x%02X cmd=0x%02X", hex, src, dst, cmd);

  if (this->last_packet_sensor_ != nullptr) {
    this->last_packet_sensor_->publish_state(hex);
  }
}

void SamsungAcSniffer::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (this->index_ == 0 && byte != MSG_START) {
      // Stay in sync: drop bytes until start marker.
      continue;
    }

    if (this->index_ < MSG_LENGTH) {
      this->buffer_[this->index_++] = byte;
    }

    if (this->index_ == MSG_LENGTH) {
      this->total_packets_++;
      if (this->total_packets_sensor_ != nullptr) {
        this->total_packets_sensor_->publish_state(this->total_packets_);
      }

      if (this->validate_frame_()) {
        this->valid_packets_++;
        if (this->valid_packets_sensor_ != nullptr) {
          this->valid_packets_sensor_->publish_state(this->valid_packets_);
        }
        this->log_frame_();
      } else {
        this->invalid_packets_++;
        if (this->invalid_packets_sensor_ != nullptr) {
          this->invalid_packets_sensor_->publish_state(this->invalid_packets_);
        }
        ESP_LOGW(TAG, "Invalid frame");
      }

      this->reset_buffer_();
    }
  }
}

}  // namespace samsung_ac_sniffer
}  // namespace esphome
