#include "samsung_nasa_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace samsung_nasa_sniffer {

SamsungNasaSniffer::SamsungNasaSniffer(uart::UARTComponent *parent)
    : uart::UARTDevice(parent), buffer_(NASA_MAX_SIZE) {}

void SamsungNasaSniffer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Samsung NASA sniffer...");
}

void SamsungNasaSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "Samsung NASA Sniffer:");
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

float SamsungNasaSniffer::get_setup_priority() const {
  return setup_priority::DATA;
}

void SamsungNasaSniffer::reset_buffer_() {
  this->index_ = 0;
  this->expected_size_ = 0;
  this->have_size_ = false;
  this->buffer_.assign(NASA_MAX_SIZE, 0);
}

uint16_t SamsungNasaSniffer::crc16_(const uint8_t *data, uint16_t length) const {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

bool SamsungNasaSniffer::validate_packet_(uint16_t size) {
  if (this->buffer_[0] != NASA_START) return false;
  if (this->buffer_[size + 1] != NASA_END) return false;

  uint16_t crc_actual = this->crc16_(this->buffer_.data() + 3, size - 4);
  uint16_t crc_expected = ((uint16_t)this->buffer_[size - 1] << 8) | this->buffer_[size];
  return crc_actual == crc_expected;
}

void SamsungNasaSniffer::log_packet_(uint16_t size) {
  char hex[(size + 4) * 3 + 1];
  for (uint16_t i = 0; i < size + 2; i++) {
    snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02X ", this->buffer_[i]);
  }
  hex[(size + 2) * 3 - 1] = '\0';

  ESP_LOGI(TAG, "NASA %s", hex);

  if (this->last_packet_sensor_ != nullptr) {
    this->last_packet_sensor_->publish_state(hex);
  }
}

void SamsungNasaSniffer::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);

    if (this->index_ == 0 && byte != NASA_START) {
      continue;
    }

    if (this->index_ < NASA_MAX_SIZE) {
      this->buffer_[this->index_++] = byte;
    }

    if (this->index_ == 3 && !this->have_size_) {
      this->expected_size_ = ((uint16_t)this->buffer_[1] << 8) | this->buffer_[2];
      this->have_size_ = true;

      if (this->expected_size_ < 14 || this->expected_size_ > 1500) {
        ESP_LOGW(TAG, "Invalid NASA size %u, resync", this->expected_size_);
        this->reset_buffer_();
        continue;
      }
    }

    if (this->have_size_ && this->index_ >= this->expected_size_ + 2) {
      this->total_packets_++;
      if (this->total_packets_sensor_ != nullptr) {
        this->total_packets_sensor_->publish_state(this->total_packets_);
      }

      if (this->validate_packet_(this->expected_size_)) {
        this->valid_packets_++;
        if (this->valid_packets_sensor_ != nullptr) {
          this->valid_packets_sensor_->publish_state(this->valid_packets_);
        }
        this->log_packet_(this->expected_size_);
      } else {
        this->invalid_packets_++;
        if (this->invalid_packets_sensor_ != nullptr) {
          this->invalid_packets_sensor_->publish_state(this->invalid_packets_);
        }
        ESP_LOGW(TAG, "Invalid NASA packet");
      }

      this->reset_buffer_();
    }
  }
}

}  // namespace samsung_nasa_sniffer
}  // namespace esphome
