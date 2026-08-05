// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#include "jii_adpd6000.h"

#if defined(__has_include)
#if __has_include("esp_attr.h")
#include "esp_attr.h"
#endif
#endif

// Host tests do not carry ESP-IDF's include path. Production builds use the
// exact WORD_ALIGNED_ATTR definition from esp_attr.h; this identical fallback
// keeps the transport-agnostic driver testable on the host.
#ifndef WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif

namespace jii {
namespace adpd6000 {

namespace {

constexpr uint16_t kReadCommandBit = 0;
constexpr uint16_t kWriteCommandBit = 1;
constexpr uint16_t kSoftwareReset = 0x8000;
constexpr uint16_t kOperatingModeMask = 0x0007;
constexpr uint16_t kOperatingModeRun = 0x0001;
constexpr uint16_t kOpticalSlotMask = 0x0F00;
constexpr uint16_t kClearFifo = 0x8000;
constexpr uint16_t kFifoByteCountMask = 0x07FF;

}  // namespace

Driver::Driver() : transport_{nullptr, nullptr} {}

Driver::Driver(Transport transport) : transport_(transport) {}

void Driver::set_transport(Transport transport) { transport_ = transport; }

int32_t Driver::encode_spi_command(uint16_t register_address, bool write,
                                   uint8_t command_bytes[2]) {
  if (command_bytes == nullptr || register_address > kMaxRegisterAddress) {
    return kInvalidArgument;
  }

  // Figures 38 and 39 of the Rev. 0 data sheet define a 15-bit address
  // followed by R/W as the least-significant command bit: 0 reads, 1 writes.
  const uint16_t command = static_cast<uint16_t>(
      (register_address << 1) | (write ? kWriteCommandBit : kReadCommandBit));
  command_bytes[0] = static_cast<uint8_t>(command >> 8);
  command_bytes[1] = static_cast<uint8_t>(command & 0xFF);
  return kOk;
}

int32_t Driver::transact(uint16_t register_address, bool write,
                         const uint8_t *tx_data, uint8_t *rx_data,
                         size_t data_length) const {
  if (transport_.transact == nullptr || register_address > kMaxRegisterAddress ||
      data_length == 0) {
    return kInvalidArgument;
  }

  uint8_t command_bytes[2];
  const int32_t encode_result =
      encode_spi_command(register_address, write, command_bytes);
  if (encode_result != kOk) {
    return encode_result;
  }
  const uint16_t command = static_cast<uint16_t>(
      (static_cast<uint16_t>(command_bytes[0]) << 8) | command_bytes[1]);
  return transport_.transact(transport_.context, command, tx_data, rx_data,
                             data_length);
}

int32_t Driver::read_register(uint16_t register_address, uint16_t *value) const {
  if (value == nullptr) {
    return kInvalidArgument;
  }

  // ESP-IDF's SPI DMA engine writes RX data in four-byte units even though an
  // ADPD register contributes only two bytes. Keep the on-wire length at two,
  // but provide the explicitly aligned four-byte storage DMA requires.
  WORD_ALIGNED_ATTR uint8_t bytes[4] = {0, 0, 0, 0};
  const int32_t result = transact(register_address, false, nullptr, bytes, 2);
  if (result != kOk) {
    return result;
  }
  *value = static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                 bytes[1]);
  return kOk;
}

int32_t Driver::write_register(uint16_t register_address, uint16_t value) const {
  const uint8_t bytes[2] = {static_cast<uint8_t>(value >> 8),
                            static_cast<uint8_t>(value & 0xFF)};
  return transact(register_address, true, bytes, nullptr, sizeof(bytes));
}

int32_t Driver::update_register(uint16_t register_address, uint16_t mask,
                                uint16_t value) const {
  uint16_t current = 0;
  int32_t result = read_register(register_address, &current);
  if (result != kOk) {
    return result;
  }
  const uint16_t updated =
      static_cast<uint16_t>((current & static_cast<uint16_t>(~mask)) |
                            (value & mask));
  return write_register(register_address, updated);
}

int32_t Driver::identify(uint8_t *chip_id, uint8_t *silicon_version) const {
  if (chip_id == nullptr || silicon_version == nullptr) {
    return kInvalidArgument;
  }
  uint16_t identity = 0;
  const int32_t result = read_register(reg::kChipId, &identity);
  if (result != kOk) {
    return result;
  }
  *chip_id = static_cast<uint8_t>(identity & 0xFF);
  *silicon_version = static_cast<uint8_t>(identity >> 8);
  return kOk;
}

int32_t Driver::software_reset() const {
  return write_register(reg::kSystemControl, kSoftwareReset);
}

int32_t Driver::set_running(bool running) const {
  return update_register(reg::kOperatingMode, kOperatingModeMask,
                         running ? kOperatingModeRun : 0);
}

int32_t Driver::set_slot_frequency(uint32_t timer_clock_hz,
                                   uint32_t frequency_hz) const {
  if (timer_clock_hz == 0 || frequency_hz == 0 ||
      frequency_hz > timer_clock_hz) {
    return kOutOfRange;
  }
  const uint32_t period = timer_clock_hz / frequency_hz;
  if (period == 0 || period > kMaxTimeslotPeriod) {
    return kOutOfRange;
  }

  int32_t result = write_register(reg::kTimeslotFrequencyLow,
                                  static_cast<uint16_t>(period & 0xFFFF));
  if (result != kOk) {
    return result;
  }
  return write_register(reg::kTimeslotFrequencyHigh,
                        static_cast<uint16_t>((period >> 16) & 0x7F));
}

int32_t Driver::set_optical_slot_count(uint8_t count) const {
  if (count > 12) {
    return kOutOfRange;
  }
  return update_register(reg::kOperatingMode, kOpticalSlotMask,
                         static_cast<uint16_t>(count) << 8);
}

int32_t Driver::clear_fifo() const {
  int32_t result = write_register(reg::kFifoStatus, kClearFifo);
  if (result != kOk) {
    return result;
  }
  // CLEAR_FIFO is a command bit. The data sheet requires it to be returned to
  // zero after the clear so the FIFO can resume operation.
  return write_register(reg::kFifoStatus, 0);
}

int32_t Driver::fifo_byte_count(uint16_t *count) const {
  if (count == nullptr) {
    return kInvalidArgument;
  }
  uint16_t status = 0;
  const int32_t result = read_register(reg::kFifoStatus, &status);
  if (result != kOk) {
    return result;
  }
  *count = static_cast<uint16_t>(status & kFifoByteCountMask);
  return kOk;
}

int32_t Driver::read_fifo(uint8_t *data, size_t length) const {
  if (data == nullptr || length == 0 || length > kFifoCapacityBytes) {
    return kInvalidArgument;
  }
  // The FIFO address does not auto-increment like normal registers. A single
  // byte-granular transaction drains consecutive bytes while CS stays low.
  return transact(reg::kFifoData, false, nullptr, data, length);
}

int32_t Driver::read_fifo_samples(uint16_t num_samples, uint8_t width,
                                  uint32_t *data) const {
  uint8_t readout[4];
  if (data == nullptr || num_samples == 0 || width == 0 ||
      width > sizeof(readout)) {
    return kInvalidArgument;
  }

  for (uint16_t i = 0; i < num_samples; ++i) {
    data[i] = 0;
    const int32_t result = read_fifo(readout, width);
    if (result != kOk) {
      // Callers historically ignore readfifo's status. Clear the failed and
      // unattempted samples so a transport fault cannot republish values left
      // in the caller's reusable buffer by a prior measurement point.
      for (uint16_t remaining = i; remaining < num_samples; ++remaining) {
        data[remaining] = 0;
      }
      return result;
    }
    for (uint8_t byte = 0; byte < width; ++byte) {
      data[i] |= static_cast<uint32_t>(readout[byte])
                 << ((width - byte - 1) * 8);
    }
  }
  return kOk;
}

}  // namespace adpd6000
}  // namespace jii
