// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#ifndef JII_ADPD6000_H
#define JII_ADPD6000_H

#include <stddef.h>
#include <stdint.h>

namespace jii {
namespace adpd6000 {

constexpr int32_t kOk = 0;
constexpr int32_t kInvalidArgument = -1;
constexpr int32_t kOutOfRange = -2;
constexpr uint16_t kMaxRegisterAddress = 0x7FFF;
constexpr uint16_t kFifoCapacityBytes = 640;
constexpr uint32_t kMaxTimeslotPeriod = 0x7FFFFF;

enum class Slot : uint8_t {
  A = 0,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
};

namespace reg {

// ADPD6000 Rev. 0 data sheet, Table 20 and Table 21. Register names and
// numeric addresses are public hardware facts. AMBIT uses only the optical,
// GPIO, clock, and FIFO subset below.
constexpr uint16_t kFifoStatus = 0x0000;
constexpr uint16_t kChipId = 0x0008;
constexpr uint16_t kTimeslotFrequencyLow = 0x000D;
constexpr uint16_t kTimeslotFrequencyHigh = 0x000E;
constexpr uint16_t kSystemControl = 0x000F;
constexpr uint16_t kOperatingMode = 0x0010;
constexpr uint16_t kInputSleep = 0x0020;
constexpr uint16_t kInputConfig = 0x0021;
constexpr uint16_t kGpioConfig = 0x0022;
constexpr uint16_t kGpio01 = 0x0023;
constexpr uint16_t kGpio23 = 0x0024;
constexpr uint16_t kGpioExternal = 0x0026;
constexpr uint16_t kFifoData = 0x002F;
constexpr uint16_t kAdcControl = 0x0046;
constexpr uint16_t kGlobalBiasControl = 0x004C;
constexpr uint16_t kIoAdjust = 0x0057;

constexpr uint16_t kSlotStride = 0x0020;
constexpr uint16_t kTimeslotControlA = 0x0120;
constexpr uint16_t kTimeslotPathA = 0x0121;
constexpr uint16_t kInputsA = 0x0122;
constexpr uint16_t kCathodeA = 0x0123;
constexpr uint16_t kAfeTrim1A = 0x0124;
constexpr uint16_t kAfeTrim2A = 0x0125;
constexpr uint16_t kAfeDac1A = 0x0126;
constexpr uint16_t kAfeDac2A = 0x0127;
constexpr uint16_t kLedPower12A = 0x0128;
constexpr uint16_t kLedModeA = 0x0129;
constexpr uint16_t kCountsA = 0x012A;
constexpr uint16_t kPeriodA = 0x012B;
constexpr uint16_t kLedPulse1A = 0x012C;
constexpr uint16_t kLedPulse2A = 0x012D;
constexpr uint16_t kIntegratorWidthA = 0x012E;
constexpr uint16_t kIntegratorOffsetA = 0x012F;
constexpr uint16_t kModulationPulseA = 0x0130;
constexpr uint16_t kPattern1A = 0x0131;
constexpr uint16_t kData1A = 0x0135;
constexpr uint16_t kData2A = 0x0136;
constexpr uint16_t kDigitalIntegrationLitA = 0x0138;
constexpr uint16_t kDigitalIntegrationDarkA = 0x0139;

constexpr uint16_t for_slot(uint16_t slot_a_register, Slot slot) {
  return static_cast<uint16_t>(slot_a_register +
                               static_cast<uint8_t>(slot) * kSlotStride);
}

}  // namespace reg

struct Transport {
  void *context;
  // The 16-bit command is shifted MSB first while CS is asserted, followed by
  // data_length bytes. Reads leave tx_data null; writes leave rx_data null.
  int32_t (*transact)(void *context, uint16_t command,
                      const uint8_t *tx_data, uint8_t *rx_data,
                      size_t data_length);
};

class Driver {
 public:
  Driver();
  explicit Driver(Transport transport);

  void set_transport(Transport transport);

  static int32_t encode_spi_command(uint16_t register_address, bool write,
                                    uint8_t command_bytes[2]);

  int32_t read_register(uint16_t register_address, uint16_t *value) const;
  int32_t write_register(uint16_t register_address, uint16_t value) const;
  int32_t update_register(uint16_t register_address, uint16_t mask,
                          uint16_t value) const;

  int32_t identify(uint8_t *chip_id, uint8_t *silicon_version) const;
  int32_t software_reset() const;
  int32_t set_running(bool running) const;
  int32_t set_slot_frequency(uint32_t timer_clock_hz,
                             uint32_t frequency_hz) const;
  int32_t set_optical_slot_count(uint8_t count) const;

  int32_t clear_fifo() const;
  int32_t fifo_byte_count(uint16_t *count) const;
  int32_t read_fifo(uint8_t *data, size_t length) const;
  int32_t read_fifo_samples(uint16_t num_samples, uint8_t width,
                            uint32_t *data) const;

 private:
  int32_t transact(uint16_t register_address, bool write,
                   const uint8_t *tx_data, uint8_t *rx_data,
                   size_t data_length) const;

  Transport transport_;
};

}  // namespace adpd6000
}  // namespace jii

#endif  // JII_ADPD6000_H
