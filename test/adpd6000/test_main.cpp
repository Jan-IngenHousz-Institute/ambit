// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#include <stdint.h>

#include <deque>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../src/src/adpd/jii_adpd6000.h"

namespace {

using jii::adpd6000::Driver;
using jii::adpd6000::Transport;

struct Operation {
  uint16_t command;
  std::vector<uint8_t> transmitted;
  size_t received_length;
  bool receive_buffer_word_aligned;
};

struct FakeTransport {
  std::map<uint16_t, uint16_t> registers;
  std::deque<uint8_t> fifo;
  std::vector<Operation> operations;
  size_t fail_on_call = 0;
  int32_t failure = -77;
  bool emulate_dma_word_write = false;

  static int32_t transact(void *context, uint16_t command,
                          const uint8_t *tx_data, uint8_t *rx_data,
                          size_t data_length) {
    FakeTransport &self = *static_cast<FakeTransport *>(context);
    Operation operation = {
        command,
        {},
        rx_data == nullptr ? 0 : data_length,
        rx_data == nullptr ||
            (reinterpret_cast<uintptr_t>(rx_data) % sizeof(uint32_t)) == 0};
    if (tx_data != nullptr) {
      operation.transmitted.assign(tx_data, tx_data + data_length);
    }
    self.operations.push_back(operation);
    if (self.fail_on_call != 0 && self.operations.size() == self.fail_on_call) {
      return self.failure;
    }

    const bool write = (command & 1U) != 0;
    const uint16_t address = command >> 1;
    if (write) {
      if (tx_data == nullptr || data_length != 2) {
        return -88;
      }
      const uint16_t value = static_cast<uint16_t>(
          (static_cast<uint16_t>(tx_data[0]) << 8) | tx_data[1]);
      self.registers[address] = value;
      if (address == jii::adpd6000::reg::kFifoStatus &&
          (value & 0x8000U) != 0) {
        self.fifo.clear();
      }
      return 0;
    }

    if (rx_data == nullptr) {
      return -89;
    }
    if (address == jii::adpd6000::reg::kFifoData) {
      if (self.fifo.size() < data_length) {
        return -90;
      }
      for (size_t i = 0; i < data_length; ++i) {
        rx_data[i] = self.fifo.front();
        self.fifo.pop_front();
      }
      return 0;
    }
    if (data_length != 2) {
      return -91;
    }
    uint16_t value = self.registers[address];
    if (address == jii::adpd6000::reg::kFifoStatus) {
      value = static_cast<uint16_t>((value & 0xF800U) | self.fifo.size());
    }
    rx_data[0] = static_cast<uint8_t>(value >> 8);
    rx_data[1] = static_cast<uint8_t>(value & 0xFF);
    if (self.emulate_dma_word_write) {
      // ESP-IDF documents that SPI DMA writes RX buffers in four-byte units,
      // even when the requested transfer length is only two bytes.
      rx_data[2] = 0xA5;
      rx_data[3] = 0x5A;
    }
    return 0;
  }

  Driver driver() {
    return Driver(Transport{this, &FakeTransport::transact});
  }
};

int failures = 0;

void check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

void test_spi_framing_and_register_byte_order() {
  uint8_t bytes[2] = {0xFF, 0xFF};
  check(Driver::encode_spi_command(jii::adpd6000::reg::kChipId, false,
                                   bytes) == 0,
        "read command encodes");
  check(bytes[0] == 0x00 && bytes[1] == 0x10,
        "CHIP_ID read is address<<1 with R/W=0");
  check(Driver::encode_spi_command(jii::adpd6000::reg::kSystemControl,
                                   true, bytes) == 0,
        "write command encodes");
  check(bytes[0] == 0x00 && bytes[1] == 0x1F,
        "SYS_CTL write is address<<1 with R/W=1");
  check(Driver::encode_spi_command(jii::adpd6000::kMaxRegisterAddress,
                                   true, bytes) == 0 &&
            bytes[0] == 0xFF && bytes[1] == 0xFF,
        "maximum 15-bit register address is accepted");
  check(Driver::encode_spi_command(
            static_cast<uint16_t>(jii::adpd6000::kMaxRegisterAddress + 1U),
            false, bytes) == jii::adpd6000::kInvalidArgument,
        "register address above 15 bits is rejected");

  FakeTransport fake;
  Driver driver = fake.driver();
  check(driver.write_register(0x0128, 0xBEEF) == 0,
        "register write succeeds");
  check(fake.operations.back().command == 0x0251,
        "write command reaches transport unchanged");
  check(fake.operations.back().transmitted == std::vector<uint8_t>({0xBE, 0xEF}),
        "register value is transmitted MSB first");

  fake.registers[0x0128] = 0xCAFE;
  fake.emulate_dma_word_write = true;
  uint16_t value = 0;
  check(driver.read_register(0x0128, &value) == 0 && value == 0xCAFE,
        "register read reconstructs MSB-first value");
  check(fake.operations.back().received_length == 2,
        "register read keeps the wire receive length at two bytes");
  check(fake.operations.back().receive_buffer_word_aligned,
        "register read provides a four-byte-aligned DMA receive buffer");
}

void test_masking_reset_and_run_transitions() {
  FakeTransport fake;
  Driver driver = fake.driver();
  fake.registers[jii::adpd6000::reg::kOperatingMode] = 0xA5F8;

  check(driver.set_running(true) == 0, "run transition succeeds");
  check(fake.registers[jii::adpd6000::reg::kOperatingMode] == 0xA5F9,
        "run changes only OP_MODE bits");
  check(driver.set_running(false) == 0, "stop transition succeeds");
  check(fake.registers[jii::adpd6000::reg::kOperatingMode] == 0xA5F8,
        "stop preserves enabled-slot and unrelated bits");
  check(driver.set_optical_slot_count(3) == 0,
        "optical slot count accepts three slots");
  check(fake.registers[jii::adpd6000::reg::kOperatingMode] == 0xA3F8,
        "slot count changes only PPG_TIMESLOT_EN");
  check(driver.set_optical_slot_count(12) == 0,
        "optical slot count accepts the twelve-slot boundary");
  check(fake.registers[jii::adpd6000::reg::kOperatingMode] == 0xACF8,
        "twelve-slot boundary preserves unrelated OPMODE bits");
  check(driver.set_optical_slot_count(13) == jii::adpd6000::kOutOfRange,
        "optical slot count rejects values above twelve");

  check(driver.software_reset() == 0, "software reset write succeeds");
  check(fake.registers[jii::adpd6000::reg::kSystemControl] == 0x8000,
        "software reset asserts SYS_CTL.SW_RESET");
}

void test_fifo_operations() {
  FakeTransport fake;
  Driver driver = fake.driver();
  fake.fifo = {0x01, 0x23, 0x45, 0x67, 0x89};

  uint16_t count = 0;
  check(driver.fifo_byte_count(&count) == 0 && count == 5,
        "FIFO byte count is read from FIFO_STATUS");
  uint8_t data[3] = {};
  check(driver.read_fifo(data, sizeof(data)) == 0,
        "byte-granular FIFO read succeeds");
  check(data[0] == 0x01 && data[1] == 0x23 && data[2] == 0x45,
        "FIFO preserves byte stream order");
  check(fake.operations.back().command == 0x005E,
        "FIFO read uses the fixed FIFO_DATA read command");

  uint8_t over_capacity = 0;
  const size_t before_invalid_read = fake.operations.size();
  check(driver.read_fifo(&over_capacity,
                         jii::adpd6000::kFifoCapacityBytes + 1U) ==
            jii::adpd6000::kInvalidArgument,
        "FIFO read rejects lengths past the hardware capacity");
  check(fake.operations.size() == before_invalid_read,
        "invalid FIFO length does not reach the transport");

  const size_t before_clear = fake.operations.size();
  check(driver.clear_fifo() == 0, "FIFO clear succeeds");
  check(fake.fifo.empty(), "FIFO clear empties fake FIFO");
  check(fake.operations.size() == before_clear + 2,
        "FIFO clear asserts then deasserts CLEAR_FIFO");
  check(fake.operations[before_clear].transmitted ==
            std::vector<uint8_t>({0x80, 0x00}) &&
            fake.operations[before_clear + 1].transmitted ==
                std::vector<uint8_t>({0x00, 0x00}),
        "FIFO clear transition writes the specified command values");
}

void test_fifo_sample_failure_clears_stale_outputs() {
  FakeTransport fake;
  Driver driver = fake.driver();
  fake.fifo = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
  fake.fail_on_call = 2;
  uint32_t samples[3] = {0xAAAAAAAAU, 0xBBBBBBBBU, 0xCCCCCCCCU};

  check(driver.read_fifo_samples(3, 3, samples) == fake.failure,
        "sample FIFO read propagates a mid-run transport failure");
  check(samples[0] == 0x012345U,
        "samples completed before the failure remain available");
  check(samples[1] == 0 && samples[2] == 0,
        "failed and remaining samples are cleared instead of staying stale");
  check(fake.operations.size() == 2,
        "sample FIFO read stops transport work at the first failure");
}

void test_frequency_bounds_and_identity() {
  FakeTransport fake;
  Driver driver = fake.driver();

  check(driver.set_slot_frequency(960000, 100) == 0,
        "100 Hz slot frequency succeeds");
  check(fake.registers[jii::adpd6000::reg::kTimeslotFrequencyLow] == 0x2580 &&
            fake.registers[jii::adpd6000::reg::kTimeslotFrequencyHigh] == 0,
        "slot period is split across low and high registers");
  check(driver.set_slot_frequency(960000, 0) == jii::adpd6000::kOutOfRange,
        "zero frequency is rejected");
  check(driver.set_slot_frequency(960000, 960001) ==
            jii::adpd6000::kOutOfRange,
        "frequency above timer clock is rejected");
  check(driver.set_slot_frequency(0xFFFFFFFFU, 1) ==
            jii::adpd6000::kOutOfRange,
        "period wider than 23 bits is rejected");

  fake.registers[jii::adpd6000::reg::kChipId] = 0x02C4;
  uint8_t chip = 0;
  uint8_t version = 0;
  check(driver.identify(&chip, &version) == 0 && chip == 0xC4 && version == 2,
        "identity separates chip ID and silicon version bytes");
}

void test_transport_error_propagation() {
  FakeTransport fake;
  Driver driver = fake.driver();
  fake.fail_on_call = 1;

  uint16_t value = 0;
  check(driver.read_register(0x0008, &value) == fake.failure,
        "register read propagates transport error");
  check(fake.operations.size() == 1, "failed read performs one transaction");

  fake.operations.clear();
  fake.fail_on_call = 2;
  fake.registers[jii::adpd6000::reg::kOperatingMode] = 0x1200;
  check(driver.set_running(true) == fake.failure,
        "read-modify-write propagates write error");
  check(fake.operations.size() == 2,
        "read-modify-write stops after failed write");

  fake.operations.clear();
  fake.fail_on_call = 1;
  check(driver.clear_fifo() == fake.failure,
        "FIFO clear propagates first transport error");
  check(fake.operations.size() == 1,
        "FIFO clear does not deassert after failed assertion");
}

}  // namespace

int main() {
  test_spi_framing_and_register_byte_order();
  test_masking_reset_and_run_transitions();
  test_fifo_operations();
  test_fifo_sample_failure_clears_stale_outputs();
  test_frequency_bounds_and_identity();
  test_transport_error_propagation();

  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return 1;
  }
  std::cout << "ADPD6000 fake-transport tests passed\n";
  return 0;
}
