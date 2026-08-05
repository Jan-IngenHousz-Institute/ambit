// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#include "ADPD_platform_ops.h"

#include "driver/spi_master.h"

int32_t jii_adpd6000_spi_transact(void *context, uint16_t command,
                                  const uint8_t *tx_data, uint8_t *rx_data,
                                  size_t data_length) {
  if (context == nullptr || data_length == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const spi_device_handle_t handle =
      *static_cast<spi_device_handle_t *>(context);
  if (handle == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  spi_transaction_t transaction = {};
  transaction.addr = command;
  transaction.length = data_length * 8;
  transaction.tx_buffer = tx_data;
  if (rx_data != nullptr) {
    transaction.rxlength = data_length * 8;
    transaction.rx_buffer = rx_data;
  }

  // Return the ESP-IDF status unchanged. The clean-room driver deliberately
  // propagates transport failures to its caller rather than turning them into
  // successful register operations on a disconnected sensor.
  return static_cast<int32_t>(
      spi_device_polling_transmit(handle, &transaction));
}
