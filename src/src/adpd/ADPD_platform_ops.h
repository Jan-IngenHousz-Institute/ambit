// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#ifndef JII_ADPD_PLATFORM_OPS_H
#define JII_ADPD_PLATFORM_OPS_H

#include <stddef.h>
#include <stdint.h>

int32_t jii_adpd6000_spi_transact(void *context, uint16_t command,
                                  const uint8_t *tx_data, uint8_t *rx_data,
                                  size_t data_length);

#endif  // JII_ADPD_PLATFORM_OPS_H
