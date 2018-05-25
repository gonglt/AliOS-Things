/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include <k_api.h>
#include "hal/soc/uart.h"
#include "hal/soc/timer.h"
#include "saml21.h"

uart_dev_t uart_0;

int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout)
{
    stdio_io_write((const uint8_t *)data, size);
    return 0;
}

int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t *recv_size, uint32_t timeout)
{
    *recv_size = expect_size;
    stdio_io_read((const uint8_t *)data, expect_size);
    return 0;
}
