/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <aos/aos.h>
#include <k_api.h>
#include <aos/kernel.h>
#include <stdio.h>
#include <stdlib.h>

#define AOS_START_STACK 1024

static ktask_t init_task_obj;
cpu_stack_t init_task_buf[AOS_START_STACK];


ktask_t *g_aos_init;

static kinit_t kinit;

extern int application_start(int argc, char **argv);
extern int aos_framework_init(void);

static void var_init()
{
    kinit.argc = 0;
    kinit.argv = NULL;
    kinit.cli_enable = 1;
}

static void sys_init(void)
{

#ifdef BOOTLOADER
    main();
#else
    var_init();

    /* System time ticker */
	mx_hal_ms_ticker_init();
    aos_kernel_init(&kinit);
#endif
}


static void sys_start(void)
{
    aos_heap_set();

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    aos_init();

    krhino_task_create(&init_task_obj, "aos-init", 0,AOS_DEFAULT_APP_PRI, 
        0, init_task_buf, AOS_START_STACK, (task_entry_t)sys_init, 1);

    aos_start();
}

int main(void)
{
    sys_start();
    return 0;
}

#include <k_api.h>
#include "hal/soc/uart.h"
#include "hal/soc/timer.h"
#include "saml21.h"

uart_dev_t uart_0;

int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout) {

    return 0;
}

int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t *recv_size, uint32_t timeout) {

    return 0;
}

void hal_reboot(void)
{
    NVIC_SystemReset();
}

static void _timer_cb(void *timer, void *arg)
{
    timer_dev_t *tmr = arg;
    tmr->config.cb(tmr->config.arg);
}

int32_t hal_timer_init(timer_dev_t *tim)
{
    if (tim->config.reload_mode == TIMER_RELOAD_AUTO) {
        krhino_timer_dyn_create((ktimer_t **)&tim->priv, "hwtmr", _timer_cb,
                                us2tick(tim->config.period), us2tick(tim->config.period), tim, 0);
    }
    else {
        krhino_timer_dyn_create((ktimer_t **)&tim->priv, "hwtmr", _timer_cb,
                                us2tick(tim->config.period), 0, tim, 0);
    }
}

int32_t hal_timer_start(timer_dev_t *tmr)
{
    return krhino_timer_start(tmr->priv);
}


void hal_timer_stop(timer_dev_t *tmr)
{
    krhino_timer_stop(tmr->priv);
    krhino_timer_dyn_del(tmr->priv);
    tmr->priv = NULL;
}