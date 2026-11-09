/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/*
 * Minimal nRF52 UARTE0 implementation of the Mynewt hal_uart API, used by
 * the NimBLE UART HCI transport (H4) to talk to an external controller.
 * Based on Mynewt's hw/mcu/nordic/nrf52xxx/src/hal_uart.c.
 */

#include <stdint.h>
#include <errno.h>
#include "os/os.h"
#include "syscfg/syscfg.h"
#include "nrfx.h"
#include "hal/hal_uart.h"

#define UARTE_INT_ENDTX     UARTE_INTEN_ENDTX_Msk
#define UARTE_INT_ENDRX     UARTE_INTEN_ENDRX_Msk
#define UARTE_ENABLE        (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos)
#define UARTE_DISABLE       (UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos)
#define UARTE_PIN_DISCONNECTED  0xFFFFFFFF

struct hal_uart {
    uint8_t u_open:1;
    uint8_t u_rx_stall:1;
    uint8_t u_tx_started:1;
    uint8_t u_rx_buf;
    uint8_t u_tx_buf[8];
    hal_uart_rx_char u_rx_func;
    hal_uart_tx_char u_tx_func;
    hal_uart_tx_done u_tx_done;
    void *u_func_arg;
};

static struct hal_uart uart0;

int
hal_uart_init_cbs(int port, hal_uart_tx_char tx_func, hal_uart_tx_done tx_done,
                  hal_uart_rx_char rx_func, void *arg)
{
    struct hal_uart *u = &uart0;

    if (port != 0 || u->u_open) {
        return -1;
    }

    u->u_rx_func = rx_func;
    u->u_tx_func = tx_func;
    u->u_tx_done = tx_done;
    u->u_func_arg = arg;

    return 0;
}

static int
hal_uart_tx_fill_buf(struct hal_uart *u)
{
    int data;
    int i;

    for (i = 0; i < (int)sizeof(u->u_tx_buf); i++) {
        data = u->u_tx_func(u->u_func_arg);
        if (data < 0) {
            break;
        }
        u->u_tx_buf[i] = data;
    }

    return i;
}

void
hal_uart_start_tx(int port)
{
    struct hal_uart *u = &uart0;
    os_sr_t sr;
    int rc;

    if (port != 0) {
        return;
    }

    OS_ENTER_CRITICAL(sr);
    if (u->u_tx_started == 0) {
        rc = hal_uart_tx_fill_buf(u);
        if (rc > 0) {
            NRF_UARTE0->INTENSET = UARTE_INT_ENDTX;
            NRF_UARTE0->TXD.PTR = (uint32_t)&u->u_tx_buf;
            NRF_UARTE0->TXD.MAXCNT = rc;
            NRF_UARTE0->TASKS_STARTTX = 1;
            u->u_tx_started = 1;
        }
    }
    OS_EXIT_CRITICAL(sr);
}

void
hal_uart_start_rx(int port)
{
    struct hal_uart *u = &uart0;
    os_sr_t sr;
    int rc;

    if (port != 0) {
        return;
    }

    if (u->u_rx_stall) {
        OS_ENTER_CRITICAL(sr);
        rc = u->u_rx_func(u->u_func_arg, u->u_rx_buf);
        if (rc == 0) {
            u->u_rx_stall = 0;
            NRF_UARTE0->TASKS_STARTRX = 1;
        }
        OS_EXIT_CRITICAL(sr);
    }
}

void
UARTE0_UART0_IRQHandler(void)
{
    struct hal_uart *u = &uart0;
    int rc;

    if ((NRF_UARTE0->INTENSET & UARTE_INT_ENDTX) && NRF_UARTE0->EVENTS_ENDTX) {
        NRF_UARTE0->EVENTS_ENDTX = 0;
        rc = hal_uart_tx_fill_buf(u);
        if (rc > 0) {
            NRF_UARTE0->TXD.PTR = (uint32_t)&u->u_tx_buf;
            NRF_UARTE0->TXD.MAXCNT = rc;
            NRF_UARTE0->TASKS_STARTTX = 1;
        } else {
            if (u->u_tx_done) {
                u->u_tx_done(u->u_func_arg);
            }
            NRF_UARTE0->INTENCLR = UARTE_INT_ENDTX;
            NRF_UARTE0->TASKS_STOPTX = 1;
            u->u_tx_started = 0;
        }
    }

    if (NRF_UARTE0->EVENTS_ENDRX) {
        NRF_UARTE0->EVENTS_ENDRX = 0;
        rc = u->u_rx_func(u->u_func_arg, u->u_rx_buf);
        if (rc < 0) {
            u->u_rx_stall = 1;
        } else {
            NRF_UARTE0->TASKS_STARTRX = 1;
        }
    }
}

static uint32_t
hal_uart_baudrate(int32_t baudrate)
{
    switch (baudrate) {
    case 9600:
        return UARTE_BAUDRATE_BAUDRATE_Baud9600;
    case 19200:
        return UARTE_BAUDRATE_BAUDRATE_Baud19200;
    case 38400:
        return UARTE_BAUDRATE_BAUDRATE_Baud38400;
    case 57600:
        return UARTE_BAUDRATE_BAUDRATE_Baud57600;
    case 115200:
        return UARTE_BAUDRATE_BAUDRATE_Baud115200;
    case 230400:
        return UARTE_BAUDRATE_BAUDRATE_Baud230400;
    case 460800:
        return UARTE_BAUDRATE_BAUDRATE_Baud460800;
    case 921600:
        return UARTE_BAUDRATE_BAUDRATE_Baud921600;
    case 1000000:
        return UARTE_BAUDRATE_BAUDRATE_Baud1M;
    default:
        return 0;
    }
}

int
hal_uart_config(int port, int32_t baudrate, uint8_t databits, uint8_t stopbits,
                enum hal_uart_parity parity, enum hal_uart_flow_ctl flow_ctl)
{
    struct hal_uart *u = &uart0;
    uint32_t cfg_reg = 0;
    uint32_t baud_reg;

    if (port != 0 || u->u_open) {
        return -1;
    }

    /* nRF52 UARTE supports only 8 data bits and 1 or 2 stop bits */
    if (databits != 8) {
        return -1;
    }
    if (stopbits == 2) {
        cfg_reg |= UARTE_CONFIG_STOP_Msk;
    } else if (stopbits != 1) {
        return -1;
    }

    switch (parity) {
    case HAL_UART_PARITY_NONE:
        break;
    case HAL_UART_PARITY_EVEN:
        cfg_reg |= UARTE_CONFIG_PARITY_Msk;
        break;
    default:
        /* Odd parity is not supported by hardware */
        return -1;
    }

    switch (flow_ctl) {
    case HAL_UART_FLOW_CTL_NONE:
        break;
    case HAL_UART_FLOW_CTL_RTS_CTS:
        cfg_reg |= UARTE_CONFIG_HWFC_Msk;
        if (MYNEWT_VAL(BLE_HCI_UART_RTS_PIN) < 0 ||
            MYNEWT_VAL(BLE_HCI_UART_CTS_PIN) < 0) {
            return -1;
        }
        break;
    default:
        return -1;
    }

    baud_reg = hal_uart_baudrate(baudrate);
    if (baud_reg == 0) {
        return -1;
    }

    NVIC_DisableIRQ(UARTE0_UART0_IRQn);

    /* TX idles high; configure as output before connecting it */
    NRF_P0->OUTSET = 1UL << MYNEWT_VAL(BLE_HCI_UART_TX_PIN);
    NRF_P0->PIN_CNF[MYNEWT_VAL(BLE_HCI_UART_TX_PIN)] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P0->PIN_CNF[MYNEWT_VAL(BLE_HCI_UART_RX_PIN)] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_UARTE0->PSEL.TXD = MYNEWT_VAL(BLE_HCI_UART_TX_PIN);
    NRF_UARTE0->PSEL.RXD = MYNEWT_VAL(BLE_HCI_UART_RX_PIN);
    if (flow_ctl == HAL_UART_FLOW_CTL_RTS_CTS) {
        NRF_P0->OUTSET = 1UL << MYNEWT_VAL(BLE_HCI_UART_RTS_PIN);
        NRF_P0->PIN_CNF[MYNEWT_VAL(BLE_HCI_UART_RTS_PIN)] =
            (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
        NRF_P0->PIN_CNF[MYNEWT_VAL(BLE_HCI_UART_CTS_PIN)] =
            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
        NRF_UARTE0->PSEL.RTS = MYNEWT_VAL(BLE_HCI_UART_RTS_PIN);
        NRF_UARTE0->PSEL.CTS = MYNEWT_VAL(BLE_HCI_UART_CTS_PIN);
    } else {
        NRF_UARTE0->PSEL.RTS = UARTE_PIN_DISCONNECTED;
        NRF_UARTE0->PSEL.CTS = UARTE_PIN_DISCONNECTED;
    }

    NRF_UARTE0->BAUDRATE = baud_reg;
    NRF_UARTE0->CONFIG = cfg_reg;

    NRF_UARTE0->EVENTS_ENDRX = 0;
    NRF_UARTE0->EVENTS_ENDTX = 0;

    NVIC_SetPriority(UARTE0_UART0_IRQn, MYNEWT_VAL(BLE_HCI_UART_IRQ_PRIO));
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);

    NRF_UARTE0->ENABLE = UARTE_ENABLE;

    NRF_UARTE0->INTENSET = UARTE_INT_ENDRX;
    NRF_UARTE0->RXD.PTR = (uint32_t)&u->u_rx_buf;
    NRF_UARTE0->RXD.MAXCNT = sizeof(u->u_rx_buf);
    NRF_UARTE0->TASKS_STARTRX = 1;

    u->u_rx_stall = 0;
    u->u_tx_started = 0;
    u->u_open = 1;

    return 0;
}

int
hal_uart_close(int port)
{
    struct hal_uart *u = &uart0;

    if (port != 0) {
        return -1;
    }
    if (!u->u_open) {
        return 0;
    }

    u->u_open = 0;

    /* Stop both directions and wait for the DMA to settle before disabling */
    NRF_UARTE0->INTENCLR = UARTE_INT_ENDTX | UARTE_INT_ENDRX;
    NRF_UARTE0->EVENTS_TXSTOPPED = 0;
    NRF_UARTE0->TASKS_STOPTX = 1;
    while (NRF_UARTE0->EVENTS_TXSTOPPED == 0) {
    }
    NRF_UARTE0->EVENTS_RXTO = 0;
    NRF_UARTE0->TASKS_STOPRX = 1;
    while (NRF_UARTE0->EVENTS_RXTO == 0 && NRF_UARTE0->EVENTS_ENDRX == 0) {
    }
    NRF_UARTE0->EVENTS_ENDRX = 0;
    NRF_UARTE0->EVENTS_ENDTX = 0;
    NRF_UARTE0->EVENTS_RXTO = 0;

    NRF_UARTE0->ENABLE = UARTE_DISABLE;
    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);

    u->u_tx_started = 0;
    u->u_rx_stall = 0;

    return 0;
}
