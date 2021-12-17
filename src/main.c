/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <devicetree.h>
#include <device.h>
#include <errno.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <storage/flash_map.h>
#include <dfu/flash_img.h>
#include <dfu/mcuboot.h>
#include <drivers/flash.h>
#include <drivers/uart.h>
//#include <storage/stream_flash.h>

//// Define area begin ////
#define MAX_DATA_LEN_RX 256
#define MAX_DATA_LEN_TX 10
#define TIMEOUT 0
//// Define area end ////

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(serial_dfu);

uint8_t buf_tx[MAX_DATA_LEN_TX] = {};
uint8_t buf_rx[MAX_DATA_LEN_RX] = {};

//// Structs and enums area begin ////
typedef enum Status_e {
    STATUS_SYNC = 0,
    STATUS_FIRMWARE_RECV = 1,
    STATUS_LOAD_FIRMWARE = 2,
    STATUS_IDLE = 3,
    STATUS_END_FIRMWARE = 4
} dfu_status_t;

struct dfu_status_s {
    enum Status_e cur_status;
    void (*cur_fun) ();
    struct dfu_status_s* next_op;
    uint8_t* buf_rx;
    uint8_t* buf_tx;
};

struct device* uart_dev;
struct flash_img_context ctx;
//// Structs and enums area end ////

//// Function  area begin ////
void uart_config(const char* dev_name, uint32_t baudrate,
                 uint8_t data_bits, uint8_t flow_ctrl, uint8_t parity, uint8_t stop_bits);
void flash_config(uint8_t area);
void load_image(dfu_status_t* status);
void sync(dfu_status_t* status);

static void event_cb(const struct device* dev, struct uart_event* evt, void* user_data);
//// Function  area end ////

void main(void) {
    uart_config("UART_3",115200, UART_CFG_DATA_BITS_8, UART_CFG_FLOW_CTRL_NONE,
                UART_CFG_PARITY_NONE, UART_CFG_STOP_BITS_1);
    flash_config(FLASH_AREA_ID(image_1));
    dfu_status_t status = STATUS_IDLE;
    uart_callback_set(uart_dev, event_cb, &status);
    int err = uart_rx_enable(uart_dev, buf_rx, MAX_DATA_LEN_RX/2, TIMEOUT);
    if(err < 0) {
        printk(" Error uart_rx_enable. Error code: %d\n", err);
        return;
    }
    while(1) {
        switch (status) {
            case STATUS_FIRMWARE_RECV:
                while (status == STATUS_FIRMWARE_RECV) {
                    k_msleep(10);
                };
                break;
            case STATUS_END_FIRMWARE:
                while (status == STATUS_END_FIRMWARE) {
                    k_msleep(10);
                };
                break;
            case STATUS_LOAD_FIRMWARE:
                load_image(&status);
            case STATUS_SYNC:
                sync(&status);
                while (status == STATUS_SYNC) {
                    k_msleep(10);
                };
                break;
            case STATUS_IDLE:
                while (status == STATUS_IDLE) {
                    k_msleep(10);
                }
                break;
        }
    }
}

void uart_config(const char* dev_name, uint32_t baudrate, uint8_t data_bits,
                 uint8_t flow_ctrl, uint8_t parity, uint8_t stop_bits) {
    int ret;
    uart_dev = device_get_binding(dev_name);
    struct uart_config uart_cfg = {
            .baudrate = baudrate,
            .data_bits = data_bits,
            .flow_ctrl = flow_ctrl,
            .parity = parity,
            .stop_bits = stop_bits
    };
    ret = uart_configure(uart_dev, &uart_cfg);
    if(ret < 0) {
        printk(" Error uart_configure %d\n", ret);
        return;
    }
}

void flash_config(uint8_t area) {
    int err;
    err = flash_img_init_id(&ctx, area);
    if (err < 0) {
        printk(" Error flash_img_init. Error code: %d\n", err);
        return;
    }
}

void load_image(dfu_status_t* status) {
    static unsigned int count = 0;
    int err;
    size_t len;
    if ( !strncmp(buf_rx, "End\n", strlen("End\n")) ) {
        *status = STATUS_END_FIRMWARE;
        return;
    }
    err = flash_img_buffered_write(&ctx, buf_rx, MAX_DATA_LEN_RX/2, true);
    if (err < 0) {
        printk(" Error flash_img_buffered_write. Error code: %d\n", err);
        return;
    }
    len = flash_img_bytes_written(&ctx);
    printk(" %u. Bytes written: %d\n", count, len);
//    err = flash_read(ctx.stream.fdev, ctx.stream.offset, &buf_rx, MAX_DATA_LEN);
//    if (err < 0) {
//        printk(" Error flash_read. Error code: %d\n", err);
//    }
//    printk(" Reading data: %s\n", buf_rx);
//    while(1);
}

static void event_cb(const struct device* dev, struct uart_event* evt, void* user_data) {
    dfu_status_t * status;
    status = (dfu_status_t *)user_data;
    switch (evt->type) {
        case UART_TX_ABORTED:
            break;
        case UART_TX_DONE:
            switch (*status) {
                case STATUS_SYNC:
                    *status = STATUS_FIRMWARE_RECV;
                    uart_rx_enable(dev, buf_rx, MAX_DATA_LEN_RX/2, TIMEOUT);
                    break;
                case STATUS_IDLE:
                case STATUS_FIRMWARE_RECV:
                case STATUS_LOAD_FIRMWARE:
                case STATUS_END_FIRMWARE:
                    break;
            }
            break;
        case UART_RX_BUF_RELEASED:
            break;
        case UART_RX_BUF_REQUEST:
            break;
        case UART_RX_DISABLED:
            break;
        case UART_RX_RDY:
            uart_rx_disable(dev);
            switch(*status) {
                case STATUS_IDLE:
                    *status = STATUS_SYNC;
                    break;
                case STATUS_FIRMWARE_RECV:
                    *status = STATUS_LOAD_FIRMWARE;
                    break;
                case STATUS_SYNC:
                case STATUS_LOAD_FIRMWARE:
                case STATUS_END_FIRMWARE:
                    break;
            }
            break;
        case UART_RX_STOPPED:
            break;
    }
}

void sync(dfu_status_t* status) {
    int err = 0;
    if ( !strncmp(buf_rx, "Start\n", strlen("Start\n")) ) {
        memset(&buf_rx[0], 0, MAX_DATA_LEN_RX);
        printk(" Transfer started");
        strcpy(buf_tx, "Start\n");
        err = uart_tx(uart_dev, buf_tx, strlen("Start\n"), TIMEOUT);
    }
    else if (*status == STATUS_LOAD_FIRMWARE) {
        *status = STATUS_SYNC;
        strcpy(buf_tx, "Next\n");
        err = uart_tx(uart_dev, buf_tx, strlen("Next\n"), SYS_FOREVER_MS);
    }
    if (err < 0) {
        printk(" Error uart_tx. Error code: %d\n", err);
    }
}