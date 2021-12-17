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
#include <drivers/flash.h>
#include <drivers/uart.h>
//#include <storage/stream_flash.h>

#define MAX_DATA_LEN_RX 256
#define MAX_DATA_LEN_TX 10
#define TIMEOUT 0

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(serial_dfu);

uint8_t buf_tx[MAX_DATA_LEN_TX] = {};
uint8_t buf_rx[MAX_DATA_LEN_RX] = {};

//// Structs and enums area begin ////
typedef enum Status_e {
    STATUS_SYNC = 0,
    STATUS_FIRMWARE_RECV,
    STATUS_LOAD_FIRMWARE,
    STATUS_IDLE,
    STATUS_END_FIRMWARE
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
void load_image(void);
void pc_communication(struct dfu_status_s* status);
void start_transfer(void);

static void event_cb(const struct device* dev, struct uart_event* evt, void* user_data);
//// Function  area end ////

void main(void) {
    uart_config("UART_3",115200, UART_CFG_DATA_BITS_8, UART_CFG_FLOW_CTRL_NONE,
                UART_CFG_PARITY_NONE, UART_CFG_STOP_BITS_1);
//    flash_config(FLASH_AREA_ID(image_1));
    dfu_status_t status;
    uart_callback_set(uart_dev, event_cb, &status);
    int err = uart_rx_enable(uart_dev, buf_rx, MAX_DATA_LEN_RX/2, TIMEOUT);
    if(err < 0) {
        printk(" Error uart_rx_enable. Error code: %d\n", err);
        return;
    }
    while(1);
//    len = flash_img_bytes_written(&ctx);
//    printk(" Bytes written: %d\n", len);
//    err = flash_read(ctx.stream.fdev, ctx.stream.offset, &buf_rx, MAX_DATA_LEN);
//    if (err < 0) {
//        printk(" Error flash_read. Error code: %d\n", err);
//    }
//    printk(" Reading data: %s\n", buf_rx);
//    while(1);
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
    struct flash_area* ar;
    uint8_t area_id = area;
    int err = flash_area_open(area_id, &ar);
    if (err < 0) {
        printk(" Error flash_area_open. Error code: %d\n", err);
        return;
    }
    ctx.flash_area = ar;
    err = flash_img_init(&ctx);
    if (err < 0) {
        printk(" Error flash_img_init. Error code: %d\n", err);
        return;
    }
}

void load_image(void) {
    while(1) {
        int err = uart_rx_enable(uart_dev, buf_rx, MAX_DATA_LEN_RX/2, SYS_FOREVER_MS);
        if (err < 0) {
            printk(" Error uart_rx_enable. Error code: %d\n", err);
            return;
        }
        uart_rx_disable(uart_dev);
        err = flash_img_buffered_write(&ctx, buf_rx, MAX_DATA_LEN_RX/2, true);
        if (err < 0) {
            printk(" Error flash_img_buffered_write. Error code: %d\n", err);
            return;
        }
        strcpy(buf_tx, "Next\n");
        err = uart_tx(uart_dev, buf_tx, strlen("Next\n"), SYS_FOREVER_MS);
    }
}

static void event_cb(const struct device* dev, struct uart_event* evt, void* user_data) {
    dfu_status_t * status;
    status = (dfu_status_t *)user_data;
    int err;
    switch (evt->type) {
        case UART_TX_ABORTED:
            break;
        case UART_TX_DONE:
            switch (*status) {
                case STATUS_IDLE:
                    break;
                case STATUS_SYNC:
                    break;
                case STATUS_FIRMWARE_RECV:
                    break;
                case STATUS_LOAD_FIRMWARE:
                    break;
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
                case STATUS_SYNC:
//                    if( (status->buf_rx) && !strncmp( (status->buf_rx), "Start\n", strlen("Start\n") ) ) {
//                        printk(" Transfer started");
//
//                        err = uart_tx(dev, status->buf_rx, strlen("Start\n"), SYS_FOREVER_MS);
//                        if(err < 0) {
//                            printk(" Error uart_tx. Error code: %d\n", err);
//                            return;
//                        }
//
////                        memset(&buf_rx[0], 0, MAX_DATA_LEN_RX);
//                        //// Start receive firmware ////
//                        err = uart_rx_enable(dev, status->buf_rx, MAX_DATA_LEN_RX/2, TIMEOUT);
//                        if(err < 0) {
//                            printk(" Error uart_rx. Error code: %d\n", err);
//                            return;
//                        }
//                        status->cur_status = STATUS_FIRMWARE_RECV;  // change current status
//                    }
                    break;
                case STATUS_FIRMWARE_RECV:
//                    status->cur_status = STATUS_LOAD_FIRMWARE;
                    break;
                case STATUS_LOAD_FIRMWARE:
                    break;
                case STATUS_END_FIRMWARE:
                    break;
            }
            break;
        case UART_RX_STOPPED:
            break;
    }
}

void pc_communication(struct dfu_status_s* status) {
    int err;
    switch(status->cur_status) {
        case STATUS_SYNC:
            uart_rx_enable(uart_dev, buf_rx, MAX_DATA_LEN_RX/2, TIMEOUT);
            break;
        case STATUS_FIRMWARE_RECV:
            break;
        case STATUS_IDLE:
            break;
        case STATUS_END_FIRMWARE:
            break;
    }
}

void start_transfer(void) {

}