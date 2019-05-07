
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_spi_flash.h"
#include "soc/cpu.h"
#include "py/obj.h"

#include "machpin.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "gpio.h"
#include "machpin.h"
#include "pins.h"
#include "py/mpconfig.h"
#include "py/mphal.h"

#define UART1_TXD  (GPIO_NUM_4)
#define UART1_RXD  (GPIO_NUM_15)
#define UART1_RTS  (UART_PIN_NO_CHANGE)
#define UART1_CTS  (UART_PIN_NO_CHANGE)

#define UART0_TXD  (GPIO_NUM_1)
#define UART0_RXD  (GPIO_NUM_3)
#define UART0_RTS  (UART_PIN_NO_CHANGE)
#define UART0_CTS  (UART_PIN_NO_CHANGE)

STATIC const mp_obj_t iso_prog_def_pin[2] = {&PIN_MODULE_P12, &PIN_MODULE_P11}; //IN2 & LED_OUT

STATIC const mp_obj_t powerpilot_def_pin[5] = {&PIN_MODULE_P8,&PIN_MODULE_P16,&PIN_MODULE_P9,&PIN_MODULE_P3,&PIN_MODULE_P4}; // DISPLAY_ON, POWER FAIL, FORCED_CALIBRATION_MODE, DISPLAY_TX,DISPLAY_RX

#define BUF_SIZE (1024)

STATIC IRAM_ATTR void UARTRxCallback(int uart_id, int rx_byte) {
}

static void init_uart(int num, int baudrate, int rxpin, int txpin )
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(num, &uart_config);
    uart_set_pin(num, txpin, rxpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(num, BUF_SIZE * 2, 0, 0, NULL, 0, UARTRxCallback);
}

void set_display_state(bool state)
{
    pin_obj_t *display_enable = pin_find(powerpilot_def_pin[0]);
    pin_config (display_enable, -1, -1, GPIO_MODE_OUTPUT, MACHPIN_PULL_UP, 1);
    display_enable->value = state;
    pin_set_value(display_enable);
}

bool get_power_fail_state()
{
    pin_obj_t *pf = pin_find(powerpilot_def_pin[1]);
    pin_config (pf, -1, -1, GPIO_MODE_INPUT, MACHPIN_PULL_UP, 1);
    vTaskDelay (10 / portTICK_PERIOD_MS);
    uint32_t val_pf = pin_get_value(pf);
    if(val_pf == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool is_forced_calibration_mode()
{
    pin_obj_t *pf = pin_find(powerpilot_def_pin[2]);
    pin_config (pf, -1, -1, GPIO_MODE_INPUT, MACHPIN_PULL_UP, 1);
    vTaskDelay (11 / portTICK_PERIOD_MS);
    uint32_t val_pf = pin_get_value(pf);
    vTaskDelay (1 / portTICK_PERIOD_MS);
    uint32_t val_pf2 = pin_get_value(pf);
    vTaskDelay (3 / portTICK_PERIOD_MS);
    uint32_t val_pf3 = pin_get_value(pf);
    if((val_pf == 0) && (val_pf2 == 0) && (val_pf3 == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void disable_display_uart_and_wait()
{
    pin_obj_t *pf1 = pin_find(powerpilot_def_pin[3]);
    pin_config (pf1, -1, -1, GPIO_MODE_INPUT, MACHPIN_PULL_NONE, 1);
    pin_obj_t *pf2 = pin_find(powerpilot_def_pin[4]);
    pin_config (pf2, -1, -1, GPIO_MODE_INPUT, MACHPIN_PULL_NONE, 1);
    pin_obj_t *led_out = pin_find(iso_prog_def_pin[1]);
    pin_config (led_out, -1, -1, GPIO_MODE_OUTPUT, MACHPIN_PULL_DOWN, 0);
    led_out->value = 1;
    pin_set_value(led_out);
    set_display_state(true);
    while (1) { }  
}


bool get_ens_calibration_status ()
{
    pin_obj_t *in2 = pin_find(iso_prog_def_pin[0]);
    pin_obj_t *led_out = pin_find(iso_prog_def_pin[1]);

    pin_config (in2, -1, -1, GPIO_MODE_INPUT, MACHPIN_PULL_UP, 1);
    pin_config (led_out, -1, -1, GPIO_MODE_OUTPUT, MACHPIN_PULL_DOWN, 0);

    // to get into calibration mode IN2 needs to toggle twice, follwing the LED_OUT pattern
    // set GPIO22 to 1
    led_out->value = 1;
    pin_set_value(led_out);

    vTaskDelay (250 / portTICK_PERIOD_MS);
    // ckeck status
    uint32_t val_in2_step1 = pin_get_value(in2);
    vTaskDelay (50/ portTICK_PERIOD_MS);


    // set GPIO22 to 0
    led_out->value = 0;
    pin_set_value(led_out);
    vTaskDelay (250 / portTICK_PERIOD_MS);
    uint32_t val_in2_step2 = pin_get_value(in2);
    vTaskDelay (50/ portTICK_PERIOD_MS);

    // set GPIO22 to 1
    led_out->value = 1;
    pin_set_value(led_out);
    vTaskDelay (250 / portTICK_PERIOD_MS);
    // ckeck status
    uint32_t val_in2_step3 = pin_get_value(in2);
    vTaskDelay (50/ portTICK_PERIOD_MS);


    // set GPIO22 to 0
    led_out->value = 0;
    pin_set_value(led_out);
    vTaskDelay (250 / portTICK_PERIOD_MS);
    uint32_t val_in2_step4 = pin_get_value(in2);
    vTaskDelay (50/ portTICK_PERIOD_MS);

   // the last 2 steps are inverse of LED OUT to avoid a short between LED_OUT and IN2 putting the device into calib mode! 
    if(val_in2_step1 == 1 && val_in2_step2 ==0 && val_in2_step3 ==0 && val_in2_step4 ==1)
    {
        led_out->value = 1;
        pin_set_value(led_out);
        return true;
    }
    else
    {
        led_out->value = 0;
        pin_set_value(led_out);
        return false;
    }

}



void run_ens_calibration ( )
{
    /* let the user that we are in calibration mode */
    pin_obj_t *led_out = pin_find(iso_prog_def_pin[1]);
    pin_obj_t *display_enable = pin_find(powerpilot_def_pin[0]);


    pin_config (display_enable, -1, -1, GPIO_MODE_OUTPUT, MACHPIN_PULL_UP, 1);
    pin_config (led_out, -1, -1, GPIO_MODE_OUTPUT, MACHPIN_PULL_DOWN, 0);
    // set GPIO22 to 1
    led_out->value = 1;
    pin_set_value(led_out);
    // enable the display
    display_enable->value = 1;
    pin_set_value(display_enable);

    /* init both uart @ 9600 baud */
    init_uart(UART_NUM_0,9600,UART0_RXD, UART0_TXD);
    init_uart(UART_NUM_1,9600,UART1_RXD, UART1_TXD);


    // Configure a temporary buffer for the incoming data
    uint8_t *data0 = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *data1 = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART 0
        int len0 = uart_read_bytes(UART_NUM_0, data0, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART 1

        uart_write_bytes(UART_NUM_1, (const char *) data0, len0);

        int len1 = uart_read_bytes(UART_NUM_1, data1, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART 1

        uart_write_bytes(UART_NUM_0, (const char *) data1, len1);

        // ISO prog v3 patch -> remove TX 0data in RX0 buffer
        if(len1 > 0)
        {
            //wait to make sure all byte have been written to the RS485 port
            if(ESP_OK == uart_wait_tx_done(UART_NUM_0, 250 / portTICK_PERIOD_MS))  // max of 250 char
            {
                //read excalty len1 bytes out of the buffer and leave
                uart_read_bytes(UART_NUM_0, data0, len1, 10 / portTICK_RATE_MS);
            }
        }
    }

    // set GPIO22 to 0 // should not get there
    led_out->value = 0;
    pin_set_value(led_out);
}
