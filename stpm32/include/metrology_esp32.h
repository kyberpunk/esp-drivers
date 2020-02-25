/*
 *  Copyright (c) 2018, Vit Holasek.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COMPONENTS_METROLOGY_INCLUDE_METROLOGY_ESP32_H_
#define COMPONENTS_METROLOGY_INCLUDE_METROLOGY_ESP32_H_

#include "metrology.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"

#define UART_TX_TIMEOUT 100
#define SPI_PORT_SPEED 10000000

typedef struct {
	uart_port_t uart_port;
	gpio_num_t txd_pin;
	gpio_num_t rxd_pin;
	uint8_t installed;
} UART_Instance_t;

typedef struct uart_handle_data_tag {
	UART_Instance_t *uart;
} uart_handle_data;

typedef struct {
	spi_host_device_t spi_host;
	gpio_num_t mosi_pin;
	gpio_num_t miso_pin;
	gpio_num_t sclk_pin;
	uint8_t initialized;
} SPI_Instance_t;

typedef struct spi_handle_data_tag {
	spi_device_handle_t handle;
	SPI_Instance_t *spi;
	uint8_t added;
} spi_handle_data;

typedef struct pin_handle_data_tag {
	gpio_num_t cs_pin;
	gpio_num_t syn_pin;
	gpio_num_t en_pin;
} pin_handle_data;

/**
 * Configure platform dependent peripherals for STPM3x control.
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[in]   com_port      COM port configuration.
 * @param[in]   spi_port      SPI port configuration.
 *
 */
void metrology_esp32_init_uart(METRO_NB_Device_t device_id, uart_handle_data *uart_handle, pin_handle_data *pin_handle);
void metrology_esp32_init_spi(METRO_NB_Device_t device_id, spi_handle_data *spi_handle, pin_handle_data *pin_handle);

#endif /* COMPONENTS_METROLOGY_INCLUDE_METROLOGY_ESP32_H_ */
