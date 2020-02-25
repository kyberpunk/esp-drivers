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

#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "metrology_platform.h"
#include "metrology_esp32.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"

#define TAG "metrology_platform"

void metrology_platform_init() {
;
}

void metrology_platform_uart_config(struct uart_handle_data_tag *uart_handle, uint32_t in_baudrate) {
	if (uart_handle != NULL) {
		UART_Instance_t *uart = uart_handle->uart;
		uart_config_t uart_config;
		memset(&uart_config, 0, sizeof(uart_config));
		uart_config.baud_rate = in_baudrate;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

		ESP_LOGI(TAG, "uart_param_config");
		esp_err_t result = uart_param_config(uart->uart_port, &uart_config);
		if (result == ESP_OK) {
			ESP_LOGI(TAG, "uart_param_config Succeed");
		} else {
			ESP_LOGE(TAG, "uart_param_config Failed");
		}
		ESP_LOGI(TAG, "uart_driver_install");
		if (uart->installed == 0) {
			uart_set_pin(uart->uart_port,
					uart->txd_pin,
					uart->rxd_pin, UART_PIN_NO_CHANGE,
					UART_PIN_NO_CHANGE);
			esp_err_t result = uart_driver_install(
					uart->uart_port, 256, 256, 0,
					NULL, 0);
			if (result == ESP_OK) {
				ESP_LOGI(TAG,	"uart_driver_install Succeed");
			} else {
				ESP_LOGE(TAG, "uart_driver_install Failed");
			}
			uart->installed = 1;
		}
	} else {
		ESP_LOGE(TAG, "metrology_platform_uart_config Failed. uart_handle is NULL.");
	}
}

void metrology_platform_spi_config(struct spi_handle_data_tag *spi_handle) {
	if (spi_handle != NULL) {
		if (spi_handle->spi->initialized == 1) {
			spi_bus_remove_device(spi_handle->handle);
			spi_bus_free(spi_handle->spi->spi_host);
			spi_handle->spi->initialized = 0;
		}

		spi_bus_config_t config;
		memset(&config, 0, sizeof(config));
		config.mosi_io_num = spi_handle->spi->mosi_pin;
		config.miso_io_num = spi_handle->spi->miso_pin;
		config.sclk_io_num = spi_handle->spi->sclk_pin;
		config.quadhd_io_num = -1;
		config.quadwp_io_num = -1;
		config.max_transfer_sz = 4094;

		spi_device_interface_config_t device_config;
		memset(&device_config, 0, sizeof(device_config));
		device_config.mode = 3; // CPOL=1 CPHA=1
		device_config.clock_speed_hz = SPI_PORT_SPEED;
		device_config.queue_size = 7;
		device_config.spics_io_num = -1;
		device_config.input_delay_ns = 50;
		device_config.flags = SPI_DEVICE_NO_DUMMY;

		esp_err_t result = spi_bus_initialize(spi_handle->spi->spi_host, &config, 0);
		if (result == ESP_OK) {
			ESP_LOGI(TAG, "spi_bus_initialize Succeed");
		} else {
			ESP_LOGE(TAG, "spi_bus_initialize Failed");
		}

		result = spi_bus_add_device(spi_handle->spi->spi_host, &device_config, &spi_handle->handle);
		if (result == ESP_OK) {
			ESP_LOGI(TAG, "spi_bus_add_device Succeed");
		} else {
			ESP_LOGE(TAG, "spi_bus_add_device Failed");
		}

		spi_handle->spi->initialized = 1;
	} else {
		ESP_LOGE(TAG, "metrology_platform_spi_config Failed. spi_handle is NULL.");
	}
}

void metrology_platform_wait_microseconds(uint32_t time) {
	//Delay in us, not recommended in FreeRTOS
	ets_delay_us(time);

	//Delay in ms or minimal delay
	//const TickType_t xTicks = (time / portTICK_PERIOD_MS) / 1000;
	//vTaskDelay(xTicks > 0 ? xTicks : 1);
}

void metrology_platform_gpio_write(struct pin_handle_data_tag *pin_handle, MET_PORT_Pin_t pin, MET_GPIO_PIN_State_t state) {
	if (pin_handle != NULL) {
		gpio_num_t gpio_pin;
		switch (pin) {
		case MET_PORT_CS:
			gpio_pin = pin_handle->cs_pin;
			break;
		case MET_PORT_EN:
			gpio_pin = pin_handle->en_pin;
			break;
		case MET_PORT_SYN:
			gpio_pin = pin_handle->syn_pin;
			break;
		default:
			return;
		}
		gpio_set_level(gpio_pin, state);
	}
}

uint32_t metrology_platform_uart_transmit(struct uart_handle_data_tag *uart_handle, uint8_t* data, uint32_t size, uint16_t timeout) {
	if (uart_handle != NULL) {
		uart_port_t uart_port = uart_handle->uart->uart_port;
		const int txBytes = uart_write_bytes(uart_port, (const char*)data, size);
		// Wait for tx done to avoid overrun
		esp_err_t result = uart_wait_tx_done(uart_port, UART_TX_TIMEOUT / portTICK_RATE_MS);
		if (result == ESP_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "UART Tx timeout!");
		}
		return txBytes;
	}
	return 0;
}

uint32_t metrology_platform_uart_receive(struct uart_handle_data_tag *uart_handle, uint8_t* data, uint32_t size, uint16_t timeout) {
	if (uart_handle != NULL) {
		const int rxBytes = uart_read_bytes(uart_handle->uart->uart_port, data, size, timeout / portTICK_RATE_MS);
		if (rxBytes <= 0) {
			ESP_LOGE(TAG, "Read no bytes!");
		}
		return rxBytes;
	}
	return 0;
}

uint32_t metrology_platform_spi_transmit_receive(struct spi_handle_data_tag *spi_handle, uint8_t* data_out, uint8_t* data_in, uint32_t size, uint16_t timeout) {
	if (spi_handle != NULL) {
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.length = size * 8; // size is in bytes, transaction length is in bits.
		t.tx_buffer = data_out;
		t.rx_buffer = data_in;
		t.user = (void*) 0;
		// Blocking SPI communication is faster (required for HDO)
		esp_err_t result = spi_device_polling_transmit(spi_handle->handle, &t);
		if (result != ESP_OK || t.rxlength == 0) {
			ESP_LOGE(TAG, "Read no bytes!");
			metrology_platform_log(MET_LOG_INFO ,"Read no bytes!");
		}
		return t.rxlength;
	}
	return 0;
}

void metrology_platform_log(MET_LogLevel_t level, const char * format, ...) {
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	printf("\n");
	va_end(args);
}

static void init_pin_handle_data(pin_handle_data *pin_handle) {
	ESP_LOGI(TAG, "init_pin_handle_data");
	uint64_t bitmask = ((1ULL << pin_handle->cs_pin) | (1ULL << pin_handle->en_pin) | (1ULL << pin_handle->syn_pin));
	gpio_config_t conf = {
		.intr_type = GPIO_PIN_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pull_down_en = 0,
		.pin_bit_mask = bitmask,
		.pull_up_en = 0
	};
	gpio_config(&conf);
}

void metrology_esp32_init_uart(METRO_NB_Device_t device_id, struct uart_handle_data_tag *uart_handle, pin_handle_data *pin_handle) {
	if (uart_handle == NULL || pin_handle == NULL || device_id >= NB_MAX_DEVICE) {
		return;
	}
	ESP_LOGI(TAG, "metrology_esp32_init_uart");
	init_pin_handle_data(pin_handle);
	Metro_Set_Uart_Handle(device_id, uart_handle);
	Metro_Set_Pin_Handle(device_id, pin_handle);
}

void metrology_esp32_init_spi(METRO_NB_Device_t device_id, struct spi_handle_data_tag *spi_handle, pin_handle_data *pin_handle) {
	if (spi_handle == NULL || pin_handle == NULL || device_id >= NB_MAX_DEVICE) {
		return;
	}
	ESP_LOGI(TAG, "metrology_esp32_init_spi");
	init_pin_handle_data(pin_handle);
	Metro_Set_Spi_Handle(device_id, spi_handle);
	Metro_Set_Pin_Handle(device_id, pin_handle);
}
