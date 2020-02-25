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

#ifndef COMPONENTS_METROLOGY_INCLUDE_METROLOGY_PLATFORM_H_
#define COMPONENTS_METROLOGY_INCLUDE_METROLOGY_PLATFORM_H_

#include "st_types.h"
#include "metrology.h"

typedef enum
{
  MET_GPIO_PIN_RESET=0,
  MET_GPIO_PIN_SET=1
} MET_GPIO_PIN_State_t;

typedef enum
{
	MET_PORT_CS=0,
	MET_PORT_SYN,
	MET_PORT_EN
} MET_PORT_Pin_t;

typedef enum
{
	MET_LOG_NONE=0,
	MET_LOG_ERROR,
	MET_LOG_WARN,
	MET_LOG_INFO,
	MET_LOG_DEBUG,
	MET_LOG_VERBOSE
} MET_LogLevel_t;

typedef uint8_t MET_UART_id;
typedef uint8_t MET_COM_PORT_id;
typedef uint8_t MET_SPI_PORT_id;

/**
 * This function initializes platform dependent modules required by metrology application.
 *
 */
void metrology_platform_init();

/**
 * Configure metrology application UART interface.
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[in]   in_baudrate   UART interface baudrate.
 *
 */
void metrology_platform_uart_config(uart_handle uart_handle, uint32_t in_baudrate);

/**
 * Configure metrology application SPI interface.
 *
 * @param[in]   device_id     Metrology device ID.
 *
 */
void metrology_platform_spi_config(spi_handle spi_handle);

/**
 * Delay in microseconds.
 *
 * @param[in]   time          Time in microseconds.
 *
 */
void metrology_platform_wait_microseconds(uint32_t time);

/**
 * Set value of STPM3x controlling GPIO pin (EN, CS, SYN).
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[in]   pin           STPM3x pin.
 * @param[in]   state         Logical state.
 *
 */
void metrology_platform_gpio_write(pin_handle pin_handle, MET_PORT_Pin_t pin, MET_GPIO_PIN_State_t state);

/**
 * Transmit data by UART interface.
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[in]   data          Data buffer to be transmitted.
 * @param[in]   size          Data size.
 * @param[in]   timeout       Transmission timeout.
 *
 * @retval                    Returns number of bytes transmitted.
 *
 */
uint32_t metrology_platform_uart_transmit(uart_handle uart_handle, uint8_t* data, uint32_t size, uint16_t timeout);

/**
 * Receive data by UART interface.
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[out]  data          Data buffer to receive data.
 * @param[in]   size          Data size to be received.
 * @param[in]   timeout       Reception timeout.
 *
 * @retval                    Returns number of bytes received.
 *
 */
uint32_t metrology_platform_uart_receive(uart_handle uart_handle, uint8_t* data, uint32_t size, uint16_t timeout);

/**
 * Transmit and receive data by SPI interface in full duplex.
 *
 * @param[in]   device_id     Metrology device ID.
 * @param[out]  data_out      Data to transmit.
 * @param[in]   data_in       Data buffer to receive.
 * @param[in]   size          Size of data to transmit.
 * @param[in]   timeout       Transmission timeout.
 *
 * @retval                    Returns number of bytes transmitted.
 *
 */
uint32_t metrology_platform_spi_transmit_receive(spi_handle spi_handle, uint8_t* data_out, uint8_t* data_in, uint32_t size, uint16_t timeout);

/**
 * Platform log function.
 *
 * @param[in]   level         Log level.
 * @param[in]   format        Format to print.
 *
 */
void metrology_platform_log(MET_LogLevel_t level, const char *format, ...);

#endif /* COMPONENTS_METROLOGY_INCLUDE_METROLOGY_PLATFORM_H_ */
