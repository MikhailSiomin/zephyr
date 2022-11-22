/*
* Copyright (c) 2022, STRIM, ALC
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>


void main(void) 
{
	printf("\nFLEXIO SPI testing\n");
	printf(  "==================\n");
	const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(flexio3_spi0));

	if (!device_is_ready(spi_dev)) {
		printk("spi device not ready.\n");
	    return;
	}

	printf("device %s is ready", spi_dev->name);

	static const struct spi_cs_control cs = {
		.gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(flexio3_spi0), cs_gpios, 0)
	};
	const static struct spi_dt_spec bus = {
		.bus = DEVICE_DT_GET(DT_NODELABEL(flexio3_spi0)),
		.config = {
			.frequency = 1000000,
			.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_LSB | SPI_WORD_SET(8),
			.slave = 0,
			.cs = &cs
		}
	};

	if (!spi_is_ready(&bus)) {
		printk("spi device not ready.\n");
	    return;
	}

	while (true) {
		uint8_t buffer_tx[4] = { 0x12, 0xF1, 0x0, 0x0 };
		uint8_t buffer_rx[4] = { 0x0, 0x0, 0xFF, 0xFF };

		const struct spi_buf tx_buf[2] = {
			{
				.buf = buffer_tx,
				.len = 1
			},
			{
				.buf = &buffer_tx[1],
				.len = 3,
			}
		};
		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		const struct spi_buf rx_buf[2] = {
			{
				.buf = NULL,
				.len = 1
			},
			{
				.buf = buffer_rx,
				.len = 3,
			}
		};
		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		int ret = spi_transceive_dt(&bus, &tx, &rx);
		if (ret) {
		 	k_sleep(K_MSEC(2000));
		}
	}
}
