// SPDX-License-Identifier: GPL-2.0
/*
 * Lattice MachXO2 Slave SPI Driver
 *
 * Manage Lattice FPGA firmware that is loaded over SPI using
 * the slave serial configuration interface.
 *
 * Copyright (C) 2018 Paolo Pisati <p.pisati@gmail.com>
 */

#include <linux/delay.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

/* MachXO2 Programming Guide - sysCONFIG Programming Commands */
#define IDCODE_PUB		{0xe0, 0x00, 0x00, 0x00}
#define ISC_ENABLE		{0xc6, 0x08, 0x00, 0x00}
#define ISC_ERASE		{0x0e, 0x04, 0x00, 0x00}
#define ISC_PROGRAMDONE		{0x5e, 0x00, 0x00, 0x00}
#define LSC_INITADDRESS		{0x46, 0x00, 0x00, 0x00}
#define LSC_PROGINCRNV		{0x70, 0x00, 0x00, 0x01}
#define LSC_READ_STATUS		{0x3c, 0x00, 0x00, 0x00}
#define LSC_REFRESH		{0x79, 0x00, 0x00}

/*
 * Max CCLK in Slave SPI mode according to 'MachXO2 Family Data
 * Sheet' sysCONFIG Port Timing Specifications (3-36)
 */
#define MACHXO2_MAX_SPEED		66000000

#define MACHXO2_LOW_DELAY_USEC		5
#define MACHXO2_HIGH_DELAY_USEC		200
#define MACHXO2_REFRESH_USEC		4800
#define MACHXO2_LOOP_FOREVER		0
#define MACHXO2_MAX_BUSY_LOOP		128
#define MACHXO2_MAX_REFRESH_LOOP	16

#define MACHXO2_PAGE_SIZE		16
#define MACHXO2_BUF_SIZE		(MACHXO2_PAGE_SIZE + 4)

/* Status register bits, errors and error mask */
#define BUSY	12
#define DONE	8
#define DVER	27
#define ENAB	9
#define ERRBITS	23
#define ERRMASK	7
#define FAIL	13

#define ENOERR	0 /* no error */
#define EID	1
#define ECMD	2
#define ECRC	3
#define EPREAM	4 /* preamble error */
#define EABRT	5 /* abort error */
#define EOVERFL	6 /* overflow error */
#define ESDMEOF	7 /* SDM EOF */

struct machxo2_spi {
	struct fpga_manager *mgr;
	struct gpio_desc *programn;
	struct gpio_desc *initn;
};

static inline u8 get_err(unsigned long *status)
{
	return (*status >> ERRBITS) & ERRMASK;
}

static int get_status(struct spi_device *spi, unsigned long *status)
{
	struct spi_message msg;
	struct spi_transfer rx, tx;
	static const u8 cmd[] = LSC_READ_STATUS;
	int ret;

	memset(&rx, 0, sizeof(rx));
	memset(&tx, 0, sizeof(tx));
	tx.tx_buf = cmd;
	tx.len = sizeof(cmd);
	rx.rx_buf = status;
	rx.len = 4;
	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		return ret;

	*status = be32_to_cpu(*status);

	return 0;
}

static const char *get_err_string(u8 err)
{
	switch (err) {
	case ENOERR:	return "No Error";
	case EID:	return "ID ERR";
	case ECMD:	return "CMD ERR";
	case ECRC:	return "CRC ERR";
	case EPREAM:	return "Preamble ERR";
	case EABRT:	return "Abort ERR";
	case EOVERFL:	return "Overflow ERR";
	case ESDMEOF:	return "SDM EOF";
	}

	return "Default switch case";
}

static void dump_status_reg(unsigned long *status)
{
	pr_debug("machxo2 status: 0x%08lX - done=%d, cfgena=%d, busy=%d, fail=%d, devver=%d, err=%s\n",
		 *status, test_bit(DONE, status), test_bit(ENAB, status),
		 test_bit(BUSY, status), test_bit(FAIL, status),
		 test_bit(DVER, status), get_err_string(get_err(status)));
}

static int wait_until_not_busy(struct spi_device *spi, int max_loops)
{
	unsigned long status;
	int ret, loop = 0;

	do {
		ret = get_status(spi, &status);
		if (ret) {
			dump_status_reg(&status);
			return ret;
		}
		loop++;
		if (MACHXO2_LOOP_FOREVER != max_loops && loop >= max_loops) {
			dump_status_reg(&status);
			return -EBUSY;
		}
	} while (test_bit(BUSY, &status));

	return 0;
}

static int machxo2_cleanup(struct fpga_manager *mgr)
{
	struct spi_device *spi = mgr->priv;
	struct spi_message msg;
	struct spi_transfer tx[2];
	static const u8 erase[] = ISC_ERASE;
	static const u8 refresh[] = LSC_REFRESH;
	int ret;

	memset(tx, 0, sizeof(tx));
	spi_message_init(&msg);
	tx[0].tx_buf = &erase;
	tx[0].len = sizeof(erase);
	spi_message_add_tail(&tx[0], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;

	ret = wait_until_not_busy(spi, MACHXO2_MAX_BUSY_LOOP);
	if (ret)
		goto fail;

	spi_message_init(&msg);
	tx[1].tx_buf = &refresh;
	tx[1].len = sizeof(refresh);
	tx[1].delay_usecs = MACHXO2_REFRESH_USEC;
	spi_message_add_tail(&tx[1], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;

	return 0;
fail:
	dev_err(&mgr->dev, "Cleanup failed\n");

	return ret;
}

static enum fpga_mgr_states machxo2_spi_state(struct fpga_manager *mgr)
{
	struct spi_device *spi = mgr->priv;
	unsigned long status;

	get_status(spi, &status);
	if (!test_bit(BUSY, &status) && test_bit(DONE, &status) &&
	    get_err(&status) == ENOERR)
		return FPGA_MGR_STATE_OPERATING;

	return FPGA_MGR_STATE_UNKNOWN;
}

static int machxo2_write_init(struct fpga_manager *mgr,
			      struct fpga_image_info *info,
			      const char *buf, size_t count)
{
	struct spi_device *spi = mgr->priv;
	struct machxo2_spi *machxo2_spi = spi_get_drvdata(spi);
	struct spi_message msg;
	struct spi_transfer tx[3];
	static const u8 enable[] = ISC_ENABLE;
	static const u8 erase[] = ISC_ERASE;
	static const u8 initaddr[] = LSC_INITADDRESS;
	unsigned long status;
	int ret;

	dev_dbg(&mgr->dev, "Starting write initialization\n");

	if ((info->flags & FPGA_MGR_PARTIAL_RECONFIG)) {
		dev_err(&mgr->dev,
			"Partial reconfiguration is not supported\n");
		return -ENOTSUPP;
	}

	if (machxo2_spi->programn) {
		gpiod_set_value(machxo2_spi->programn, 1);
		ndelay(55);
		gpiod_set_value(machxo2_spi->programn, 0);

		if (machxo2_spi->initn) {
			ndelay(100);
			// Wait for initn to go high.
			do {
				ret = gpiod_get_value(machxo2_spi->initn);
				if (ret < 0)
					goto fail;
			} while (!ret);
		} else {
			udelay(150);
		}
	}

	get_status(spi, &status);
	dump_status_reg(&status);
	memset(tx, 0, sizeof(tx));
	spi_message_init(&msg);
	tx[0].tx_buf = &enable;
	tx[0].len = sizeof(enable);
	spi_message_add_tail(&tx[0], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;

	ret = wait_until_not_busy(spi, MACHXO2_MAX_BUSY_LOOP);
	if (ret)
		goto fail;

	get_status(spi, &status);
	dump_status_reg(&status);
	if (test_bit(FAIL, &status)) {
		ret = -1;
		goto fail;
	}

	dev_dbg(&mgr->dev, "Erasing flash\n");

	spi_message_init(&msg);
	tx[1].tx_buf = &erase;
	tx[1].len = sizeof(erase);
	spi_message_add_tail(&tx[1], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;

	ret = wait_until_not_busy(spi, MACHXO2_LOOP_FOREVER);
	if (ret)
		goto fail;

	get_status(spi, &status);
	dump_status_reg(&status);
	if (test_bit(FAIL, &status)) {
		ret = -1;
		goto fail;
	}

	spi_message_init(&msg);
	tx[2].tx_buf = &initaddr;
	tx[2].len = sizeof(initaddr);
	spi_message_add_tail(&tx[2], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;

	get_status(spi, &status);
	dump_status_reg(&status);

	return 0;
fail:
	dev_err(&mgr->dev, "Error during FPGA init: %d\n", ret);

	return ret;
}

static int machxo2_write(struct fpga_manager *mgr, const char *buf,
			 size_t count)
{
	struct spi_device *spi = mgr->priv;
	struct spi_message msg;
	struct spi_transfer tx;
	static const u8 progincr[] = LSC_PROGINCRNV;
	u8 payload[MACHXO2_BUF_SIZE];
	unsigned long status;
	int i, ret;

	dev_dbg(&mgr->dev, "Starting write\n");

	if (count % MACHXO2_PAGE_SIZE != 0) {
		dev_err(&mgr->dev, "Malformed payload.\n");
		return -EINVAL;
	}
	get_status(spi, &status);
	dump_status_reg(&status);
	memcpy(payload, &progincr, sizeof(progincr));
	for (i = 0; i < count; i += MACHXO2_PAGE_SIZE) {
		memcpy(&payload[sizeof(progincr)], &buf[i], MACHXO2_PAGE_SIZE);
		memset(&tx, 0, sizeof(tx));
		spi_message_init(&msg);
		tx.tx_buf = payload;
		tx.len = MACHXO2_BUF_SIZE;
		spi_message_add_tail(&tx, &msg);
		ret = spi_sync(spi, &msg);
		if (ret) {
			dev_err(&mgr->dev, "Error loading the bitstream.\n");
			return ret;
		}

		ret = wait_until_not_busy(spi, MACHXO2_MAX_BUSY_LOOP);
		if (ret) {
			dev_err(&mgr->dev, "FPGA busy while loading the bitstream.\n");
			return ret;
		}
	}
	get_status(spi, &status);
	dump_status_reg(&status);

	return 0;
}

static int machxo2_write_complete(struct fpga_manager *mgr,
				  struct fpga_image_info *info)
{
	struct spi_device *spi = mgr->priv;
	struct spi_message msg;
	struct spi_transfer tx[2];
	static const u8 progdone[] = ISC_PROGRAMDONE;
	static const u8 refresh[] = LSC_REFRESH;
	unsigned long status;
	int ret, refreshloop = 0;

	dev_dbg(&mgr->dev, "Completing write\n");

	memset(tx, 0, sizeof(tx));
	spi_message_init(&msg);
	tx[0].tx_buf = &progdone;
	tx[0].len = sizeof(progdone);
	spi_message_add_tail(&tx[0], &msg);
	ret = spi_sync(spi, &msg);
	if (ret)
		goto fail;
	ret = wait_until_not_busy(spi, MACHXO2_MAX_BUSY_LOOP);
	if (ret)
		goto fail;

	get_status(spi, &status);
	dump_status_reg(&status);
	if (!test_bit(DONE, &status)) {
		machxo2_cleanup(mgr);
		goto fail;
	}

	dev_dbg(&mgr->dev, "Starting refresh\n");

	do {
		spi_message_init(&msg);
		tx[1].tx_buf = &refresh;
		tx[1].len = sizeof(refresh);
		tx[1].delay_usecs = MACHXO2_REFRESH_USEC;
		spi_message_add_tail(&tx[1], &msg);
		ret = spi_sync(spi, &msg);
		if (ret)
			goto fail;

		/* check refresh status */
		get_status(spi, &status);
		dump_status_reg(&status);
		if (!test_bit(BUSY, &status) && test_bit(DONE, &status) &&
		    get_err(&status) == ENOERR)
			break;
		if (++refreshloop == MACHXO2_MAX_REFRESH_LOOP) {
			dev_err(&mgr->dev, "Max refresh retry reached.\n");
			ret = machxo2_cleanup(mgr);
			if (ret)
				goto fail;
			ret = -EBUSY;
			goto fail;
		}
	} while (1);

	get_status(spi, &status);
	dump_status_reg(&status);

	return 0;
fail:
	dev_err(&mgr->dev, "Refresh failed.\n");

	return ret;
}

static const struct fpga_manager_ops machxo2_ops = {
	.state = machxo2_spi_state,
	.write_init = machxo2_write_init,
	.write = machxo2_write,
	.write_complete = machxo2_write_complete,
};

static int machxo2_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct machxo2_spi *machxo2_spi;

	if (spi->max_speed_hz > MACHXO2_MAX_SPEED) {
		dev_err(dev, "Speed is too high\n");
		return -EINVAL;
	}

	machxo2_spi = devm_kzalloc(dev, sizeof(*machxo2_spi), GFP_KERNEL);
	if (!machxo2_spi)
		return -ENOMEM;

	machxo2_spi->programn = devm_gpiod_get_optional(dev, "programn",
		GPIOD_OUT_HIGH);
	if (!machxo2_spi->programn)
		dev_dbg(dev, "Programn pin not set\n");

	machxo2_spi->initn = devm_gpiod_get_optional(dev, "initn", GPIOD_IN);
	if (!machxo2_spi->initn)
		dev_dbg(dev, "Initn pin not set\n");

	machxo2_spi->mgr = devm_fpga_mgr_create(dev,
		"Lattice MachXO2 SPI FPGA Manager", &machxo2_ops, spi);
	if (!machxo2_spi->mgr)
		return -ENOMEM;

	spi_set_drvdata(spi, machxo2_spi);

	return fpga_mgr_register(machxo2_spi->mgr);
}

static int machxo2_spi_remove(struct spi_device *spi)
{
	struct machxo2_spi *machxo2_spi = spi_get_drvdata(spi);

	fpga_mgr_unregister(machxo2_spi->mgr);

	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "lattice,machxo2-slave-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static const struct spi_device_id lattice_ids[] = {
	{ "machxo2-slave-spi", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lattice_ids);

static struct spi_driver machxo2_spi_driver = {
	.driver = {
		.name = "machxo2-slave-spi",
		.of_match_table = of_match_ptr(of_match),
	},
	.probe = machxo2_spi_probe,
	.remove = machxo2_spi_remove,
	.id_table = lattice_ids,
};

module_spi_driver(machxo2_spi_driver)

MODULE_AUTHOR("Paolo Pisati <p.pisati@gmail.com>");
MODULE_DESCRIPTION("Load Lattice FPGA firmware over SPI");
MODULE_LICENSE("GPL v2");
