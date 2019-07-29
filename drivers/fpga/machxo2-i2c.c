/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <asm/uaccess.h>

#define MACHXO2_NAME "machxo2-i2c-flash"

/*
 * If true, we will try to recover the i2c bus on failures. This requires
 * additional entries in the device tree:
 *
 * sda-gpio
 * scl-gpio
 *
 * pinctrl with "i2c_active", "i2c_sleep", "i2c_gpio" pins
 */
/* #define MACHX02_INCLUDE_I2C_RECOVERY */

/*
 *  Due to not-well-understood problems on the i2c bus, the MachXO2 sometimes
 *  releases the i2c bus in the middle of a transfer.  We have seen at least
 *  three errors that can result from this:
 *      Transaction NACKs
 *      Incorrect data read when verifying (trailing bytes become 0xFF)
 *      Invalid status read with FAIL bit set, among others
 *
 *  Until the root cause is better understood, we are implementing a retry
 *  strategy:  On any of these errors, wait one second and retry, up to a
 *  maximum of 4 retries (5 tries total)
 */
#define MACHXO2_RETRY_COUNT 4 /* 4 retries for everything! (incl. NACKs,
	                             failure status, and page verify fails) */
#define MACHXO2_I2C_DELAY 100 /* 100us between all i2c transactions */

#define MACHXO2_PAGE_SIZE 16 /* flash row/page size, in bytes */

/* -----------------------------------------------------------------
 * Structures and static variables
 * ----------------------------------------------------------------- */

struct machxo2_userdata {
	struct i2c_client* i2c;
	struct mutex mutex;
	struct wake_lock wakelock;

	u32 section_size_config; /* config section length in pages */
	u32 section_size_ufm; /* UFM section length in pages */
	u32 refresh_delay; /* delay needed after refresh, in seconds */

	struct regulator *vdd1_reg; /* optional device regulator */
	struct regulator *vdd2_reg; /* optional device regulator */

#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	struct dentry *debugFolder;
	struct debugfs_blob_wrapper flashPre;
	struct debugfs_blob_wrapper flashErased;
	struct debugfs_blob_wrapper flashPost;
#endif
};

enum machxo2_section {
	MACHXO2_SECTION_CONFIG,
	MACHXO2_SECTION_UFM,
};

/* -----------------------------------------------------------------
 * Regulator settings
 * ----------------------------------------------------------------- */

static int machxo2_regulator_enable(struct machxo2_userdata *userdata)
{
	int ret = 0;
	if (userdata->vdd1_reg) {
		ret = regulator_enable(userdata->vdd1_reg);
		if (ret) {
			dev_err(&userdata->i2c->dev, "Error in enabling vdd1, %d\n", ret);
			goto reg_error;
		}
	}
	if (userdata->vdd2_reg) {
		ret = regulator_enable(userdata->vdd2_reg);
		if (ret) {
			dev_err(&userdata->i2c->dev, "Error in enabling vdd2, %d\n", ret);
			goto regio_error;
		}
	}

	return 0;
regio_error:
	if (userdata->vdd1_reg)
		regulator_disable(userdata->vdd1_reg);
reg_error:
	return ret;
}


static int machxo2_regulator_disable(struct machxo2_userdata *userdata)
{
	int ret = 0;
	if (userdata->vdd2_reg) {
		ret = regulator_disable(userdata->vdd2_reg);
		if (ret) {
			dev_err(&userdata->i2c->dev, "Error in disabling vdd2, %d\n", ret);
			goto regio_error;
		}
	}
	if (userdata->vdd1_reg) {
		ret = regulator_disable(userdata->vdd1_reg);
		if (ret) {
			dev_err(&userdata->i2c->dev, "Error in disabling vdd1, %d\n", ret);
			goto reg_error;
		}
	}

	return 0;
reg_error:
	if (userdata->vdd2_reg)
		ret = regulator_enable(userdata->vdd2_reg);
regio_error:
	return ret;
}

static int machxo2_regulator_init(struct i2c_client* client, struct machxo2_userdata *userdata)
{
	userdata->vdd1_reg =  devm_regulator_get(&client->dev, "vdd1");
	if (IS_ERR(userdata->vdd1_reg)) {
		userdata->vdd1_reg = NULL;
	}
	userdata->vdd2_reg =  devm_regulator_get(&client->dev, "vdd2");
	if (IS_ERR(userdata->vdd2_reg)) {
		userdata->vdd2_reg = NULL;
	}
	return 0;
}

/* -----------------------------------------------------------------
 * Low-level API
 * ----------------------------------------------------------------- */

/* Do what is necessary to return the bus to a sane state after a failure */
int machxo2_pre_retry(struct machxo2_userdata* userdata)
{
	int ret = 0;
#ifdef MACHX02_INCLUDE_I2C_RECOVERY
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state, *active_state;
	struct i2c_adapter *bus;
	struct device *dev = &userdata->i2c->dev;
	struct gpio_desc *scl_gpio, *sda_gpio;
	int sda_val;
	int i;

	bus = userdata->i2c->adapter;

	/* Lock the i2c bus */
	i2c_lock_adapter( bus );

	/* Let pm_runtime know we are using the bus, don't put it to sleep */
	pm_runtime_get_sync( bus->dev.parent );

 	pinctrl = pinctrl_get( bus->dev.parent );
	if( IS_ERR_OR_NULL(pinctrl) ) {
		dev_err( dev, "pinctrl_get: %p\n", pinctrl );
		ret = PTR_ERR(pinctrl);
		goto out;
	}
	gpio_state = pinctrl_lookup_state( pinctrl, "i2c_gpio" );
	if( IS_ERR_OR_NULL(gpio_state) ) {
		dev_err( dev, "pinctrl_lookup_state(i2c_gpio): %p\n", gpio_state );
		ret = PTR_ERR(gpio_state);
		goto out_pinctrl;
	}

	active_state = pinctrl_lookup_state( pinctrl, "i2c_active" );
	if( IS_ERR_OR_NULL(active_state) ) {
		dev_err( dev, "pinctrl_lookup_state(i2c_gpio): %p\n", active_state );
		ret = PTR_ERR(active_state);
		goto out_pinctrl;
	}

	ret = pinctrl_select_state( pinctrl, gpio_state );
	if( ret < 0 ) {
		dev_err( dev, "pinctrl_select_state(gpio): %d\n", -ret );
		goto out_pinctrl;
	}

	scl_gpio = gpiod_get( dev, "scl", GPIOD_OUT_HIGH );
	if( IS_ERR( scl_gpio ) )
	{
		dev_err( dev, "gpiod_get(scl): %ld", PTR_ERR(scl_gpio) );
		ret = PTR_ERR(scl_gpio);
		goto out_state;
	}

	sda_gpio = gpiod_get( dev, "sda", GPIOD_IN );
	if( IS_ERR( sda_gpio ) )
	{
		dev_err( dev, "gpiod_get(sda): %ld", PTR_ERR(sda_gpio) );
		ret = PTR_ERR(sda_gpio);
		goto out_gpio_scl;
	}

	msleep(50);

	sda_val = gpiod_get_value(sda_gpio);

	for( i = 0; i < 10 && sda_val == 0; i++ )
	{
		gpiod_set_value(scl_gpio,0);
		udelay( 50 );
		gpiod_set_value(scl_gpio,1);
		udelay( 50 );
		sda_val = gpiod_get_value(sda_gpio);
	}

	if( i >= 10 ) {
		dev_err( dev, "Unable to reset bus, sda is still stuck low after %d tries\n", i );
		ret = -EIO;
		goto out_gpio_sda;
	}

	msleep(50);

out_gpio_sda:
	gpiod_put(sda_gpio);
out_gpio_scl:
	ret = gpiod_direction_input(scl_gpio);
	if( ret < 0 ) {
		dev_err( dev, "gpio_direction_input(scl): %d\n", -ret );
	}
	gpiod_put(scl_gpio);
out_state:
	pinctrl_select_state( pinctrl, active_state );
out_pinctrl:
	pinctrl_put( pinctrl );
out:
	pm_runtime_put(bus->dev.parent);
	i2c_unlock_adapter( bus );
#endif

	return ret;
}


/* Sends one command, optionally receiving data if recvSize != 0 */
static int machxo2_cmd(struct machxo2_userdata* userdata, const void* send, size_t sendSize, void* recv, size_t recvSize, int retries) {
	struct i2c_msg msgs[2];
	int ret;
retry:

	WARN_ON(sendSize == 0);

	msgs[0].addr = userdata->i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = sendSize;
	msgs[0].buf = (void*) send;

	//dev_dbg(&userdata->i2c->dev, "MachXO2 Addr: 0x%x cmd: 0x%x\n", (int) userdata->i2c->addr, (int) send[0]);

	if(recvSize == 0) {
		ret = i2c_transfer(userdata->i2c->adapter, msgs, 1);
	} else {
		msgs[1].addr = userdata->i2c->addr;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = recvSize;
		msgs[1].buf = recv;
		ret = i2c_transfer(userdata->i2c->adapter, msgs, 2);
	}

	usleep_range( MACHXO2_I2C_DELAY, 2*MACHXO2_I2C_DELAY );

	if( ret < 0 && retries > 0 ) {
		dev_warn( &userdata->i2c->dev, "Transfer failed, %d retries left\n", retries-- );
		machxo2_pre_retry(userdata);
		goto retry;
	}

	return ret;
}

/* Gets the status registers. Returns <0 on error. Status is written to the status pointer. */
static int machxo2_get_status(struct machxo2_userdata* userdata, u32 *status) {
	u8 sendBuf[4] = {0x3C, 0, 0, 0};
	__be32 bestatus;
	int ret;

	ret = machxo2_cmd(userdata, sendBuf, sizeof(sendBuf), &bestatus, sizeof(bestatus), MACHXO2_RETRY_COUNT);
	if(ret >= 0)
		*status = be32_to_cpu(bestatus);

	return ret;
}

static void machxo2_dump_status(struct machxo2_userdata* userdata) __attribute__((unused));
static void machxo2_dump_status(struct machxo2_userdata* userdata) {
	u32 status;
	int ret;

	ret = machxo2_get_status(userdata, &status);
	if(ret < 0)
		dev_err(&userdata->i2c->dev, "Unable to dump MachXO2 status: %x\n", -ret);
	else
		dev_err(&userdata->i2c->dev, "MachXO2 status: %x\n", status);
}

/* Returns 0 if free, >0 if busy, or <0 if error */
static int machxo2_is_busy(struct machxo2_userdata* userdata) {
	static const u32 BIT_BUSY = 1<<12;
	static const u32 BIT_FAIL = 1<<13;

	int ret;
	u32 status;

	int retries = MACHXO2_RETRY_COUNT;
	retry:

	ret = machxo2_get_status(userdata, &status);
	if(ret < 0)
		return ret;

	if(status & BIT_FAIL) {
		dev_err(&userdata->i2c->dev, "MachXO2 fail bit set\n");

		/* If the last byte of the status is 0xFF, retry on the assumption that
		   The FPGA just went away and let go of the bus mid-transaction */
		if((status & 0xFF) == 0xFF && retries > 0) {
			dev_warn(&userdata->i2c->dev, "Retries left: %d\n", retries-- );
			machxo2_pre_retry(userdata);
			goto retry;
		}

		return -EIO;
	}

	return status & BIT_BUSY;
}

/* Waits until not busy */
static int machxo2_wait(struct machxo2_userdata* userdata) {
	int ret;

	while((ret = machxo2_is_busy(userdata))) {
		if(ret < 0)
			return ret;
		msleep(100);
	}
	return 0;
}

/* Enables the configuration interface, in transparent mode */
static int machxo2_enable_config(struct machxo2_userdata* userdata) {
	int ret;
	u8 buf[3] = {0x74, 0x08, 0};

	ret = machxo2_cmd(userdata, buf, sizeof(buf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	ret = machxo2_wait(userdata);
	if(ret < 0)
		return ret;

	return 0;
}

/* Disables the configuration interface */
static int machxo2_disable_config(struct machxo2_userdata* userdata) {
	int ret;
	u8 disableCmdBuf[3] = {0x26, 0, 0};
	u8 bypassBuf[4] = {0xFF, 0xFF, 0xFF, 0xFF};

	ret = machxo2_cmd(userdata, disableCmdBuf, sizeof(disableCmdBuf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	ret = machxo2_wait(userdata);
	if(ret < 0)
		return ret;

	return machxo2_cmd(userdata, bypassBuf, sizeof(bypassBuf), NULL, 0, MACHXO2_RETRY_COUNT);
}

/* Erases a flash section. */
static int machxo2_erase(struct machxo2_userdata* userdata, enum machxo2_section section) {
	int ret;
	u8 buf[4] = {0x0E, 0, 0, 0};

	WARN_ON(section != MACHXO2_SECTION_CONFIG && section != MACHXO2_SECTION_UFM);

	buf[1] = section == MACHXO2_SECTION_UFM ? 0x8 : 0x4; /* specify magic bits for the section to erase */

	ret = machxo2_cmd(userdata, buf, sizeof(buf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	ret = machxo2_wait(userdata);
	if(ret < 0)
		return ret;

	return 0;
}

/* Refresh FPGA, loading new code */
static int machxo2_refresh(struct machxo2_userdata* userdata) {
	int ret;
	u8 buf[3] = {0x79, 0, 0};

	ret = machxo2_cmd(userdata, buf, sizeof(buf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	mdelay(userdata->refresh_delay); /* Cannot use I2C bus while refreshing */

	ret = machxo2_wait(userdata);
	if(ret < 0)
		return ret;

	return 0;
}

/* Seeks the page address pointers. */
static int machxo2_seek(struct machxo2_userdata* userdata, u16 page, enum machxo2_section section) {
	int ret;
	u8 buf[8] = {0xB4, 0};

	WARN_ON(section != MACHXO2_SECTION_CONFIG && section != MACHXO2_SECTION_UFM);

	buf[4] = section == MACHXO2_SECTION_UFM ? 0x40 : 0;
	buf[6] = page >> 8;
	buf[7] = page & 0xFF;

	ret = machxo2_cmd(userdata, buf, sizeof(buf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	ret = machxo2_wait(userdata);
	if(ret < 0)
		return ret;

	return 0;
}

/* Reads one page from either UFM or the configuration section. Takes a page location, but assumes that it's already
 * seeked there, and only uses it for retrying. */
static int machxo2_read_page(struct machxo2_userdata* userdata, u16 page, void* dest, enum machxo2_section section) {
	u8 cmdBuf[4] = {0,0,0,1};
	int ret;
	unsigned int retries = MACHXO2_RETRY_COUNT;

	WARN_ON(section != MACHXO2_SECTION_CONFIG && section != MACHXO2_SECTION_UFM);

	cmdBuf[0] = section == MACHXO2_SECTION_UFM ? 0xCA : 0x73;

	while(true) {
		if(retries != MACHXO2_RETRY_COUNT) {
			/* Don't seek on first try, as that wastes bandwidth */

			ret = machxo2_seek(userdata, page, section);
			if(ret < 0) {
				dev_err(&userdata->i2c->dev, "Failed to seek to the page to retry reading\n");
				return ret;
			}
		}

		ret = machxo2_cmd(userdata, cmdBuf, sizeof(cmdBuf), dest, MACHXO2_PAGE_SIZE, 0);
		if(ret >= 0)
			break;
		else if(retries == 0) {
			return ret;
		} else {
			dev_warn(&userdata->i2c->dev, "Read failed: %d. Retrying read (%u left)\n", -ret, retries);
			retries--;
			machxo2_pre_retry( userdata );
		}
	}

	return ret;
}

/* Writes one page to either UFM or the configuration section. Pages are MACHXO2_PAGE_SIZE bytes long.
 * Takes a page location, but assumes that it's already seeked there, and only uses it for retrying. */
static int machxo2_write_page(struct machxo2_userdata* userdata, u16 page, const u8* src, enum machxo2_section section) {
	int ret;
	u8 cmdBuf[4+MACHXO2_PAGE_SIZE] = {0,0,0,0x1};
	unsigned int retries = MACHXO2_RETRY_COUNT;

	WARN_ON(section != MACHXO2_SECTION_CONFIG && section != MACHXO2_SECTION_UFM);

	cmdBuf[0] = section == MACHXO2_SECTION_UFM ? 0xC9 : 0x70;
	memcpy(&cmdBuf[4], src, MACHXO2_PAGE_SIZE);

	while(true) {
		if(retries != MACHXO2_RETRY_COUNT) {
			/* Don't seek on first try, as that wastes bandwidth */

			ret = machxo2_seek(userdata, page, section);
			if(ret < 0) {
				dev_err(&userdata->i2c->dev, "Failed to seek to the page to retry writing\n");
				return ret;
			}
		}

		ret = machxo2_cmd(userdata, cmdBuf, sizeof(cmdBuf), NULL, 0, 0);
		if(ret >= 0)
			break;
		else if(retries == 0) {
			return ret;
		} else {
			dev_warn(&userdata->i2c->dev, "Write failed: %d. Retrying write (%u left)\n", -ret, retries);
			retries--;
			machxo2_pre_retry( userdata );
		}
	}

	ret = machxo2_wait(userdata);
	if(ret < 0) {
		return ret;
	}

	return 0;
}

/* Finishes flashing by setting the DONE bit */
static int machxo2_finish_flash(struct machxo2_userdata* userdata) {
	int ret;
	u8 cmdBuf[4] = {0x5E, 0, 0, 0};

	ret = machxo2_cmd(userdata, cmdBuf, sizeof(cmdBuf), NULL, 0, MACHXO2_RETRY_COUNT);
	if(ret < 0)
		return ret;

	return machxo2_wait(userdata);
}

/* Gets the size of a flash section, in pages. */
static u16 machxo2_section_size(struct machxo2_userdata* userdata, enum machxo2_section section) {
	WARN_ON(section != MACHXO2_SECTION_CONFIG && section != MACHXO2_SECTION_UFM);
	switch(section) {
	case MACHXO2_SECTION_CONFIG:
		return userdata->section_size_config;
	case MACHXO2_SECTION_UFM:
		return userdata->section_size_ufm;
	default:
		return 0;
	}
}

/* -----------------------------------------------------------------
 * High-level API
 * ----------------------------------------------------------------- */

/* Reads a section of the FPGA and verifies against an expected image.
 *  Assumes that config mode is enabled, and that the buffer pointed to by img is
 * the length of the selected flash section */
static int machxo2_verify_section(struct machxo2_userdata *userdata, const u8 *img, enum machxo2_section section) {
	u16 page;
	int ret;
	u8 pageData[MACHXO2_PAGE_SIZE];
	int badBytes = 0;
	struct device *dev = &userdata->i2c->dev;

	ret = machxo2_seek(userdata, 0, section);
	if(ret < 0)
		return ret;

	for(page = 0; page < machxo2_section_size(userdata, section); page++, img+=MACHXO2_PAGE_SIZE) {
		size_t i;
		int retries = MACHXO2_RETRY_COUNT;
		retry:

		ret = machxo2_read_page(userdata, page, pageData, section);
		if(ret < 0)
			return ret;

		for( i = 0; i < MACHXO2_PAGE_SIZE; i++ )
		{
			if(pageData[i] != img[i]) {
				size_t err_pos = page*MACHXO2_PAGE_SIZE + i;
				dev_warn(dev, "Flash mismatch at byte 0x%08zx: expected 0x%02x, read 0x%02x\n",
					err_pos, img[i], pageData[i]);
				badBytes++;
			}
		}
		if(badBytes > 0 && retries > 0) {
			/* If the FPGA stopped responding halfway through a read, we will get bad
			bytes back, but the programmed data may be fine.  Retry the read to be sure */
			dev_warn(dev, "Retry verifying page: %d retries left\n", retries-- );
			badBytes = 0;
			machxo2_pre_retry(userdata);

			ret = machxo2_seek(userdata, page, section);
			if(ret < 0) {
				dev_err(&userdata->i2c->dev, "Failed to seek to the page to retry reading\n");
				return ret;
			}

			goto retry;
		}

		if(badBytes > 0) {
			dev_warn(dev, "Additional mismatches skipped.\n");
			break;
		}
	}

	return badBytes;
}

/* Flashes a section of the FPGA. Assumes that config mode is enabled, and that the buffer pointed to by img is
 * the length of the selected flash section */
static int machxo2_flash_section(struct machxo2_userdata *userdata, const u8 *img, enum machxo2_section section) {
	size_t pages = machxo2_section_size(userdata, section);
	size_t page;
	int ret;

#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	ret = machxo2_read_section(userdata, userdata->flashPre.data, MACHXO2_SECTION_CONFIG);
	if(ret < 0) {
		dev_err(&userdata->i2c->dev, "Error capturing pre-flash image: %d\n", -ret);
		return ret;
	}
#endif

	ret = machxo2_erase(userdata, section);
	if(ret < 0)
		return ret;

#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	ret = machxo2_read_section(userdata, userdata->flashErased.data, MACHXO2_SECTION_CONFIG);
	if(ret < 0) {
		dev_err(&userdata->i2c->dev, "Error capturing erased image: %d\n", -ret);
		return ret;
	}
#endif

	ret = machxo2_seek(userdata, 0, section);
	if(ret < 0)
		return ret;

	for(page = 0; page < pages; page++, img += MACHXO2_PAGE_SIZE) {
		ret = machxo2_write_page(userdata, page, img, section);
		if(ret < 0)
			return ret;
	}

	return machxo2_finish_flash(userdata);
}

/* -----------------------------------------------------------------
 * SysFS API
 * ----------------------------------------------------------------- */

static ssize_t machxo2_sysfs_flash_fpga(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct machxo2_userdata *userdata = i2c_get_clientdata(container_of(dev, struct i2c_client, dev));
	const struct firmware* firmware;
	ssize_t ret;
	int retries = 100; /* 100 retries will take about an hour, but if we
	                      fail, the device is liable to be bricked.  If
			      we don't succeed in 100 tries, then the flash is
			      probably dead and we're bricked anyway. */
	u8 *verifyBuf;

	if(mutex_lock_killable(&userdata->mutex)) {
		return -EINTR;
	}
	wake_lock(&userdata->wakelock);
	ret = machxo2_regulator_enable(userdata);
	if (ret)
		goto unlock;

	dev_dbg(dev, "Reading firmware: `%s`\n", buf);

	ret = request_firmware(&firmware, buf, dev);
	if(ret < 0) {
		dev_err(dev, "Error requesting firmware: %zd\n", -ret);
		goto unlock;
	}

	if(firmware->size != machxo2_section_size(userdata, MACHXO2_SECTION_CONFIG)*MACHXO2_PAGE_SIZE) {
		dev_err(dev, "Flash size mismatch; image size: %zu, fpga size: %u\n",
			firmware->size,
			(unsigned int)(machxo2_section_size(userdata, MACHXO2_SECTION_CONFIG)*MACHXO2_PAGE_SIZE)
		);
		ret = -ENOSPC;
		goto free_firmware;
	}

	verifyBuf = kmalloc(firmware->size, GFP_KERNEL);
	if(!verifyBuf) {
		dev_err(dev, "Error allocating verification buffer: out of memory\n");
		ret = -ENOMEM;
		goto free_firmware;
	}

retry:
	ret = machxo2_enable_config(userdata);
	if(ret < 0) {
		dev_err(dev, "Error initializing config mode: %zd\n", -ret);
		goto free_buf;
	}

	dev_notice(dev, "Beginning firmware write: '%s'\n", buf);

	ret = machxo2_flash_section(userdata, firmware->data, MACHXO2_SECTION_CONFIG);
	if(ret < 0) {
		dev_err(dev, "Error flashing firmware: %zd\n", -ret);
		goto disable_config;
	}

	dev_notice(dev, "Firmware written, reading for verification...\n");

#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	memcpy(userdata->flashPost.data, verifyBuf, userdata->flashPost.size);
#endif

	dev_notice(dev, "Verifying...\n");
	ret = machxo2_verify_section(userdata, firmware->data, MACHXO2_SECTION_CONFIG);
	/* ret < 0 is an error condition */
	if(ret < 0) {
		dev_err(dev, "Error reading firmware: %zd\n", -ret);
		goto disable_config;
	}

	/* ret > 0 indicates the number of bytes that failed verification */
	if( ret > 0 ) {
		if( retries > 0 )
		{
			int disableRet;

			dev_warn( dev, "Verification failed, retrying (%d retries left)\n", retries );
			retries--;
			ret = 0;

			disableRet = machxo2_disable_config(userdata);
			if(disableRet < 0)
				dev_warn(dev, "Error disabling config mode: %d\n", -disableRet);
			goto retry;
		}

		dev_err(dev, "Verification failed, aborting.\n");

		ret = -EIO;
		goto disable_config;
	}

	dev_notice(dev, "Verification complete, refreshing device\n");
	ret = machxo2_refresh(userdata);
	if(ret < 0)
		goto disable_config;

	ret = count;

disable_config:
	if(ret < 0) { /* refresh will exit config mode in the success path */
		int disableRet = machxo2_disable_config(userdata);
		if(disableRet < 0)
			dev_warn(dev, "Error disabling config mode: %d\n", -disableRet);
	}
free_buf:
	kfree(verifyBuf);
free_firmware:
	release_firmware(firmware);
	machxo2_regulator_disable(userdata);
unlock:
	wake_unlock(&userdata->wakelock);
	mutex_unlock(&userdata->mutex);
	return ret;
}

static DEVICE_ATTR(flash_fpga, S_IWUSR | S_IWGRP, NULL, machxo2_sysfs_flash_fpga);

static struct attribute *machxo2_attr[] = {
		&dev_attr_flash_fpga.attr,
		NULL};
static const struct attribute_group machxo2_attr_group = {
		.attrs = machxo2_attr,
};

/* -----------------------------------------------------------------
 * Probe/remove
 * ----------------------------------------------------------------- */
static int machxo2_probe(struct i2c_client* client, const struct i2c_device_id* id) {
	int ret;
	struct machxo2_userdata* userdata;
#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	char debugName[128];
#endif

	dev_dbg(&client->dev, "Registering device");

	userdata = devm_kzalloc(&client->dev, sizeof(*userdata), GFP_KERNEL);
	if(!userdata) {
		ret = -ENOMEM;
		goto free_userdata;
	}
	userdata->i2c = client;
	mutex_init(&userdata->mutex);
	wake_lock_init(&userdata->wakelock, WAKE_LOCK_SUSPEND, MACHXO2_NAME);

	ret = of_property_read_u32(client->dev.of_node, "section-size-config", &userdata->section_size_config);
	if(ret < 0) {
		dev_err(&client->dev, "Error reading section-size-config from device tree\n");
		goto free_userdata;
	}

	ret = of_property_read_u32(client->dev.of_node, "section-size-ufm", &userdata->section_size_ufm);
	if(ret < 0) {
		dev_err(&client->dev, "Error reading section-size-ufm from device tree\n");
		goto free_userdata;
	}

	ret = of_property_read_u32(client->dev.of_node, "trefresh-time", &userdata->refresh_delay);
	if(ret < 0) {
		dev_err(&client->dev, "Error reading trefresh-time from device tree\n");
		goto free_userdata;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Device does not support full I2C\n");
		ret = -ENODEV;
		goto free_userdata;
	}
	ret = machxo2_regulator_init(client, userdata);
	if (ret) {
		goto free_userdata;
	}

	i2c_set_clientdata(client, userdata);

#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	userdata->flashPre.size = machxo2_section_size(userdata, MACHXO2_SECTION_CONFIG)*MACHXO2_PAGE_SIZE;
	userdata->flashErased.size = userdata->flashPre.size;
	userdata->flashPost.size = userdata->flashPre.size;

	userdata->flashPre.data = kzalloc(userdata->flashPre.size, GFP_KERNEL);
	userdata->flashErased.data = kzalloc(userdata->flashErased.size, GFP_KERNEL);
	userdata->flashPost.data = kzalloc(userdata->flashPost.size, GFP_KERNEL);

	if(!userdata->flashPre.data || !userdata->flashErased.data || !userdata->flashPost.data) {
		ret = -ENOMEM;
		goto free_captures;
	}

	snprintf(debugName, ARRAY_SIZE(debugName), "machxo2-%d-%04x",
		i2c_adapter_id(client->adapter), client->addr);
	userdata->debugFolder = debugfs_create_dir(debugName, NULL);
	if(userdata->debugFolder == ERR_PTR(-ENODEV))
		userdata->debugFolder = NULL;

	if(userdata->debugFolder) {
		debugfs_create_blob("flashPre", (S_IRUSR | S_IRGRP), userdata->debugFolder, &userdata->flashPre);
		debugfs_create_blob("flashErased", (S_IRUSR | S_IRGRP), userdata->debugFolder, &userdata->flashErased);
		debugfs_create_blob("flashPost", (S_IRUSR | S_IRGRP), userdata->debugFolder, &userdata->flashPost);
	}
#endif

	ret = sysfs_create_group(&(client->dev.kobj), &machxo2_attr_group);
	if (ret < 0) {
		goto free_debugfile;
	}

	dev_dbg(&client->dev, "Device created, has %u pages of config flash\n", (unsigned int)userdata->section_size_config);

	return 0;

free_debugfile:
#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	debugfs_remove_recursive(userdata->debugFolder);
free_captures:
	kfree(userdata->flashPre.data);
	kfree(userdata->flashErased.data);
	kfree(userdata->flashPost.data);
#endif
free_userdata:
	devm_kfree(&client->dev, userdata);

	dev_dbg(&client->dev, "Error registering device: %d\n", ret);

	return ret;
}

static int machxo2_remove(struct i2c_client* client) {
	struct machxo2_userdata* userdata;
	dev_dbg(&client->dev, "Removing device");

	userdata = i2c_get_clientdata(client);
#ifdef CONFIG_MACHXO2_I2C_FLASH_DEBUG
	debugfs_remove_recursive(userdata->debugFolder);
	kfree(userdata->flashPre.data);
	kfree(userdata->flashErased.data);
	kfree(userdata->flashPost.data);
#endif
	sysfs_remove_group(&(client->dev.kobj), &machxo2_attr_group);
	devm_kfree(&client->dev, userdata);
	return 0;
}

static struct i2c_device_id machxo2_idtable[] = {
	{ MACHXO2_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, machxo2_idtable);

static const struct of_device_id machxo2_compat[] = {
	{.compatible = "machxo2-i2c-flash"},
	{}
};
MODULE_DEVICE_TABLE(of, machxo2_compat);

#ifdef CONFIG_PM_SLEEP
static int machxo2_resume(struct device *dev){
	return 0;
}
static int machxo2_suspend(struct device *dev){
	return 0;
}
static const struct dev_pm_ops machxo2_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(machxo2_suspend, machxo2_resume)
};
#endif

static struct i2c_driver machxo2_driver = {
	.driver = {
		.name			= MACHXO2_NAME,
		.owner			= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm				= &machxo2_pm_ops,
#endif
		.of_match_table	= machxo2_compat,
	},
	.id_table	= machxo2_idtable,
	.probe		= machxo2_probe,
	.remove		= machxo2_remove,
};

/* -----------------------------------------------------------------
 * Module Init/info
 * ----------------------------------------------------------------- */
static int __init machxo2_initialize(void) {
	pr_debug(MACHXO2_NAME " driver initializing\n");
	return i2c_add_driver(&machxo2_driver);
}

static void __exit machxo2_exit(void) {
	pr_debug(MACHXO2_NAME " driver exiting\n");
	i2c_del_driver(&machxo2_driver);
}

module_init(machxo2_initialize);
module_exit(machxo2_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alex Parrill <aparrill@d3engineering.com>");
MODULE_DESCRIPTION("D3 Peripheral Board FPGA Flash Update Driver");
