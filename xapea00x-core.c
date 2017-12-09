/* XAP-EA-00x driver for Linux
 *
 *  Copyright (c) 2017 Xaptum, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include "xapea00x-backports.h"

#define USB_VENDOR_ID_SILABS           0x10c4
#define USB_PRODUCT_ID_XAPEA001        0x8BDE

#define XAPEA00X_NUM_CS                1

#define XAPEA00X_CMD_READ              0x00
#define XAPEA00X_CMD_WRITE             0x01
#define XAPEA00X_CMD_WRITEREAD         0x02

#define XAPEA00X_BREQTYP_SET           0x40

#define XAPEA00X_BREQ_SET_GPIO_VALUES  0x21
#define XAPEA00X_BREQ_SET_GPIO_CS      0x25
#define XAPEA00X_BREQ_SET_SPI_WORD     0x31

#define XAPEA00X_USB_TIMEOUT           1000 // msecs

#define XAPEA00X_GPIO_CS_DISABLED      0x00

#define XAPEA00X_TPM_SPI_WORD          0x08
#define XAPEA00X_TPM_MODALIAS          "tpm_tis_spi"

/*******************************************************************************
 * Constants from the TCP TPM 2.0 specification
 */

#define TPM_RETRY                      50
#define TPM_TIMEOUT                    5    // msecs
#define TPM_TIMEOUT_RANGE_US           300  // usecs

#define TPM2_TIMEOUT_A                 750  // msecs
#define TPM2_TIMEOUT_B                 2000 // msecs
#define TPM2_TIMEOUT_C                 200  // msecs
#define TPM2_TIMEOUT_D                 30   // msecs

#define TPM_ACCESS_0                   0x0000
#define TPM_STS_0                      0x001B
#define TPM_DATA_FIFO_0                0x0024

#define TPM2_ST_NO_SESSIONS            0x8001
#define TPM2_ST_SESSIONS               0x8002

#define TPM2_CC_STARTUP                0x0144
#define TPM2_CC_SHUTDOWN               0x0145
#define TPM2_CC_SELF_TEST              0x0143
#define TPM2_CC_GET_RANDOM             0x017B

enum tis_access {
	TPM_ACCESS_VALID = 0x80,
	TPM_ACCESS_ACTIVE_LOCALITY = 0x20,
	TPM_ACCESS_REQUEST_PENDING = 0x04,
	TPM_ACCESS_REQUEST_USE = 0x02
};

enum tis_status {
	TPM_STS_VALID          = 0x80,
	TPM_STS_COMMAND_READY  = 0x40,
	TPM_STS_GO             = 0x20,
	TPM_STS_DATA_AVAIL     = 0x10,
	TPM_STS_DATA_EXPECT    = 0x08
};

/*******************************************************************************
 * xapea00x device structures
 */

struct xapea00x_device {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;

	struct spi_master *spi_master;
	struct spi_device *tpm;
	struct work_struct tpm_probe;
};

struct xapea00x_bulk_command {
	__be16 reserved1;
	__u8   command;
	__u8   reserved2;
	__be32 length;
} __attribute__((__packed__));

/*******************************************************************************
 * Bridge USB transfers
 */

/**
 * xapea00x_prep_bulk_command - Prepares the bulk command header with
 * the supplied values.
 * @header: pointer to header to prepare
 * @command: the command id for the command
 * @length: length in bytes of the command data
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static void xapea00x_prep_bulk_command(struct xapea00x_bulk_command *header,
                                       u8 command, int length)
{
	header->reserved1 = 0;
	header->command   = command;
	header->reserved2 = 0;
	header->length    = __cpu_to_le32(length);
}

/**
 * xapea00x_bulk_write - Issues a builk write to the bridge chip.
 * @dev: pointer to the device to write to
 * @command: the command started by this write (WRITE, READ, WRITE_READ)
 * @data: pointer to the data to write. Must be DMA capable (e.g.,
 *	  kmalloc-ed, not stack).
 * @len: length in bytes of the data to write
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_bulk_write(struct xapea00x_device *dev,
                               struct xapea00x_bulk_command *header,
                               const void *data, int len)
{
	u8 *buf;
	unsigned int pipe;
	int buf_len, actual_len, retval;

	buf_len = sizeof(struct xapea00x_bulk_command) + len;
	buf = kmalloc(buf_len, GFP_KERNEL);
	if (!buf)
	{
		retval = -ENOMEM;
		goto out;
	}

	memcpy(buf, header, sizeof(struct xapea00x_bulk_command));
	memcpy(buf + sizeof(struct xapea00x_bulk_command), data, len);

	pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out->bEndpointAddress);
	retval = usb_bulk_msg(dev->udev, pipe, buf, buf_len, &actual_len,
	                      XAPEA00X_USB_TIMEOUT);
	if (retval) {
		dev_warn(&dev->interface->dev,
		         "%s: usb_bulk_msg() failed with %d\n",
		         __func__, retval);
		goto free_buf;
	}

	retval = 0;

free_buf:
	kfree(buf);

out:
	return retval;
}

/**
 * xapea00x_bulk_read - Issues a bulk read to the bridge chip.
 * @dev: pointer to the device to read from
 * @data: pointer to the data read. Must be DMA capable (e.g.,
 *	  kmalloc-ed, not stack).
 * @len: length in bytes of the data to read
 *
 * Return: If successful, 0. Otherwise a negative error code.
 */
static int xapea00x_bulk_read(struct xapea00x_device *dev, void* data, int len)
{
	unsigned int pipe;
	void *buf;
	int actual_len, retval;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto out;
	}

	pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in->bEndpointAddress);
	retval = usb_bulk_msg(dev->udev, pipe, buf, len, &actual_len,
	                      XAPEA00X_USB_TIMEOUT);

	if (retval) {
		dev_warn(&dev->interface->dev, "%s: usb_bulk_msg() failed with %d\n",
		          __func__, retval);
		goto free_buf;
	}

	memcpy(data, buf, actual_len);
	retval = 0;

free_buf:
	kfree(buf);

out:
	return retval;
}

/**
 * xapea00x_ctrl_write - Issues a send control transfer to the bridge
 * chip.
 * @dev: pointer to the device to write to
 * @bRequest: the command
 * @wValue: the command value
 * @wIndex: the command index
 * @data: pointer to the command data
 * @len: length in bytes of the command data
 *
 * The possible bRequest, wValue, and wIndex values and data format
 * are specified in the hardware datasheet.
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error code.
 */
static int xapea00x_ctrl_write(struct xapea00x_device *dev, u8 bRequest,
			       u16 wValue, u16 wIndex, u8 *data, u16 len)
{
	unsigned int pipe;
	void *buf;
	int retval;

	/* control_msg buffer must be dma-capable (e.g.k kmalloc-ed,
	 * not stack).	Copy data into such buffer here, so we can use
	 * simpler stack allocation in the callers - we have no
	 * performance concerns given the small buffers and low
	 * throughputs of this device.
	 */
	buf = kmalloc(len, GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto out;
	}
	memcpy(buf, data, len);

	pipe = usb_sndctrlpipe(dev->udev, 0);
	retval = usb_control_msg(dev->udev, pipe, bRequest,
	                         XAPEA00X_BREQTYP_SET, wValue, wIndex,
	                         buf, len, XAPEA00X_USB_TIMEOUT);
	if (retval < 0) {
		dev_warn(&dev->interface->dev, "usb_control_msg() failed with %d\n",
		         retval);
		goto free_buf;
	}

	retval = 0;

free_buf:
	kfree(buf);

out:
	return retval;
}

/*******************************************************************************
 * Bridge SPI reads and writes
 */
/**
 * xeapea00x_spi_read - Performs a read from the active channel
 * @dev: pointer to the device to perform the read
 * @rx_buf: pointer to the buffer to read the data into.  Must be
 *	    DMA-capable (e.g., kmalloc-ed, not stack).
 * @len: length in bytes of the data to read
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_spi_read(struct xapea00x_device *dev,
			     void* rx_buf, int len)
{
	struct xapea00x_bulk_command header;
	int retval;

	xapea00x_prep_bulk_command(&header, XAPEA00X_CMD_READ, len);
	retval = xapea00x_bulk_write(dev, &header, NULL, 0);
	if (retval)
		goto out;

	retval = xapea00x_bulk_read(dev, rx_buf, len);

out:
	return retval;
}

/**
 *xapea00x_spi_write - Performs a write to the active channel
 * @dev: pointer to the device to perform the write
 * @tx_buf: pointer to the data to write. Must be DMA-capable (e.g.,
 *	    kmalloc-ed, not stack).
 * @len: length in bytes of the data to write
 */
static int xapea00x_spi_write(struct xapea00x_device *dev,
			      const void* tx_buf, int len)
{
	struct xapea00x_bulk_command header;
	int retval;

	xapea00x_prep_bulk_command(&header, XAPEA00X_CMD_WRITE, len);
	retval = xapea00x_bulk_write(dev, &header, tx_buf, len);

	return retval;
}

/**
 * xapea00x_spi_write_read - Performs a simultaneous write and read on
 * the active channel
 * @dev: pointer to the device to perform the write/read
 * @tx_buf: pointer to the data to write. Must be DMA-capable (e.g.,
 *	    kmalloc-ed, not stack).
 * @rx_buf: pointer to the buffer to read the data into. Must be
 *	    DMA-capable (e.g., kmalloc-ed, not stack).
 * @len: length in bytes of the data to write/read
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_spi_write_read(struct xapea00x_device *dev,
				   const void* tx_buf, void* rx_buf, int len)
{
	struct xapea00x_bulk_command header;
	int retval;

	xapea00x_prep_bulk_command(&header, XAPEA00X_CMD_WRITEREAD, len);
	retval = xapea00x_bulk_write(dev, &header, tx_buf, len);
	if (retval)
		goto out;

	retval = xapea00x_bulk_read(dev, rx_buf, len);

out:
	return retval;
}

/*******************************************************************************
 * Bridge configuration commands
 */
/**
 * xapea00x_set_gpio_value - Sets the value on the specified pin of
 * the bridge chip.
 * @dev: pointer to the device containing the bridge whose pin to set
 * @pin: the number of the pin to set
 * @value: the value to set the pin to, 0 or 1
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_set_gpio_value(struct xapea00x_device *dev, u8 pin, u8 value)
{
	u8 data[4] = { 0, 0, 0, 0 };
	switch (pin) {
	case 10:
	case  9:
	case  8:
	case  7:
	case  6:
		data[0] = value << (pin - 4);
		data[2] = 1 << (pin - 4);
		break;
	case  5:
		data[0] = value;
		data[2] = 1;
		break;
	case  4:
	case  3:
	case  2:
	case  1:
	case  0:
		data[1] = value << (pin + 3);
		data[3] = 1 << (pin + 3);
	}
	return xapea00x_ctrl_write(dev, XAPEA00X_BREQ_SET_GPIO_VALUES,
	                           0, 0, data, 4);
}

/**
 * xapea00x_set_gpio_cs - Sets the chip select control on the specified
 * pin of the bridge chip.
 * @dev: pointer to the device containing the bridge whose cs to set
 * @pin: the number of the pin to set
 * @control: the chip select control value for the pin, 0, 1, or 2
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_set_gpio_cs(struct xapea00x_device *dev, u8 pin, u8 control)
{
	u8 data[2] = { pin, control };
	return xapea00x_ctrl_write(dev, XAPEA00X_BREQ_SET_GPIO_CS,
	                           0, 0, data, 2);
}

/**
 * xapea00x_set_spi_word - Sets the SPI config word on the specified
 * channel.
 * @dev: pointer to the device containing the SPI channel to configure
 * @channel: the number of the SPI channel to configure
 * @word: the SPI config word
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_set_spi_word(struct xapea00x_device *dev,
                                 u8 channel, u8 word)
{
	u8 data[2] = { channel, word };
	return xapea00x_ctrl_write(dev, XAPEA00X_BREQ_SET_SPI_WORD,
	                           0, 0, data, 2);
}

/*******************************************************************************
 * SPI master functions
 */

/**
 * xapea00x_spi_setup - Setup the SPI channel for the TPM.
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_spi_setup(struct spi_device *spi)
{
	struct xapea00x_device *dev;
	int retval;

	dev = spi_master_get_devdata(spi->master);

	/* Verify that this is the TPM device */
	if (spi->chip_select != 0) {
		retval = -EINVAL;
		goto err;
	}
      
	/* Set the SPI parameters for the TPM channel. */
	retval = xapea00x_set_spi_word(dev, 0, XAPEA00X_TPM_SPI_WORD);
	if (retval)
		goto err;

	/*
	 * Disable auto chip select for the TPM channel.
	 * Must be done after setting the SPI parameters.
	 */
	retval = xapea00x_set_gpio_cs(dev, 0, XAPEA00X_GPIO_CS_DISABLED);
	if (retval)
		goto err;

	/* De-assert chip select for the TPM channel. */
	retval = xapea00x_set_gpio_value(dev, 0, 1);
	if (retval)
		goto err;

	dev_dbg(&dev->interface->dev, "configured spi channel for tpm\n");
	return 0;

err:
	dev_err(&dev->interface->dev, "configuring SPI channel failed with %d\n", retval);
	return retval;
}

/**
 * xapea00x_spi_cleanup
 *
 * Context: !in_interrupt()
 */
static void xapea00x_spi_cleanup(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "%s\n", __func__);
}

/**
 * xapea00x_spi_transfer - Execute a single SPI transfer.
 * @dev: pointer to the device to do the transfer on
 * @tx_buf: pointer to the data to send, if not NULL
 * @rx_buf: pointer to the buffer to store the received data, if not NULL
 * @len: length in bytes of the data to send/receive
 * @cs_hold: If non-zero, the chip select will remain asserted
 * @delay_usecs: If nonzero, how long to delay after last bit transfer
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_spi_transfer(struct xapea00x_device *dev,
                                 const void *tx_buf, void *rx_buf, u32 len,
                                 int cs_hold, u16 delay_usecs)
{
	int retval;

	/* Assert chip select */
	retval = xapea00x_set_gpio_value(dev, 0, 0);
	if (retval)
		goto out;

	/* empty transfer */
	if (!tx_buf && !rx_buf) {
		retval = 0;
	}
	/* read transfer */
	else if (!tx_buf) {
		retval = xapea00x_spi_read(dev, rx_buf, len);
	}
	/* write transfer */
	else if (!rx_buf) {
		retval = xapea00x_spi_write(dev, tx_buf, len);
	}
	/* write_read transfer */
	else {
		retval = xapea00x_spi_write_read(dev, tx_buf, rx_buf, len);
	}

	/* Deassert chip select, if requested */
	if (!cs_hold)
		retval = xapea00x_set_gpio_value(dev, 0, 1);

	/* Delay for the requested time */
	udelay(delay_usecs);

out:
	return retval;
}

/**
 * xapea00x_spi_transfer_one_message - Execute a full SPI message.
 * @master: The SPI master on which to execute the message.
 * @msg: The SPI message to execute.
 *
 * Context: !in_interrupt()
 *
 * Return: If successfull, 0. Otherwise a negative erorr number.
 */
static int xapea00x_spi_transfer_one_message(struct spi_master *master,
                                             struct spi_message *msg)
{
	struct xapea00x_device *dev;
	struct spi_transfer *xfer;
	int is_last, retval;
      
	dev = spi_master_get_devdata(master);

	/* perform all transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		is_last = list_is_last(&xfer->transfer_list, &msg->transfers);

		/* Transfer message */
		retval = xapea00x_spi_transfer(dev, xfer->tx_buf,
                                               xfer->rx_buf, xfer->len,
                                               is_last == xfer->cs_change,
                                               xfer->delay_usecs);
		if (retval) {
			goto out;
		}

		msg->actual_length += xfer->len;
	}

	retval = 0;

out:
	msg->status = retval;
	spi_finalize_current_message(master);
	return retval;
}

/**
 * xapea00x_spi_probe - Register and configure the SPI master.
 * @dev: the device whose SPI master to register
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_spi_probe(struct xapea00x_device *dev)
{
	struct spi_master *spi_master;
	int retval;

	spi_master = spi_alloc_master(&dev->udev->dev, sizeof(void*));
	if (!spi_master) {
		retval = -ENOMEM;
		goto err_out;
	}

	spi_master_set_devdata(spi_master, dev);

	spi_master->min_speed_hz = 93 * 1000 + 800; /* 93.9kHz */
	spi_master->max_speed_hz = 12 * 1000 * 1000; /* 12 MHz */

	spi_master->bus_num = -1; /* dynamically assigned */
	spi_master->num_chipselect = XAPEA00X_NUM_CS;
	spi_master->mode_bits = SPI_MODE_0;

	spi_master->flags = 0;
	spi_master->setup = xapea00x_spi_setup;
	spi_master->cleanup = xapea00x_spi_cleanup;
	spi_master->transfer_one_message = xapea00x_spi_transfer_one_message;

	retval = spi_register_master(spi_master);

	if (retval)
		goto free_spi;

	dev->spi_master = spi_master;
	dev_dbg(&dev->interface->dev, "registered SPI master\n");

	return 0;

free_spi:
	spi_master_put(spi_master);
	dev->spi_master = NULL;

err_out:
	return retval;
}

/*******************************************************************************
 * TPM functions
 */

static void xapea00x_tpm_msleep(int msecs)
{
	usleep_range(msecs * 1000,
		     msecs * 1000 + TPM_TIMEOUT_RANGE_US);
}

static int xapea00x_tpm_transfer(struct xapea00x_device *dev,
				 u32 addr, u8 *in, u8 *out, u16 len)
{
	u8 header[4];
	int i, retval;

	header[0] = (in ? 0x80 : 0x00) | (len - 1);
	header[1] = 0xd4;
	header[2] = addr >> 8;
	header[3] = addr;

	retval = xapea00x_spi_transfer(dev, header, header, 4, 1, 0);
	if (retval)
		goto out;

	/* handle SPI wait states */
	if ((header[3] & 0x01) == 0x00) {
		header[0] = 0;

		for (i = 0; i < TPM_RETRY; i++) {
			retval = xapea00x_spi_transfer(dev, header, header, 1,
			                               1, 0);
			if (retval)
				goto out;
			if ((header[0] & 0x01) == 00)
				break;
		}
		
		if (i == TPM_RETRY) {
			retval = -ETIMEDOUT;
			goto out;
		}
	}
	
	retval = xapea00x_spi_transfer(dev, out, in, len, 0, 0);
	if (retval)
		goto out;

out:
        return retval;
}

static int xapea00x_tpm_read_bytes(struct xapea00x_device *dev, u32 addr,
                                   void *result, u16 len)
{
	return xapea00x_tpm_transfer(dev, addr, result, NULL, len);
}

static int xapea00x_tpm_write_bytes(struct xapea00x_device *dev, u32 addr,
                                    void *data, u16 len)
{
	return xapea00x_tpm_transfer(dev, addr, NULL, data, len);
}

static int xapea00x_tpm_read8(struct xapea00x_device *dev, u32 addr, u8 *result)
{
	return xapea00x_tpm_read_bytes(dev, addr, result, 1);
}

static int xapea00x_tpm_write8(struct xapea00x_device *dev, u32 addr, u8 data)
{
	return xapea00x_tpm_write_bytes(dev, addr, &data, 1);
}

static int xapea00x_tpm_read16(struct xapea00x_device *dev, u32 addr, u16 *result)
{
	u16 result_le;
	int retval;

	retval = xapea00x_tpm_read_bytes(dev, addr, &result_le, 2);
	if (retval)
		goto out;

	*result = __le16_to_cpu(result_le);
	retval = 0;

out:
	return retval;
}

static int xapea00x_tpm_write16(struct xapea00x_device *dev, u32 addr, u16 data)
{
	u16 data_le;

	data_le = __cpu_to_le16(data);
	return xapea00x_tpm_write_bytes(dev, addr, &data_le, 2);
}

static int xapea00x_tpm_read32(struct xapea00x_device *dev, u32 addr, u32 *result)
{
	u32 result_le;
	int retval;

	retval = xapea00x_tpm_read_bytes(dev, addr, &result_le, 4);
	if (retval)
		goto out;

	*result = __le32_to_cpu(result_le);
	retval = 0;

out:
	return retval;
}

static int xapea00x_tpm_write32(struct xapea00x_device *dev, u32 addr, u32 data)
{
	u32 data_le;

	data_le = __cpu_to_le32(data);
	return xapea00x_tpm_write_bytes(dev, addr, &data_le, 4);
}

static int xapea00x_tpm_wait_reg8(struct xapea00x_device *dev,
                                  u8 addr, u8 flags,
                                  int timeout_msecs)
{
	unsigned long stop = jiffies + msecs_to_jiffies(timeout_msecs);
	u8 reg;
	int retval;

	do {
		retval = xapea00x_tpm_read8(dev, addr, &reg);
		if (retval)
			goto out;

		if ((reg & flags) == flags) {
			retval = 0;
			goto out;
		}

		xapea00x_tpm_msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));

	retval = -ETIMEDOUT;

out:
	return retval;
}

static int xapea00x_tpm_wait_reg32(struct xapea00x_device *dev,
                                   u32 addr, u32 flags,
                                   int timeout_msecs)
{
	unsigned long stop = jiffies + msecs_to_jiffies(timeout_msecs);
	u32 reg;
	int retval;

	do {
		retval = xapea00x_tpm_read32(dev, addr, &reg);
		if (retval)
			goto out;

		if ((reg & flags) == flags) {
			retval = 0;
			goto out;
		}

		xapea00x_tpm_msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));

	retval = -ETIMEDOUT;

out:
	return retval;
}

	/* return xapea00x_tpm_check_reg8(dev, TPM_ACCESS_0, TPM_ACCESS_VALID, */
	/* return xapea00x_tpm_check_reg8(dev, TPM_ACCESS_0, TPM_ACCESS_ACTIVE_LOCA, */
	/* return xapea00x_tpm_check_reg32(dev, TPM_STS_0, TPM_STS_COMMAND_READ, */
	/* return xapea00x_tpm_check_reg32(dev, TPM_STS_0, TPM_STS_VALID, */
	/* return xapea00x_tpm_check_reg32(dev, TPM_STS_0, TPM_STS_VALID); */
	/* return xapea00x_tpm_wait_for_cond(dev, xapea00x_tpm_check_startup, */
        /*                                   TPM2_TIMEOUT_A) */


static int xapea00x_tpm_request_locality(struct xapea00x_device *dev)
{
	int retval;

	retval = xapea00x_tpm_write8(dev, TPM_ACCESS_0, TPM_ACCESS_REQUEST_USE);
	if (retval)
		goto out;

	retval = xapea00x_tpm_wait_reg8(dev, TPM_ACCESS_0,
                                        TPM_ACCESS_REQUEST_USE,
                                        TPM2_TIMEOUT_A);

out:
	return retval;
}

static int xapea00x_tpm_release_locality(struct xapea00x_device *dev)
{
	return xapea00x_tpm_write8(dev, TPM_ACCESS_0,
	                           TPM_ACCESS_ACTIVE_LOCALITY);
}

static int xapea00x_tpm_burst_count(struct xapea00x_device *dev, u32 *count)
{
	u32 reg;
	int retval;

	retval = xapea00x_tpm_read32(dev, TPM_STS_0, &reg);
	if (retval)
		goto out;

	*count = (reg >> 8) & 0xFFFF;
	retval = 0;

out:
	return retval;
}

static int xapea00x_tpm_send(struct xapea00x_device *dev, void* data, u32 len)
{
	u32 burst;
	int retval;

	/* wait for TPM to be ready for command */
	retval = xapea00x_tpm_wait_reg32(dev, TPM_STS_0, TPM_STS_COMMAND_READY,
                                         TPM2_TIMEOUT_B);
	if (retval)
		goto err;

	/* Write the command */
	while (len > 0) {
		xapea00x_tpm_burst_count(dev, &burst);
		burst = min(burst, len);

		retval = xapea00x_tpm_write_bytes(dev, TPM_DATA_FIFO_0, data, burst);
		if (retval)
			goto cancel;

		retval = xapea00x_tpm_wait_reg32(dev, TPM_STS_0, TPM_STS_VALID,
		                                 TPM2_TIMEOUT_C);
		if (retval)
			goto cancel;

		data += burst;
		len -= burst;
	}

	/* Do it */
	retval = xapea00x_tpm_write32(dev, TPM_STS_0, TPM_STS_GO);
	if (retval)
		goto cancel;

	return 0;

cancel:
	/* Attempt to cancel */
	retval = xapea00x_tpm_write32(dev, TPM_STS_0, TPM_STS_COMMAND_READY);

err:
	return retval;
}

static int xapea00x_tpm_recv(struct xapea00x_device *dev, void* data, u32 len)
{
	u32 burst;
	int retval;

	/* wait for TPM to have data available */
	retval = xapea00x_tpm_wait_reg32(dev, TPM_STS_0, TPM_STS_DATA_AVAIL,
                                         TPM2_TIMEOUT_C);
	if (retval)
		goto cancel;

	/* read the header */
	if (len < 10) {
		retval = -EINVAL;
		goto cancel;
	}
	
	retval = xapea00x_tpm_read_bytes(dev, TPM_DATA_FIFO_0, data, 10);
	if (retval)
		goto cancel;

	len -= 10;
	data += 10;

	// TODO - parse the header to determine the size of the body
	
	/* read the body */
	while (len > 0) {
		xapea00x_tpm_burst_count(dev, &burst);
		burst = min(burst, len);

		retval = xapea00x_tpm_read_bytes(dev, TPM_DATA_FIFO_0, data, burst);
		if (retval)
			goto cancel;

		len -= burst;
		data += burst;
	}

	/* wait for valid */
	retval = xapea00x_tpm_wait_reg32(dev, TPM_STS_0, TPM_STS_VALID,
	                                 TPM2_TIMEOUT_C);
	if (retval)
		goto err;

	return 0;

cancel:
	/* Attempt to cancel */
	retval = xapea00x_tpm_write32(dev, TPM_STS_0, TPM_STS_COMMAND_READY);

err:
	return retval;
}

static int xapea00x_tpm_transmit(struct xapea00x_device *dev, void* command,
                                 u32 len)
{
	int retval;

	retval = xapea00x_tpm_request_locality(dev);
	if (retval)
		goto out;

	retval = xapea00x_tpm_send(dev, command, len);

out:
	return retval;
}

static int xapea00x_tpm_platform_initialize(struct xapea00x_device *dev)
{
	int retval;

	/* wait for TPM to startup */
	retval =  xapea00x_tpm_wait_reg8(dev, TPM_ACCESS_0, TPM_ACCESS_VALID,
	                                 TPM2_TIMEOUT_A);
	if (retval)
		goto out;

out:
	return retval;
}

/**
 * xapea00x_tpm_probe - Register and intialize the TPM device
 * @work: the work struct contained by the xapea00x device
 *
 * Context: !in_interrupt()
 */
static void xapea00x_tpm_probe(struct work_struct *work)
{
	struct xapea00x_device *dev;
	struct spi_master *spi_master;
	struct spi_device *tpm;
	int retval;

	dev = container_of(work, struct xapea00x_device, tpm_probe);

	/*
	 * This driver is the "platform" in TPM terminology. Before
	 * passing control of the TPM to the Linux TPM subsystem, do
	 * the TPM initialization normally done by the platform code
	 * (e.g., BIOS).
	 */
	retval = xapea00x_tpm_platform_initialize(dev);
	if (retval) {
		dev_err(&dev->interface->dev,
			"unable to do TPM platform initialization\n");
		goto err;
	}

	/* Now register the TPM with the Linux TPM subsystem */
	spi_master = dev->spi_master;	
	tpm = spi_alloc_device(dev->spi_master);
	if (!tpm) {
		dev_err(&dev->interface->dev,
		        "unable to allocate spi device for TPM\n");
		goto err;
	}

	tpm->max_speed_hz  = spi_master->max_speed_hz;
	tpm->chip_select   = 0;
	tpm->mode	   = SPI_MODE_0;
	tpm->bits_per_word = 8;
	strncpy(tpm->modalias, XAPEA00X_TPM_MODALIAS, sizeof(tpm->modalias));

	retval = spi_add_device(tpm);
	if (retval) {
		dev_err(&dev->interface->dev, "spi_add_device() failed with %d\n",
		        retval);
		goto free_tpm;
	}

	dev->tpm = tpm;

	dev_info(&dev->interface->dev, "TPM initialization complete\n");
	return;

free_tpm:
	spi_dev_put(tpm);
	dev->tpm = NULL;

err:
	dev_err(&dev->interface->dev, "TPM initialization failed\n");
	return;
}

/*******************************************************************************
 * USB driver structs and functions
 */

static const struct usb_device_id xapea00x_devices[] = {
	{ USB_DEVICE(USB_VENDOR_ID_SILABS, USB_PRODUCT_ID_XAPEA001) },
	{ }
};
MODULE_DEVICE_TABLE(usb, xapea00x_devices);

static int xapea00x_usb_probe(struct usb_interface *interface,
                              const struct usb_device_id *id)
{
	struct xapea00x_device *dev;
	int retval;

	dev = kzalloc(sizeof(struct xapea00x_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* ---------------------- USB ------------------------ */
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	retval = usb_find_common_endpoints(interface->cur_altsetting,
	                                   &dev->bulk_in, &dev->bulk_out,
	                                   NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
		        "could not find both bulk-in and bulk-out endpoints\n");
		goto err_out;
	}

	usb_set_intfdata(interface, dev);

	/* ---------------------- SPI Master ------------------------ */
	retval = xapea00x_spi_probe(dev);
	if (retval) {
		dev_err(&interface->dev, "could not initialize SPI master\n");
		goto err_out;
	}

	/* ---------------------- TPM SPI Device ------------------------ */
	INIT_WORK(&dev->tpm_probe, xapea00x_tpm_probe);
	schedule_work(&dev->tpm_probe);
	dev_info(&interface->dev, "scheduled intialization of TPM\n");

	/* ---------------------- Finished ------------------------ */
	dev_info(&interface->dev, "device connected\n");
	return 0;

err_out:
	dev_err(&interface->dev, "initialization failed with %d\n", retval);
	return retval;
}

static void xapea00x_usb_disconnect(struct usb_interface *interface)
{
	struct xapea00x_device *dev = usb_get_intfdata(interface);

	spi_unregister_master(dev->spi_master);
	dev_dbg(&interface->dev, "unregistered SPI master\n");

	dev_info(&interface->dev, "device disconnected\n");
}

static struct usb_driver xapea00x_driver = {
	.name		      = "xapea00x",
	.probe		      = xapea00x_usb_probe,
	.disconnect	      = xapea00x_usb_disconnect,
	.suspend	      = NULL,
	.resume	      = NULL,
	.reset_resume	      = NULL,
	.id_table	      = xapea00x_devices,
	.supports_autosuspend = 0
};

/*******************************************************************************
 * Module functions
 */

static int __init xapea00x_init(void)
{
	int retval;

	retval = usb_register_driver(&xapea00x_driver, THIS_MODULE, "xapea00x");
	if (retval)
		pr_err("xapea00x: usb_register_driver() failed with %d\n", retval);

	return retval;
}

static void __exit xapea00x_exit(void)
{
	usb_deregister(&xapea00x_driver);
}

module_init(xapea00x_init);
module_exit(xapea00x_exit);

MODULE_AUTHOR("David R. Bild <david.bild@xaptum.com>");
MODULE_DESCRIPTION("Xaptum XAP-EA-00x ENF Access card");
MODULE_LICENSE("GPL");

MODULE_ALIAS("xapea00x");
