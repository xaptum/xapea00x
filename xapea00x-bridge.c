/* XAP-EA-00x driver for Linux
 *
 *  Copyright (c) 2017-2018 Xaptum, Inc.
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

#include "xapea00x.h"

#define XAPEA00X_BR_CMD_READ			0x00
#define XAPEA00X_BR_CMD_WRITE			0x01
#define XAPEA00X_BR_CMD_WRITEREAD		0x02

#define XAPEA00X_BR_BREQTYP_SET		0x40

#define XAPEA00X_BR_BREQ_SET_GPIO_VALUES	0x21
#define XAPEA00X_BR_BREQ_SET_GPIO_CS		0x25
#define XAPEA00X_BR_BREQ_SET_SPI_WORD		0x31

#define XAPEA00X_BR_USB_TIMEOUT		1000 // msecs

/*
 * The bridge sometimes fails if consecutive USB transfers
 * are issued too close together. We delay a few microseconds
 * after each transfer to give the bridge time to finish
 * processing.
 */
#define XAPEA00X_BR_USB_DELAY			250 // usecs

#define XAPEA00X_BR_CS_DISABLED		0x00

/*******************************************************************************
 * Bridge USB transfers
 */

struct xapea00x_br_bulk_command {
	__u16  reserved1;
	__u8   command;
	__u8   reserved2;
	__le32 length;
} __attribute__((__packed__));

/**
 * xapea00x_br_prep_bulk_command - Prepares the bulk command header with
 * the supplied values.
 * @hdr: pointer to header to prepare
 * @command: the command id for the command
 * @length: length in bytes of the command data
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static void xapea00x_br_prep_bulk_command(struct xapea00x_br_bulk_command *hdr,
					  u8 command, int length)
{
	hdr->reserved1 = 0;
	hdr->command   = command;
	hdr->reserved2 = 0;
	hdr->length    = __cpu_to_le32(length);
}

/**
 * xapea00x_br_bulk_write - Issues a bulk write to the bridge chip.
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
static int xapea00x_br_bulk_write(struct xapea00x_device *dev,
				  struct xapea00x_br_bulk_command *header,
				  const void *data, int len)
{
	u8 *buf;
	unsigned int pipe;
	int buf_len, actual_len, retval;

	buf_len = sizeof(struct xapea00x_br_bulk_command) + len;
	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto out;
	}

	memcpy(buf, header, sizeof(struct xapea00x_br_bulk_command));
	memcpy(buf + sizeof(struct xapea00x_br_bulk_command), data, len);

	pipe = usb_sndbulkpipe(dev->udev, dev->bulk_out->bEndpointAddress);
	retval = usb_bulk_msg(dev->udev, pipe, buf, buf_len, &actual_len,
			      XAPEA00X_BR_USB_TIMEOUT);
	if (retval)
		goto free_buf;

	udelay(XAPEA00X_BR_USB_DELAY);

free_buf:
	kzfree(buf);

out:
	return retval;
}

/**
 * xapea00x_br_bulk_read - Issues a bulk read to the bridge chip.
 * @dev: pointer to the device to read from
 * @data: pointer to the data read. Must be DMA capable (e.g.,
 *	  kmalloc-ed, not stack).
 * @len: length in bytes of the data to read
 *
 * Return: If successful, 0. Otherwise a negative error code.
 */
static int xapea00x_br_bulk_read(struct xapea00x_device *dev, void *data,
				 int len)
{
	unsigned int pipe;
	void *buf;
	int actual_len, retval;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto out;
	}

	pipe = usb_rcvbulkpipe(dev->udev, dev->bulk_in->bEndpointAddress);
	retval = usb_bulk_msg(dev->udev, pipe, buf, len, &actual_len,
			      XAPEA00X_BR_USB_TIMEOUT);

	if (retval)
		goto free_buf;

	memcpy(data, buf, actual_len);
	udelay(XAPEA00X_BR_USB_DELAY);

free_buf:
	kzfree(buf);

out:
	return retval;
}

/**
 * xapea00x_br_ctrl_write - Issues a send control transfer to the bridge
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
static int xapea00x_br_ctrl_write(struct xapea00x_device *dev, u8 bRequest,
				  u16 wValue, u16 wIndex, u8 *data, u16 len)
{
	unsigned int pipe;
	void *buf;
	int retval;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		retval = -ENOMEM;
		goto out;
	}
	memcpy(buf, data, len);

	pipe = usb_sndctrlpipe(dev->udev, 0);
	retval = usb_control_msg(dev->udev, pipe, bRequest,
				 XAPEA00X_BR_BREQTYP_SET, wValue, wIndex,
				 buf, len, XAPEA00X_BR_USB_TIMEOUT);
	if (retval < 0)
		goto free_buf;

	udelay(XAPEA00X_BR_USB_DELAY);
	retval = 0;

free_buf:
	kzfree(buf);

out:
	return retval;
}

/*******************************************************************************
 * Bridge configuration commands
 */

/**
 * xapea00x_br_set_gpio_value - Sets the value on the specified pin of
 * the bridge chip.
 * @dev: pointer to the device containing the bridge whose pin to set
 * @pin: the number of the pin to set
 * @value: the value to set the pin to, 0 or 1
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_br_set_gpio_value(struct xapea00x_device *dev, u8 pin,
				      u8 value)
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
		break;
	}
	return xapea00x_br_ctrl_write(dev, XAPEA00X_BR_BREQ_SET_GPIO_VALUES,
				      0, 0, data, 4);
}

/**
 * xapea00x_br_set_gpio_cs - Sets the chip select control on the specified
 * pin of the bridge chip.
 * @dev: pointer to the device containing the bridge whose cs to set
 * @pin: the number of the pin to set
 * @control: the chip select control value for the pin, 0, 1, or 2
 *
 * Context: !in_interrupt()
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int xapea00x_br_set_gpio_cs(struct xapea00x_device *dev, u8 pin,
				   u8 control)
{
	u8 data[2] = { pin, control };

	return xapea00x_br_ctrl_write(dev, XAPEA00X_BR_BREQ_SET_GPIO_CS,
				      0, 0, data, 2);
}

/*******************************************************************************
 * Bridge configuration commands
 */
/**
 * xapea00x_br_disable_cs - disable the built-in chip select
 * capability of the specified channel. It does not support holding
 * the CS active between SPI transfers, a feature required for the
 * TPM. Instead, we manually control the CS pin as a GPIO.
 * @dev: pointer to the device containing the bridge whose cs to disable
 * @channel: the SPI channel whose cs to disable
 *
 * Context: !in_interrupt()
 *
 * Return: If successful 0. Otherwise a negative error number.
 */
int xapea00x_br_disable_cs(struct xapea00x_device *dev, u8 channel)
{
	return xapea00x_br_set_gpio_cs(dev, channel,
				       XAPEA00X_BR_CS_DISABLED);
}

/**
 * xapea00x_br_assert_cs - assert the chip select pin for the
 * specified channel.
 * @dev: pointer to the device containing the bridge who cs to assert
 * @channel: the SPI channel whose cs to assert
 *
 * Context: !in_interrupt()
 *
 * Return: If successful 0. Otherwise a negative error number.
 */
int xapea00x_br_assert_cs(struct xapea00x_device *dev, u8 channel)
{
	return xapea00x_br_set_gpio_value(dev, channel, 0);
}

/**
 * xapea00x_br_deassert_cs - deassert the chip select pin for the
 * specified channel.
 * @dev: pointer to the device containing the bridge who cs to deassert
 * @channel: the SPI channel whose cs to deassert
 *
 * Context: !in_interrupt()
 *
 * Return: If successful 0. Otherwise a negative error number.
 */
int xapea00x_br_deassert_cs(struct xapea00x_device *dev, u8 channel)
{
	return xapea00x_br_set_gpio_value(dev, channel, 1);
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
int xapea00x_br_spi_read(struct xapea00x_device *dev, void *rx_buf, int len)
{
	struct xapea00x_br_bulk_command header;
	int retval;

	xapea00x_br_prep_bulk_command(&header, XAPEA00X_BR_CMD_READ, len);
	retval = xapea00x_br_bulk_write(dev, &header, NULL, 0);
	if (retval)
		goto out;

	retval = xapea00x_br_bulk_read(dev, rx_buf, len);

out:
	return retval;
}

/**
 *xapea00x_br_spi_write - Performs a write to the active channel
 * @dev: pointer to the device to perform the write
 * @tx_buf: pointer to the data to write. Must be DMA-capable (e.g.,
 *	    kmalloc-ed, not stack).
 * @len: length in bytes of the data to write
 */
int xapea00x_br_spi_write(struct xapea00x_device *dev, const void *tx_buf,
			  int len)
{
	struct xapea00x_br_bulk_command header;
	int retval;

	xapea00x_br_prep_bulk_command(&header, XAPEA00X_BR_CMD_WRITE, len);
	retval = xapea00x_br_bulk_write(dev, &header, tx_buf, len);

	return retval;
}

/**
 * xapea00x_br_spi_write_read - Performs a simultaneous write and read on
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
int xapea00x_br_spi_write_read(struct xapea00x_device *dev, const void *tx_buf,
			       void *rx_buf, int len)
{
	struct xapea00x_br_bulk_command header;
	int retval;

	xapea00x_br_prep_bulk_command(&header, XAPEA00X_BR_CMD_WRITEREAD, len);
	retval = xapea00x_br_bulk_write(dev, &header, tx_buf, len);
	if (retval)
		goto out;

	retval = xapea00x_br_bulk_read(dev, rx_buf, len);

out:
	return retval;
}
