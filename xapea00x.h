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

struct xapea00x_device {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;
	u16 pid;
	u16 vid;

	struct spi_master *spi_master;
	struct spi_device *tpm;
	struct work_struct tpm_probe;
};

/* Public bridge functions */
int xapea00x_br_spi_read(struct xapea00x_device *dev, void* rx_buf, int len);
int xapea00x_br_spi_write(struct xapea00x_device *dev, const void* tx_buf,
                          int len);
int xapea00x_br_spi_write_read(struct xapea00x_device *dev, const void* tx_buf,
                               void* rx_buf, int len);
int xapea00x_br_set_gpio_value(struct xapea00x_device *dev, u8 pin, u8 value);
int xapea00x_br_set_gpio_cs(struct xapea00x_device *dev, u8 pin, u8 control);
int xapea00x_br_set_spi_word(struct xapea00x_device *dev, u8 channel, u8 word);

/* Shared SPI function */
int xapea00x_spi_transfer(struct xapea00x_device *dev,
                          const void *tx_buf, void *rx_buf, u32 len,
                          int cs_hold, u16 delay_usecs);

/* Shared TPM functions */
int xapea00x_tpm_platform_initialize(struct xapea00x_device *dev);

