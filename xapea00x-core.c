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

#include "xapea00x.h"
#include "xapea00x-backports.h"

#define XAPEA00X_TPM_MODALIAS          "tpm_tis_spi"

#define kref_to_xapea00x(k) container_of(k, struct xapea00x_device, kref);

static void xapea00x_delete(struct kref *kref)
{
    struct xapea00x_device *dev = kref_to_xapea00x(kref);
    usb_put_dev(dev->udev);
    kfree(dev);
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

	mutex_lock(&dev->usb_mutex);
	if (!dev->interface) {
		retval = -ENODEV;
		goto out;
	}

	/* Verify that this is the TPM device */
	if (spi->chip_select != 0) {
		retval = -EINVAL;
		goto err;
	}
      
	/*
	 * Disable auto chip select for the TPM channel.
	 * Must be done after setting the SPI parameters.
	 */
	retval = xapea00x_br_disable_cs(dev, 0);
	if (retval)
		goto err;

	/* De-assert chip select for the TPM channel. */
	retval = xapea00x_br_deassert_cs(dev, 0);
	if (retval)
		goto err;

	dev_dbg(&dev->interface->dev, "configured spi channel for tpm\n");
	retval = 0;
	goto out;

err:
	dev_err(&dev->interface->dev,
	        "configuring SPI channel failed with %d\n", retval);

out:
	mutex_unlock(&dev->usb_mutex);
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
int xapea00x_spi_transfer(struct xapea00x_device *dev,
                          const void *tx_buf, void *rx_buf, u32 len,
                          int cs_hold, u16 delay_usecs)
{
	int retval;

	/* Assert chip select */
	retval = xapea00x_br_assert_cs(dev, 0);
	if (retval)
		goto out;

	/* empty transfer */
	if (!tx_buf && !rx_buf) {
		retval = 0;
	}
	/* read transfer */
	else if (!tx_buf) {
		retval = xapea00x_br_spi_read(dev, rx_buf, len);
	}
	/* write transfer */
	else if (!rx_buf) {
		retval = xapea00x_br_spi_write(dev, tx_buf, len);
	}
	/* write_read transfer */
	else {
		retval = xapea00x_br_spi_write_read(dev, tx_buf, rx_buf, len);
	}

	/* Deassert chip select, if requested */
	if (!cs_hold)
		retval = xapea00x_br_deassert_cs(dev, 0);

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

	mutex_lock(&dev->usb_mutex);
	if (!dev->interface) {
		retval = -ENODEV;
		goto out;
	}

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

	mutex_unlock(&dev->usb_mutex);
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

	spi_master = spi_alloc_master(&dev->interface->dev, sizeof(void*));
	if (!spi_master) {
		retval = -ENOMEM;
		goto err_out;
	}

	spi_master_set_devdata(spi_master, dev);

	spi_master->min_speed_hz = 93 * 1000 + 800; /* 93.9kHz */
	spi_master->max_speed_hz = 12 * 1000 * 1000; /* 12 MHz */

	spi_master->bus_num = -1; /* dynamically assigned */
	spi_master->num_chipselect = 1;
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

struct xapea00x_async_probe {
	struct work_struct work;
	struct xapea00x_device *dev;
};

#define work_to_probe(w) container_of(w, struct xapea00x_async_probe, work)

/**
 * xapea00x_init_async_probe - initialize an async probe with the
 * specified values.
 * @probe: pointer to the async_probe to initialize
 * @dev: pointer to the device to probe
 * @f: pointer to the probe function
 */
static void xapea00x_init_async_probe(struct xapea00x_async_probe *probe,
                                      struct xapea00x_device *dev,
                                      void (*f)(struct work_struct *work))
{
	INIT_WORK(&probe->work, f);
	probe->dev = dev;

	kref_get(&dev->kref);
	spi_master_get(dev->spi_master);
}

/**
 * xapea00x_free_async_probe - clean up the internals of the async
 * probe. Call this method after the probe has completed.
 *
 * The caller is responsible for freeing the probe itself, if
 * dynamically allocated.
 *
 * @probe: pointer to the async_probe to clean up
 */
static void xapea00x_cleanup_async_probe(struct xapea00x_async_probe *probe)
{
	spi_master_put(probe->dev->spi_master);
	kref_put(&probe->dev->kref, xapea00x_delete);
}

static struct spi_board_info tpm_board_info = {
	.modalias        = XAPEA00X_TPM_MODALIAS,
	.max_speed_hz    = 43 * 1000 * 1000, // Hz
	.chip_select     = 0,
	.mode            = SPI_MODE_0,
	.platform_data   = NULL,
	.controller_data = NULL,
};

/**
 * xapea00x_tpm_probe - Register and initialize the TPM device
 * @work: the work struct contained by the xapea00x device
 *
 * Context: !in_interrupt()
 */
static void xapea00x_tpm_probe(struct work_struct *work)
{
	struct xapea00x_async_probe *probe = work_to_probe(work);
	struct xapea00x_device *dev = probe->dev;
	struct spi_master *spi_master = dev->spi_master;
	struct spi_device *tpm;
	int retval;

	mutex_lock(&dev->usb_mutex);
	if (!dev->interface) {
		retval = -ENODEV;
		goto out;
	}
	/*
	 * This driver is the "platform" in TPM terminology. Before
	 * passing control of the TPM to the Linux TPM subsystem, do
	 * the TPM initialization normally done by the platform code
	 * (e.g., BIOS).
	 */
	retval = xapea00x_tpm_platform_initialize(dev);
	if (retval) {
		dev_err(&dev->interface->dev,
		        "unable to do TPM platform initialization: %d\n", retval);
		goto err;
	}

	/*
	 * Now register the TPM with the Linux TPM subsystem.  This
	 * may call through to xapea00x_spi_transfer_one_message(), so
	 * don't hold usb_mutex here.
	*/
	mutex_unlock(&dev->usb_mutex);
	tpm = spi_new_device(spi_master, &tpm_board_info);
	mutex_lock(&dev->usb_mutex);
	if (!dev->interface) {
		retval = -ENODEV;
		goto out;
	}
	if (!tpm) {
		dev_err(&dev->interface->dev,
		        "unable to add spi device for TPM\n");
		goto err;
	}

	dev->tpm = tpm;
	dev_info(&dev->interface->dev, "TPM initialization complete\n");
	goto out;

err:
	dev_err(&dev->interface->dev,
	        "TPM initialization failed with %d\n", retval);

out:
	mutex_unlock(&dev->usb_mutex);
	xapea00x_cleanup_async_probe(probe);
	kzfree(probe);
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

static int xapea00x_probe(struct usb_interface *interface,
                          const struct usb_device_id *id)
{
	struct xapea00x_device *dev;
	struct xapea00x_async_probe *probe;
	int retval;

	dev = kzalloc(sizeof(struct xapea00x_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	mutex_init(&dev->usb_mutex);

	/* ---------------------- USB ------------------------ */
	dev->interface = interface;
	dev->udev = usb_get_dev(interface_to_usbdev(interface));

	dev->vid = __le16_to_cpu(dev->udev->descriptor.idVendor);
	dev->pid = __le16_to_cpu(dev->udev->descriptor.idProduct);

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
	probe = kzalloc(sizeof(struct xapea00x_async_probe), GFP_KERNEL);
	xapea00x_init_async_probe(probe, dev, xapea00x_tpm_probe);

	schedule_work(&probe->work);
	dev_info(&interface->dev, "scheduled initialization of TPM\n");

	/* ---------------------- Finished ------------------------ */
	dev_info(&interface->dev, "device connected\n");
	return 0;

err_out:
	dev_err(&interface->dev, "device failed with %d\n", retval);
	kref_put(&dev->kref, xapea00x_delete);
	return retval;
}

static void xapea00x_disconnect(struct usb_interface *interface)
{
	struct xapea00x_device *dev = usb_get_intfdata(interface);
        usb_set_intfdata(interface, NULL);

	spi_unregister_master(dev->spi_master);

	mutex_lock(&dev->usb_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->usb_mutex);

	kref_put(&dev->kref, xapea00x_delete);

	dev_info(&dev->udev->dev, "device disconnected\n");
}

static struct usb_driver xapea00x_driver = {
	.name                 = "xapea00x",
	.probe                = xapea00x_probe,
	.disconnect           = xapea00x_disconnect,
	.suspend              = NULL,
	.resume               = NULL,
	.reset_resume         = NULL,
	.id_table             = xapea00x_devices,
	.supports_autosuspend = 0
};

module_usb_driver(xapea00x_driver);

MODULE_AUTHOR("David R. Bild <david.bild@xaptum.com>");
MODULE_DESCRIPTION("Xaptum XAP-EA-00x ENF Access card");
MODULE_LICENSE("GPL");

MODULE_ALIAS("xapea00x");
