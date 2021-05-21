/* XAP-EA-00x driver for Linux
 *
 *  Copyright (c) 2017-2020 Xaptum, Inc.
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

#define XAPEA00X_TPM_MODALIAS	"tpm_tis_spi"

#define kref_to_xapea00x(k) container_of(k, struct xapea00x_device, kref)

static void xapea00x_delete(struct kref *kref)
{
	struct xapea00x_device *dev = kref_to_xapea00x(kref);

	usb_put_dev(dev->udev);
	kfree(dev);
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
 * xapea00x_cleanup_async_probe - clean up the internals of the async
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
	.modalias	= XAPEA00X_TPM_MODALIAS,
	.max_speed_hz	= 43 * 1000 * 1000, // Hz
	.chip_select	= 0,
	.mode		= SPI_MODE_0
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
			"unable to do TPM platform initialization: %d\n",
			retval);
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
		retval = -ENODEV;
		dev_err(&dev->interface->dev,
			"unable to add spi device for TPM\n");
		goto err;
	}

	dev->tpm = tpm;
	goto out;

err:
	dev_err(&dev->interface->dev,
		"TPM initialization failed with %d\n", retval);

out:
	mutex_unlock(&dev->usb_mutex);
	xapea00x_cleanup_async_probe(probe);
	kzfree(probe);
}

/*******************************************************************************
 * USB driver structs and functions
 */

static const struct usb_device_id xapea00x_devices[] = {
	{ USB_DEVICE(USB_VENDOR_ID_SILABS, USB_PRODUCT_ID_XAPEA001) },
	{ USB_DEVICE(USB_VENDOR_ID_XAPTUM, USB_PRODUCT_ID_XAPEA002) },
	{ USB_DEVICE(USB_VENDOR_ID_XAPTUM, USB_PRODUCT_ID_XAPEA003) },
	{ USB_DEVICE(USB_VENDOR_ID_XAPTUM, USB_PRODUCT_ID_XAPEA004) },
	{ }
};
MODULE_DEVICE_TABLE(usb, xapea00x_devices);

static int xapea00x_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	struct xapea00x_device *dev;
	struct xapea00x_async_probe *probe;
	int retval;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
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
		goto free_dev;
	}

	usb_set_intfdata(interface, dev);

	/* ---------------------- SPI Master ------------------------ */
	retval = xapea00x_spi_probe(dev);
	if (retval) {
		dev_err(&interface->dev, "could not initialize SPI master\n");
		goto free_dev;
	}

	/* ---------------------- TPM SPI Device ------------------------ */
	probe = kzalloc(sizeof(*probe), GFP_KERNEL);
	if (!probe) {
		retval = -ENOMEM;
		goto free_spi;
	}
	xapea00x_init_async_probe(probe, dev, xapea00x_tpm_probe);

	schedule_work(&probe->work);

	/* ---------------------- Finished ------------------------ */
	return 0;

free_spi:
	spi_unregister_master(dev->spi_master);

free_dev:
	kref_put(&dev->kref, xapea00x_delete);

	dev_err(&interface->dev, "device failed with %d\n", retval);
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
}

static struct usb_driver xapea00x_driver = {
	.name		= KBUILD_MODNAME,
	.probe		= xapea00x_probe,
	.disconnect	= xapea00x_disconnect,
	.id_table	= xapea00x_devices
};

module_usb_driver(xapea00x_driver);

MODULE_AUTHOR("David R. Bild <david.bild@xaptum.com>");
MODULE_DESCRIPTION("Xaptum XAP-EA-00x ENF Access card");
MODULE_LICENSE("GPL");
