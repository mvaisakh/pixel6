#ifndef LWIS_DEVICE_H_
#define LWIS_DEVICE_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kernel.h>

/*
 *  struct lwis_core
 *  This struct applies to all LWIS devices that are defined in the
 *  device tree.
 */
struct lwis_core
{
	struct class *pclass;
	struct idr *pidr;
	struct cdev *pcdev;
	struct mutex lock;
	int device_major;
};

/*
 *  struct lwis_device
 *  This struct applies to each of the LWIS devices, e.g. /dev/lwisN
 */
struct lwis_device
{
	int id;
	struct device *pdev;
	struct platform_device *ppdev;
};

/*
 *  struct lwis_client
 *  This struct applies to each client that uses a LWIS device, i.e. each
 *  application that calls open() on a /dev/lwisN device.
 */
struct lwis_client
{
	struct mutex lock;
	struct lwis_device *pdev;
};

#endif  /* LWIS_DEVICE_H_ */