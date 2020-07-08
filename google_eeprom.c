/*
 * Copyright 2018 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/nvmem-consumer.h>
#include "google_bms.h"

#define BATT_TOTAL_HIST_LEN	928
#define BATT_ONE_HIST_LEN	28
#define BATT_MAX_HIST_CNT	\
		(BATT_TOTAL_HIST_LEN / BATT_ONE_HIST_LEN) // 33.14

#define BATT_EEPROM_TAG_BRID_OFFSET	0x17
#define BATT_EEPROM_TAG_BRID_LEN	1
#define BATT_EEPROM_TAG_MINF_OFFSET	0x00
#define BATT_EEPROM_TAG_MINF_LEN	GBMS_MINF_LEN
#define BATT_EEPROM_TAG_DINF_OFFSET	0x20
#define BATT_EEPROM_TAG_DINF_LEN	GBMS_DINF_LEN
#define BATT_EEPROM_TAG_BCNT_OFFSET	0x40
#define BATT_EEPROM_TAG_BCNT_LEN	(GBMS_CCBIN_BUCKET_COUNT * 2)
#define BATT_EEPROM_TAG_GMSR_OFFSET	0x54
#define BATT_EEPROM_TAG_GMSR_LEN	GBMS_GMSR_LEN
#define BATT_EEPROM_TAG_HIST_OFFSET	0x6A
#define BATT_EEPROM_TAG_HIST_LEN	BATT_ONE_HIST_LEN
#define BATT_EEPROM_TAG_BGPN_OFFSET	0x03
#define BATT_EEPROM_TAG_BGPN_LEN	GBMS_BGPN_LEN

static int gbee_storage_info(gbms_tag_t tag, size_t *addr, size_t *count,
			    void *ptr)
{
	int ret = 0;

	switch (tag) {
	case GBMS_TAG_MINF:
		*addr = BATT_EEPROM_TAG_MINF_OFFSET;
		*count = BATT_EEPROM_TAG_MINF_LEN;
		break;
	case GBMS_TAG_DINF:
		*addr = BATT_EEPROM_TAG_DINF_OFFSET;
		*count = BATT_EEPROM_TAG_DINF_LEN;
		break;
	case GBMS_TAG_HIST:
		*addr = BATT_EEPROM_TAG_HIST_OFFSET;
		*count = BATT_EEPROM_TAG_HIST_LEN;
		break;
	case GBMS_TAG_SNUM:
	case GBMS_TAG_BGPN:
		*addr = BATT_EEPROM_TAG_BGPN_OFFSET;
		*count = BATT_EEPROM_TAG_BGPN_LEN;
		break;
	case GBMS_TAG_GMSR:
		*addr = BATT_EEPROM_TAG_GMSR_OFFSET;
		*count = BATT_EEPROM_TAG_GMSR_LEN;
		break;
	case GBMS_TAG_BCNT:
		*addr = BATT_EEPROM_TAG_BCNT_OFFSET;
		*count = BATT_EEPROM_TAG_BCNT_LEN;
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static int gbee_storage_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static gbms_tag_t keys[] = { GBMS_TAG_BGPN, GBMS_TAG_MINF,
				     GBMS_TAG_DINF, GBMS_TAG_HIST,
				     GBMS_TAG_BRID, GBMS_TAG_SNUM,
				     GBMS_TAG_GMSR, GBMS_TAG_BCNT };
	const int count = ARRAY_SIZE(keys);

	if (index < 0 || index >= count)
		return -ENOENT;

	*tag = keys[index];
	return 0;
}

static int gbee_storage_read(gbms_tag_t tag, void *buff, size_t size,
			    void *ptr)
{
	struct nvmem_device *nvmem = ptr;
	size_t offset = 0, len = 0;
	int ret;

	if (tag == GBMS_TAG_BRID) {
		u8 temp;

		if (size != sizeof(u32))
			return -ENOMEM;

		ret = nvmem_device_read(nvmem, BATT_EEPROM_TAG_BRID_OFFSET,
					1, &temp);
		pr_err("cannot read nvram %d\n", ret);
		if (ret < 0)
			return ret;

		((u32*)buff)[0] = temp;
		return len;
	}

	ret = gbee_storage_info(tag, &offset, &len, ptr);
	if (ret < 0)
		return ret;
	if (!len)
		return -ENOENT;
	if (len > size)
		return -ENOMEM;

	ret = nvmem_device_read(nvmem, offset, len, buff);
	if (ret == 0)
		ret = len;

	return ret;
}

static int gbee_storage_write(gbms_tag_t tag, const void *buff, size_t size,
			      void *ptr)
{
	struct nvmem_device *nvmem = ptr;
	size_t offset = 0, len = 0;
	int ret;

	if ((tag != GBMS_TAG_DINF) && (tag != GBMS_TAG_GMSR) &&
	    (tag != GBMS_TAG_BCNT))
		return -ENOENT;

	ret = gbee_storage_info(tag, &offset, &len, ptr);
	if (ret < 0)
		return ret;
	if (size > len)
		return -ENOMEM;

	ret = nvmem_device_write(nvmem, offset, size, (void *)buff);
	if (ret == 0)
		ret = size;

	return ret;
}

static int gbee_storage_read_data(gbms_tag_t tag, void *data, size_t count,
				  int idx, void *ptr)
{
	struct nvmem_device *nvmem = ptr;
	size_t offset = 0, len = 0;
	int ret;

	switch (tag) {
	case GBMS_TAG_HIST:
		ret = gbee_storage_info(tag, &offset, &len, ptr);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	if (ret < 0)
		return ret;

	if (!data || !count) {
		if (idx == GBMS_STORAGE_INDEX_INVALID)
			return 0;
		else
			return BATT_MAX_HIST_CNT;
	}

	if (idx < 0)
		return -EINVAL;

	/* index == 0 is ok here */
	if (idx >= BATT_MAX_HIST_CNT)
		return -ENODATA;

	if (len > count)
		return -EINVAL;

	offset += len * idx;

	ret = nvmem_device_read(nvmem, offset, len, data);
	if (ret == 0)
		ret = len;

	return ret;
}

static int gbee_storage_write_data(gbms_tag_t tag, const void *data,
				   size_t count, int idx, void *ptr)
{
	struct nvmem_device *nvmem = ptr;
	size_t offset = 0, len = 0;
	int ret;

	switch (tag) {
	case GBMS_TAG_HIST:
		ret = gbee_storage_info(tag, &offset, &len, ptr);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	if (ret < 0)
		return ret;

	if (idx < 0 || !data || !count)
		return -EINVAL;

	/* index == 0 is ok here */
	if (idx >= BATT_MAX_HIST_CNT)
		return -ENODATA;

	if (count > len)
		return -EINVAL;

	offset += len * idx;

	ret = nvmem_device_write(nvmem, offset, len, (void *)data);
	if (ret == 0)
		ret = len;

	return ret;
}


static struct gbms_storage_desc gbee_storage_dsc = {
	.info = gbee_storage_info,
	.iter = gbee_storage_iter,
	.read = gbee_storage_read,
	.write = gbee_storage_write,
	.read_data = gbee_storage_read_data,
	.write_data = gbee_storage_write_data,
};

/*
 * Caller will use something like of_nvmem_device_get() to retrieve the
 * nvmem_device instance.
 * TODO: map nvram cells to tags
 */
int gbee_register_device(const char *name, struct nvmem_device *nvram)
{
	return gbms_storage_register(&gbee_storage_dsc, name, nvram);
}

void gbee_destroy_device(void)
{

}
