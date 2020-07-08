// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020, Google Inc
 *
 * MAX77759 MAXQ opcode management.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#include "logbuffer.h"
#include "max77759_regs.h"

#define PAYLOAD_REQUEST_LENGTH_BYTES		33
#define PAYLOAD_RESPONSE_LENGTH_BYTES		3
#define OPCODE_GPIO_CONTROL_READ		0x23
#define OPCODE_GPIO_CONTROL_R_REQ_LEN		1
#define OPCODE_GPIO_CONTROL_R_RES_LEN		2
#define OPCODE_GPIO_CONTROL_R_RES_OFFSET	1
#define OPCODE_GPIO_CONTROL_WRITE		0x24
#define OPCODE_GPIO_CONTROL_W_REQ_LEN		2
#define OPCODE_GPIO_CONTROL_W_REQ_OFFSET	1
#define OPCODE_GPIO_CONTROL_W_RES_LEN		1
#define OPCODE_CHECK_CC_AND_SBU			0x85
#define	OPCODE_CHECK_CC_AND_SBU_REQ_LEN		9
#define OPCODE_CHECK_CC_AND_SBU_RES_LEN		5
#define MAXQ_AP_DATAOUT0			0x81
#define MAXQ_AP_DATAOUT32			0xA1
#define MAXQ_AP_DATAIN0				0xB1
#define MAXQ_REPLY_TIMEOUT_MS			50

struct max77759_maxq {
	struct completion reply_done;
	/* Denotes the current request in progress. */
	unsigned int req_no;
	/* Updated by the irq handler. */
	unsigned int req_done;
	/* Protects req_no and req_done variables. */
	struct mutex req_lock;
	struct logbuffer *log;
	bool init_done;
	struct regmap *regmap;

	/* To make sure that there is only one request in progress. */
	struct mutex maxq_lock;
	u8 request_opcode;
	bool poll;
};

enum request_payload_offset {
	REQUEST_OPCODE,
	/* Following are specific to CHECK_CC_AND_SBU */
	REQUEST_TYPE,
	CC1RD,
	CC2RD,
	CCADCSKIPPED,
	CC1ADC,
	CC2ADC,
	SBU1ADC,
	SBU2ADC,
	WRITE_BYTES
};

enum response_payload_offset {
	RESPONSE_OPCODE,
	RESPONSE_TYPE,
	RESULT,
	CC_THRESH,
	SBU_THRESH
};

/* Caller holds req_lock */
static int maxq_read_response_locked(struct max77759_maxq *maxq,
				     u8 *response, u8 response_len)
{
	int ret = 0;

	ret = regmap_bulk_read(maxq->regmap, MAXQ_AP_DATAIN0, response,
			       response_len);
	if (ret) {
		logbuffer_log(maxq->log, "I2C request failed ret:%d", ret);
		return -EINVAL;
	}

	if (response[RESPONSE_OPCODE] != maxq->request_opcode) {
		logbuffer_log(maxq->log,
			      "OPCODE does not match request:%u response:%u",
			      maxq->request_opcode, response[RESPONSE_OPCODE]);
		return -EINVAL;
	}

	return ret;
}

/* Caller holds req_lock */
static int maxq_wait_for_response_locked(struct max77759_maxq *maxq,
					 unsigned int current_request,
					 u8 *response, u8 response_len)
{
	int ret = 0;
	unsigned long timeout = MAXQ_REPLY_TIMEOUT_MS;

	/* Wait for response from Maxq */
	if (maxq->poll) {
		msleep(timeout);
		ret = maxq_read_response_locked(maxq, response, response_len);
		goto exit;
	}

	/* IRQ from MAXQ */
	while (timeout) {
		mutex_unlock(&maxq->req_lock);
		timeout = wait_for_completion_timeout(&maxq->reply_done,
						      timeout);
		mutex_lock(&maxq->req_lock);
		if (current_request == maxq->req_done) {
			ret = maxq_read_response_locked(maxq, response,
							response_len);
			break;
		} else if (!timeout) {
			logbuffer_log(maxq->log,
				      "Timeout or Request number does not match current:req:%u req_done:%u",
				      current_request, maxq->req_done);
			ret = -EINVAL;
		}
	}

exit:
	logbuffer_log(maxq->log, "MAXQ DONE: current_req: %u ret:%d",
		      current_request, ret);
	return ret;
}

int maxq_issue_opcode_command(struct max77759_maxq *maxq, u8 *request,
			      u8 request_length, u8 *response,
			      u8 response_length)
{
	unsigned int current_request;
	int ret = -ENODEV;

	if (!maxq->init_done)
		return ret;

	mutex_lock(&maxq->maxq_lock);
	maxq->request_opcode = request[REQUEST_OPCODE];
	mutex_lock(&maxq->req_lock);
	current_request = ++maxq->req_no;

	logbuffer_log(maxq->log, "MAXQ REQ:current_req: %u opcode:%u",
		      current_request, maxq->request_opcode);
	ret = regmap_bulk_write(maxq->regmap, MAXQ_AP_DATAOUT0, request,
				request_length);
	if (ret) {
		logbuffer_log(maxq->log, "I2C request write failed");
		goto req_unlock;
	}

	if (request_length < PAYLOAD_REQUEST_LENGTH_BYTES) {
		ret = regmap_write(maxq->regmap, MAXQ_AP_DATAOUT32, 0);
		if (ret) {
			logbuffer_log(maxq->log,
				      "I2C request write DATAOUT32 failed");
			goto req_unlock;
		}
	}

	ret = maxq_wait_for_response_locked(maxq, current_request,
					    response, response_length);
req_unlock:
	mutex_unlock(&maxq->req_lock);
	mutex_unlock(&maxq->maxq_lock);
	return ret;
}

void maxq_irq(struct max77759_maxq *maxq)
{
	mutex_lock(&maxq->req_lock);
	maxq->req_done = maxq->req_no;
	logbuffer_log(maxq->log, "MAXQ IRQ: req_done: %u", maxq->req_done);
	complete(&maxq->reply_done);
	mutex_unlock(&maxq->req_lock);
}
EXPORT_SYMBOL_GPL(maxq_irq);

int maxq_query_contaminant(struct max77759_maxq *maxq, u8 cc1_raw,
			   u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw, u8 cc1_rd,
			   u8 cc2_rd, u8 type, u8 cc_adc_skipped,
			   u8 *response, u8 response_len)
{
	int ret;
	u8 payload[OPCODE_CHECK_CC_AND_SBU_REQ_LEN];

	payload[REQUEST_OPCODE] = OPCODE_CHECK_CC_AND_SBU;
	payload[REQUEST_TYPE] = type;
	payload[CC1RD] = cc1_rd;
	payload[CC2RD] = cc2_rd;
	payload[CCADCSKIPPED] = cc_adc_skipped;
	payload[CC1ADC] = cc1_raw;
	payload[CC2ADC] = cc2_raw;
	payload[SBU1ADC] = sbu1_raw;
	payload[SBU2ADC] = sbu2_raw;

	logbuffer_log(maxq->log,
		      "MAXQ opcode:%#x type:%#x cc1_rd:%#x cc2_rd:%#x ADCSKIPPED:%#x cc1adc:%#x cc2adc:%#x sbu1adc:%#x sbu2adc:%#x",
		      OPCODE_CHECK_CC_AND_SBU, type, cc1_rd, cc2_rd,
		      cc_adc_skipped, cc1_raw, cc2_raw, sbu1_raw, sbu2_raw);
	ret = maxq_issue_opcode_command(maxq, payload,
					OPCODE_CHECK_CC_AND_SBU_REQ_LEN,
					response, response_len);
	if (!ret)
		logbuffer_log(maxq->log, "MAXQ Contaminant response:%u",
			      response[RESULT]);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_query_contaminant);

int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio)
{
	int ret;
	u8 request = OPCODE_GPIO_CONTROL_READ;
	u8 response[OPCODE_GPIO_CONTROL_R_RES_LEN];

	logbuffer_log(maxq->log, "MAXQ gpio read opcode:%#x", request);
	ret = maxq_issue_opcode_command(maxq, &request,
					OPCODE_GPIO_CONTROL_R_REQ_LEN,
					response,
					OPCODE_GPIO_CONTROL_R_RES_LEN);
	if (!ret)
		*gpio = response[OPCODE_GPIO_CONTROL_R_RES_OFFSET];
	logbuffer_log(maxq->log, "MAXQ GPIO:%#x", *gpio);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_control_read);

int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio)
{
	int ret;
	u8 request[OPCODE_GPIO_CONTROL_W_REQ_LEN];
	u8 response[OPCODE_GPIO_CONTROL_W_RES_LEN];

	request[REQUEST_OPCODE] = OPCODE_GPIO_CONTROL_WRITE;
	request[OPCODE_GPIO_CONTROL_W_REQ_OFFSET] = gpio;
	logbuffer_log(maxq->log, "MAXQ gpio write opcode:%#x val:%#x",
		      request[REQUEST_OPCODE],
		      request[OPCODE_GPIO_CONTROL_W_REQ_OFFSET]);
	ret = maxq_issue_opcode_command(maxq, request,
					OPCODE_GPIO_CONTROL_W_REQ_LEN,
					response,
					OPCODE_GPIO_CONTROL_W_RES_LEN);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_control_write);

struct max77759_maxq *maxq_init(struct device *dev, struct regmap *regmap,
				bool poll)
{
	struct max77759_maxq *maxq;

	maxq = devm_kzalloc(dev, sizeof(*maxq), GFP_KERNEL);
	if (IS_ERR_OR_NULL(maxq))
		return ERR_PTR(-ENOMEM);

	maxq->log = debugfs_logbuffer_register("maxq");
	if (IS_ERR_OR_NULL(maxq->log)) {
		dev_err(dev, "MAXQ logbuffer register failed\n");
		return (struct max77759_maxq *)maxq->log;
	}
	maxq->regmap = regmap;

	init_completion(&maxq->reply_done);
	mutex_init(&maxq->maxq_lock);
	mutex_init(&maxq->req_lock);
	maxq->poll = poll;
	maxq->init_done = true;

	logbuffer_log(maxq->log, "MAXQ: probe done");

	return maxq;
}
EXPORT_SYMBOL_GPL(maxq_init);

void maxq_remove(struct max77759_maxq *maxq)
{
	maxq->init_done = false;
	debugfs_logbuffer_unregister(maxq->log);
}
EXPORT_SYMBOL_GPL(maxq_remove);
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("Maxim 77759 MAXQ OPCDOE management");
MODULE_LICENSE("GPL");
