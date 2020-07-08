/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MAX77759 MAXQ interface.
 */

#ifdef CONFIG_MAXQ_MAX77759
struct max77759_maxq;
extern struct max77759_maxq *maxq_init(struct device *dev,
				       struct regmap *regmap,
				       bool poll);
extern void maxq_remove(struct max77759_maxq *maxq);
extern void maxq_irq(struct max77759_maxq *maxq);
/* Helpers */
extern int maxq_query_contaminant(struct max77759_maxq *maxq, u8 cc1_raw,
				  u8 cc2_raw, u8 sbu1_raw,
				  u8 sbu2_raw, u8 cc1_rd, u8 cc2_rd,
				  u8 type, u8 cc_adc_skipped,
				  u8 *response, u8 response_len);
extern int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio);
extern int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio);
# else
static inline struct max77759_maxq *maxq_init(struct device *dev,
					      struct regmap *regmap,
					      bool poll)
{
	return -EINVAL;
}
static inline void maxq_remove(struct max77759_maxq *maxq)
{
	return -EINVAL;
}
static inline void maxq_irq(struct max77759_maxq *maxq)
{
	return -EINVAL;
}
/* Helpers */
static inline int maxq_query_contaminant(struct max77759_maxq *maxq,
					 u8 cc1_raw, u8 cc2_raw,
					 u8 sbu1_raw, u8 sbu2_raw,
					 u8 cc1_rd, u8 cc2_rd,
					 u8 type, u8 cc_adc_skipped,
					 u8 *response, u8 response_len)
{
	return -EINVAL;
}
extern int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio)
{
	return -EINVAL;
}
extern int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio)
{
	return -EINVAL;
}
#endif
