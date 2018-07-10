/*
 * Renesas R-Car 3 ADC
 *
 * Copyright 2018 CogentEmbedded, Inc.
 *
 * based on linux/drivers/iio/vf610_adc.c
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* This will be the driver name the kernel reports */
#define DRIVER_NAME "rcar-sadc"

#define RCAR_ADPHYS		0x00
#define RCAR_ADFLAG		0x04
#define RCAR_ADEMSK		0x08
#define RCAR_ADECLR		0x0c
#define RCAR_ADFIFOEN		0x10
#define RCAR_ADFIFORST		0x14
#define RCAR_ADFIFOSTS		0x18
#define RCAR_ADFIFORC		0x1c
#define RCAR_ADCHSELP		0x20
#define RCAR_ADMON(x)		(0x24 + 0x4 * (x))
#define ADMONSET(x)		(0x4c + 0x4 * (x))
#define RCAR_ADCTRANS(x)	(0x74 + 0x4 * (x))
#define RCAR_ADCLASTREG		0x98

#define RCAR_ALL_CHANS		0x3ff

#define RCAR_ADC_TIMEOUT	msecs_to_jiffies(100)

struct rcar_adc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;

	u32 vref_uv;
	struct regulator *vref;

	struct completion completion;
};

static int rcar_chan_to_idx(int ch)
{
	/* regular channels starts from 1 */
	if ((ch >= 0) && (ch <= 7))
		return ch + 1;
	else if (ch == 8)
		return 0;	/* VDD1 */
	else
		return 9;	/* VDD2 */
}

static void rcar_adc_hw_init(struct rcar_adc *info)
{
	uint32_t reg;

	/* ADCHSELP register should be set 32'H0000 0030 before measuring */
	writel(0x30, info->regs + RCAR_ADCHSELP);
	/* reset all fifo */
	writel(RCAR_ALL_CHANS, info->regs + RCAR_ADFIFORST);
	/* enable all channels */
	writel(RCAR_ALL_CHANS, info->regs + RCAR_ADFIFOEN);
	/* AD FIFO read command register */
	writel(RCAR_ALL_CHANS, info->regs + RCAR_ADFIFORC);

	reg = readl(info->regs + RCAR_ADPHYS);
	writel(reg | (1 << 28) | (1 << 24), info->regs + RCAR_ADPHYS);
}

#define RCAR_ADC_CHAN(_idx, _chan_type) {			\
	.type = (_chan_type),					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = (_idx),			\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = 12,				\
		.storagebits = 16,			\
	},						\
}

static const struct iio_chan_spec rcar_adc_iio_channels[] = {
	/* external */
	RCAR_ADC_CHAN(0, IIO_VOLTAGE),
	RCAR_ADC_CHAN(1, IIO_VOLTAGE),
	RCAR_ADC_CHAN(2, IIO_VOLTAGE),
	RCAR_ADC_CHAN(3, IIO_VOLTAGE),
	RCAR_ADC_CHAN(4, IIO_VOLTAGE),
	RCAR_ADC_CHAN(5, IIO_VOLTAGE),
	RCAR_ADC_CHAN(6, IIO_VOLTAGE),
	RCAR_ADC_CHAN(7, IIO_VOLTAGE),
	/* internal */
	RCAR_ADC_CHAN(8, IIO_VOLTAGE),	/* VDD1 */
	RCAR_ADC_CHAN(9, IIO_VOLTAGE),	/* VDD2 */
	/* sentinel */
};

static irqreturn_t rcar_adc_isr(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = (struct iio_dev *)dev_id;
	struct rcar_adc *info = iio_priv(indio_dev);

	writel(RCAR_ALL_CHANS, info->regs + RCAR_ADECLR);

	return IRQ_HANDLED;
}

static int rcar_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{
	u32 value;
	struct rcar_adc *info = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		value = readl(info->regs +
			      RCAR_ADMON(rcar_chan_to_idx(chan->channel)))
			& 0x0fff;
		switch (chan->type) {
		case IIO_VOLTAGE:
			*val = value;
			break;
		default:
			return -EINVAL;
		}

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = info->vref_uv / 1000;
		*val2 = 12;	/* 12 bit mode */
		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		break;
	}

	return -EINVAL;
}

static int rcar_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	return 0;
}

static int rcar_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	return iio_triggered_buffer_predisable(indio_dev);
}

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.postenable = &rcar_adc_buffer_postenable,
	.predisable = &rcar_adc_buffer_predisable,
	.validate_scan_mask = &iio_validate_scan_mask_onehot,
};

static int rcar_adc_reg_access(struct iio_dev *indio_dev,
			unsigned reg, unsigned writeval,
			unsigned *readval)
{
	struct rcar_adc *info = iio_priv(indio_dev);

	if ((readval == NULL) ||
		((reg % 4) || (reg > RCAR_ADCLASTREG)))
		return -EINVAL;

	*readval = readl(info->regs + reg);

	return 0;
}

static const struct iio_info rcar_adc_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &rcar_adc_read_raw,
	.debugfs_reg_access = &rcar_adc_reg_access,
};

static const struct of_device_id rcar_adc_match[] = {
	{ .compatible = "renesas,sadc-r8a77970", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rcar_adc_match);

static int rcar_adc_probe(struct platform_device *pdev)
{
	struct rcar_adc *info;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct rcar_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	info->dev = &pdev->dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	ret = devm_request_irq(info->dev, irq,
			       rcar_adc_isr, 0,
			       dev_name(&pdev->dev), indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq: %d\n", irq);
		return ret;
	}

	info->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed getting clock: %ld\n",
			PTR_ERR(info->clk));
		return PTR_ERR(info->clk);
	}

	info->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(info->vref)) {
		dev_err(&pdev->dev, "failed getting regulator: %ld\n",
			PTR_ERR(info->vref));
		return PTR_ERR(info->vref);
	}

	ret = regulator_enable(info->vref);
	if (ret) {
		dev_err(&pdev->dev, "failed enabling regulator: %d\n",
			ret);
		return ret;
	}

	ret = regulator_get_voltage(info->vref);
	if (ret <= 0) {
		dev_err(&pdev->dev, "failed getting regulator voltage: %d\n",
			ret);
		return ret;
	}
	info->vref_uv = ret;

	platform_set_drvdata(pdev, indio_dev);

	init_completion(&info->completion);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &rcar_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = rcar_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(rcar_adc_iio_channels);

	ret = clk_prepare_enable(info->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock: %d\n",
			ret);
		goto error_adc_clk_enable;
	}

	rcar_adc_hw_init(info);

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					 NULL, &iio_triggered_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialise the buffer\n");
		goto error_iio_device_register;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_adc_buffer_init;
	}

	return 0;

error_adc_buffer_init:
	iio_triggered_buffer_cleanup(indio_dev);
error_iio_device_register:
	clk_disable_unprepare(info->clk);
error_adc_clk_enable:
	regulator_disable(info->vref);

	return ret;
}

static int rcar_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct rcar_adc *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(info->vref);
	clk_disable_unprepare(info->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rcar_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct rcar_adc *info = iio_priv(indio_dev);

	clk_disable_unprepare(info->clk);
	regulator_disable(info->vref);

	return 0;
}

static int rcar_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct rcar_adc *info = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(info->vref);
	if (ret)
		return ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		goto disable_reg;

	rcar_adc_hw_init(info);

	return 0;

disable_reg:
	regulator_disable(info->vref);
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(rcar_adc_pm_ops, rcar_adc_suspend, rcar_adc_resume);

static struct platform_driver rcar_adc_driver = {
	.probe          = rcar_adc_probe,
	.remove         = rcar_adc_remove,
	.driver         = {
		.name   = DRIVER_NAME,
		.of_match_table = rcar_adc_match,
		.pm     = &rcar_adc_pm_ops,
	},
};

module_platform_driver(rcar_adc_driver);

MODULE_AUTHOR("Andrey Gusakov <andrey.gusakov@cogentembedded.com>");
MODULE_DESCRIPTION("Renesas R-Car V3M SADC driver");
MODULE_LICENSE("GPL v2");
