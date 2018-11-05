/*************************************************************************/ /*
 IMR

 Copyright (C) 2015-2017 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/

/* PRQA S 292,2212,2214 EOF */
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>

#include <linux/of.h>
#include <linux/of_irq.h>

/* IMR uio driver name */
#define DRIVER_NAME "uio_imr"

/* IMR register definition */
#define IMR_REG_IMR_ADDRESS	(0x018U)
#define IMR_REG_IMR_BIT_BASE	(0x3U << 3)
#define IMR_REG_IMR_BIT_INT	(0x1U << 2)
#define IMR_REG_IMR_BIT_IER	(0x1U << 1)
#define IMR_REG_IMR_BIT_TRA	(0x1U << 0)

/**
 * struct uio_platdata - the uio platform data structure
 * @uioinfo:     UIO device capabilities
 * @lock	 lock flag for irq.
 * @flags:       flags for request_irq.
 * @pdev:	IMR platform device data.
 * @base_reg:    IMR base register address.
 * @clk:	 clock data.
 *
 * the uio platform data structure.
 */
struct uio_platdata {
	struct uio_info *uioinfo;
	spinlock_t lock;
	unsigned long flags;
	struct platform_device *pdev;
	void __iomem *base_reg;
	struct clk *clock;
};

static void write_register(struct uio_platdata *priv,
			   u32 reg_offs, u32 data);
static int uio_imr_open(struct uio_info *info,
			__attribute__((unused)) struct inode *inode);
static int uio_imr_release(struct uio_info *info,
			__attribute__((unused)) struct inode *inode);
static irqreturn_t uio_imr_handler(__attribute__((unused)) int irq,
				   struct uio_info *dev_info);
static int uio_imr_irqcontrol(struct uio_info *info, s32 irq_on);
static int uio_imr_probe(struct platform_device *pdev);
static int uio_imr_remove(struct platform_device *pdev);
static int uio_runtime_imr_nop(__attribute__((unused)) struct device *dev);

/**
 * write_register() - register setting
 * @priv:       uio platform data.
 * @reg_offs:   register offset.
 * @data:       register value.
 *
 * register setting.
 *
 *
 * Return: none
 */
static void write_register(struct uio_platdata *priv,
			   u32 reg_offs, u32 data)
{
	iowrite32(data, (u8 *)priv->base_reg + reg_offs); /* PRQA S 488 */
}

/**
 * uio_imr_open() - open imr module
 * @info:       UIO device capabilities.
 * @inode:      inode.
 *
 * Open imr module.
 *
 *
 * Return: 0   normal end.
 */
/* PRQA S 3206 2 */
static int uio_imr_open(struct uio_info *info,
			__attribute__((unused)) struct inode *inode)
{
	struct uio_platdata *pdata = info->priv;
	/* PRQA S 3200 1 */
	pr_debug("uio_imr_open enter. name=%s\n", pdata->uioinfo->name);

	/* Wait until the Runtime PM code has woken up the device */
	(void)pm_runtime_get_sync(&pdata->pdev->dev);

	return 0;
}

/**
 * uio_imr_release() - close imr module
 * @info:       UIO device capabilities.
 * @inode:      inode.
 *
 * Close imr module.
 *
 *
 * Return: 0   normal end.
 */
/* PRQA S 3206 2 */
static int uio_imr_release(struct uio_info *info,
			   __attribute__((unused)) struct inode *inode)
{
	struct uio_platdata *pdata = info->priv;

	pr_debug("uio_imr_release enter\n"); /* PRQA S 3200 */

	/* Tell the Runtime PM code that the device has become idle */
	(void)pm_runtime_put_sync(&pdata->pdev->dev);

	return 0;
}

/**
 * uio_imr_handler() - IMR interrupt handler
 * @irq:	irq No.
 * @dev_info:   UIO device capabilities.
 *
 * IMR interrupt handler.
 *
 *
 * Return: IRQ_HANDLED   normal end.
 */
/* PRQA S 3206 1*/
static irqreturn_t uio_imr_handler(__attribute__((unused))int irq,
				   struct uio_info *dev_info)
{
	struct uio_platdata *pdata = dev_info->priv;


	pr_debug("uio_imr_handler enter\n"); /* PRQA S 3200 */

	/* Mask interrupt */
	write_register(pdata, IMR_REG_IMR_ADDRESS,
		       IMR_REG_IMR_BIT_BASE | (IMR_REG_IMR_BIT_INT |
		       IMR_REG_IMR_BIT_IER | IMR_REG_IMR_BIT_TRA));

	return IRQ_HANDLED;
}

/**
 * uio_imr_irqcontrol() - IMR irq controller
 * @info:     UIO device capabilities.
 * @irq_on:       irq enable/disable.
 *
 * IMR irq controller. Enable and disable the interrupt.
 *
 *
 * Return: 0   normal end.
 */
static int uio_imr_irqcontrol(struct uio_info *info, s32 irq_on)
{
	struct uio_platdata *pdata = info->priv;
	u64 flag;

	pr_debug("uio_imr_irqcontrol enter\n"); /* PRQA S 3200 */

	spin_lock_irqsave(&pdata->lock, flag);
	if (irq_on != 0) {
		if (test_and_clear_bit(0, &pdata->flags) != 0)
			enable_irq((u32)info->irq);
	} else {
		if (test_and_set_bit(0, &pdata->flags) == 0)
			disable_irq((u32)info->irq);
	}
	spin_unlock_irqrestore(&pdata->lock, flag);

	return 0;
}

/* PRQA S 1053,1041,605 10 */
static const struct of_device_id rcar_imr_dt_ids[] = {
	{ .compatible = "renesas,r8a7795-imr-lx4", .data = 0 },
	{ .compatible = "renesas,r8a7796-imr-lx4", .data = 0 },
	{ .compatible = "renesas,r8a77970-imr-lx4", .data = 0 },
	{ .compatible = "renesas,r8a77980-imr-lx4", .data = 0 },
	{},
};
MODULE_DEVICE_TABLE(of, rcar_imr_dt_ids);  /* PRQA S 605 */

/**
 * uio_imr_probe() - Initialize IMR module.
 * @pdev:     platform device data.
 *
 * Initialize IMR module.
 *
 *
 * Return: 0         normal end.
 *         -EINVAL   parameter error.
 *         -ENOMEM   memory error.
 *         -ENODEV   system error.
 */
static int uio_imr_probe(struct platform_device *pdev)
{
	struct uio_info *uioinfo_data = NULL;
	struct uio_platdata *pdata;
	struct uio_mem *uiomem;
	int ret = 0;
	unsigned int i;
	struct resource *rsc;
	unsigned int irq_l;
	unsigned long remap_size;

	if (pdev == NULL) {
		pr_err("missing pdev\n");
		ret = -EINVAL;
	} else {
		if (pdev->dev.of_node == NULL) {
			dev_err(&pdev->dev, "missing pdev->dev.of_node\n");
			ret = -EINVAL;
		}
	}

	if (ret == 0) {
		pr_debug("uio_imr_probe enter name = %s\n",  /* PRQA S 3200 */
			 pdev->dev.of_node->name);

		uioinfo_data = devm_kzalloc(&pdev->dev,
					    sizeof(*uioinfo_data),
					    GFP_KERNEL);
		if (uioinfo_data == NULL)
			ret = -ENOMEM;
	}

	if (ret == 0) {
		uioinfo_data->name = pdev->dev.of_node->name;
		uioinfo_data->version = "0.1";

		/* get irq number */
		irq_l = irq_of_parse_and_map(pdev->dev.of_node, 0);
		if ((int)irq_l == -ENXIO)
			uioinfo_data->irq = platform_get_irq(pdev, 0);
		else
			uioinfo_data->irq = (int)irq_l;

		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			ret = -ENOMEM;
	}

	if (ret == 0) {
		pdata->uioinfo = uioinfo_data;
		spin_lock_init(&pdata->lock);  /* PRQA S 3200 */
		pdata->flags = 0;
		pdata->pdev = pdev;

		uiomem = &uioinfo_data->mem[0];

		for (i = 0; i < pdev->num_resources; ++i) {
			/* PRQA S 491 1 */
			struct resource *r = &pdev->resource[i];

			if (r->flags == IORESOURCE_IRQ) {
				uioinfo_data->irq = (long)r->start;
			} else if (r->flags != IORESOURCE_MEM) {
				;
			} else {
				if (uiomem >=
				    &uioinfo_data->mem[MAX_UIO_MAPS]) {
					dev_warn(&pdev->dev,
					  "device has more than "
					  __stringify(MAX_UIO_MAPS)
					  " I/O memory resources.\n");
					break;
				}

				uiomem->memtype = UIO_MEM_PHYS;
				uiomem->addr = r->start;
				uiomem->size = (r->end - r->start) + 1;
				++uiomem; /* PRQA S 489 */
			}
		}

		while (uiomem < &uioinfo_data->mem[MAX_UIO_MAPS]) {
			uiomem->size = 0;
			++uiomem; /* PRQA S 489 */
		}

		uioinfo_data->handler = &uio_imr_handler;
		uioinfo_data->irqcontrol = &uio_imr_irqcontrol;
		uioinfo_data->open = &uio_imr_open;
		uioinfo_data->release = &uio_imr_release;
		uioinfo_data->priv = pdata;

		pm_runtime_enable(&pdev->dev);

		ret = uio_register_device(&pdev->dev, pdata->uioinfo);
		if (ret != 0) {
			pm_runtime_disable(&pdev->dev);
			dev_err(&pdev->dev, "could not register uio device\n");
			ret = -ENODEV;
		}
	}

	if (ret == 0) {
		platform_set_drvdata(pdev, pdata);
		pdata->clock = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(pdata->clock)) {
			pm_runtime_disable(&pdev->dev);
			dev_err(&pdev->dev, "could not get clock\n");
			ret = -ENODEV;
		} else {
			/* clock enable */
			(void)clk_prepare_enable(pdata->clock);
		}
	}

	if (ret == 0) {
		rsc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (rsc == 0) {
			pm_runtime_disable(&pdev->dev);
			dev_err(&pdev->dev, "could not platform_get_resource\n");
			ret = -ENODEV;
		}
	}

	if (ret == 0) {
		remap_size = (rsc->end - rsc->start) + 1;
		if (!request_mem_region(rsc->start,
					remap_size,
					uioinfo_data->name)) {
			dev_err(&pdev->dev, "could not request IO\n");
			pm_runtime_disable(&pdev->dev);
			ret = -ENOMEM;
		}
	}

	if (ret == 0) {
		/* IMR Register Adderss */
		pdata->base_reg = devm_ioremap_nocache(&pdev->dev,
							rsc->start,
							remap_size);
		if (pdata->base_reg == NULL) {
			release_mem_region(rsc->start, resource_size(rsc));
			dev_err(&pdev->dev, "could not remap IMR register\n");
			pm_runtime_disable(&pdev->dev);
			ret = -ENOMEM;
		} else {
			/* PRQA S 3200 2 */
			pr_debug("IMR reg_base = %x size = %x\n",
				(uint32_t)(rsc->start), (uint32_t)remap_size);
		}
	}

	return ret;
}

/**
 * uio_imr_remove() - release IMR module.
 * @pdev:     platform device data.
 *
 * release IMR module.
 *
 *
 * Return: 0   normal end.
 */
static int uio_imr_remove(struct platform_device *pdev)
{
	struct resource *rsc;
	struct uio_platdata *pdata = platform_get_drvdata(pdev);

	/* PRQA S 3200 1 */
	pr_debug("uio_imr_remove enter name = %s\n", pdata->uioinfo->name);

	clk_disable_unprepare(pdata->clock);

	uio_unregister_device(pdata->uioinfo);

	pm_runtime_disable(&pdev->dev);

	irq_dispose_mapping((u32)pdata->uioinfo->irq);

	pdata->uioinfo->handler = NULL;
	pdata->uioinfo->irqcontrol = NULL;

	platform_set_drvdata(pdev, NULL);

	if (pdata->base_reg != NULL)
		devm_iounmap(&pdev->dev, pdata->base_reg);

	rsc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (rsc != 0)
		release_mem_region(rsc->start, resource_size(rsc));

	return 0;
}

/**
 * uio_runtime_imr_nop() - Runtime PM callback function.
 * @dev:     device data.
 *
 * Runtime PM callback function.
 *
 *
 * Return: 0   normal end.
 */
 /* PRQA S 3206 1 */
static int uio_runtime_imr_nop(__attribute__((unused)) struct device *dev)
{
	pr_debug("uio_runtime_imr_nop enter\n"); /* PRQA S 3200 */
	return 0;
}
/* PRQA S 1053 4 */
static const struct dev_pm_ops uio_dev_pm_imr_ops = {
	.runtime_suspend = &uio_runtime_imr_nop,
	.runtime_resume = &uio_runtime_imr_nop,
};

static struct platform_driver uio_imr_platform_driver = {
	.probe = &uio_imr_probe,
	.remove = &uio_imr_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &uio_dev_pm_imr_ops,
		.of_match_table = of_match_ptr(rcar_imr_dt_ids),
	},
};

module_platform_driver(uio_imr_platform_driver);


MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Userspace I/O driver for IMR");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
