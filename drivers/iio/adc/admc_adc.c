/*
 * Analog Devices MC-ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

/**
 * Note:
 * This driver is an old copy from the cf_axi_adc/axi-adc driver.
 * And some things were common with that driver. The cf_axi_adc/axi-adc
 * driver is a more complete implementation, while this one is just caring
 * about Motor Control.
 * The code duplication [here] is intentional, as we try to cleanup the
 * AXI ADC and decouple it from this driver.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include <linux/fpga/adi-axi-common.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)

/* AXI TRIGGER REGS */
#define ADI_REG_VERSION                 0X000
#define ADI_REG_SCRATCH                 0x004
#define ADI_REG_VALID_PROBES            0x008

#define ADI_REG_TRIGGERS_REL            0x00C
#define ADI_REG_TRIGGER_TYPE            0x010
#define ADI_REG_FIFO_DEPTH              0x014

/* channel 0 registers */
#define ADI_REG_EDGE_DETECT_EN_0        0x040
#define ADI_REG_RISE_EDGE_EN_0          0x044
#define ADI_REG_FALL_EDGE_EN_0          0x048
#define ADI_REG_LOW_LVL_EN_0            0x04C
#define ADI_REG_HIGH_LVL_EN_0           0x050
#define ADI_REG_LIMIT_0                 0x054
#define ADI_REG_HYSTERESIS_0            0x058 
#define ADI_REG_TRIGGER_ADC_0           0x05C

/* channel 1 registers */
#define ADI_REG_EDGE_DETECT_EN_1        0x060
#define ADI_REG_RISE_EDGE_EN_1          0x064
#define ADI_REG_FALL_EDGE_EN_1          0x068
#define ADI_REG_LOW_LVL_EN_1            0x06C
#define ADI_REG_HIGH_LVL_EN_1           0x070
#define ADI_REG_LIMIT_1                 0x074
#define ADI_REG_HYSTERESIS_1            0x078 
#define ADI_REG_TRIGGER_ADC_1           0x07C

/* channel 2 registers */
#define ADI_REG_EDGE_DETECT_EN_2        0x080
#define ADI_REG_RISE_EDGE_EN_2          0x084
#define ADI_REG_FALL_EDGE_EN_2          0x088
#define ADI_REG_LOW_LVL_EN_2            0x08C
#define ADI_REG_HIGH_LVL_EN_2           0x090
#define ADI_REG_LIMIT_2                 0x094
#define ADI_REG_HYSTERESIS_2            0x098
#define ADI_REG_TRIGGER_ADC_2           0x09C

/* channel 3 registers */
#define ADI_REG_EDGE_DETECT_EN_3        0x0A0
#define ADI_REG_RISE_EDGE_EN_3          0x0A4
#define ADI_REG_FALL_EDGE_EN_3          0x0A8
#define ADI_REG_LOW_LVL_EN_3            0x0AC
#define ADI_REG_HIGH_LVL_EN_3           0x0B0
#define ADI_REG_LIMIT_3                 0x0B4
#define ADI_REG_HYSTERESIS_3            0x0B8
#define ADI_REG_TRIGGER_ADC_3           0x0BC

#define ADI_ENABLE			(1 << 0)

#define ID_AD_MC_ADC   1

struct axiadc_chip_info {
	char				*name;
	unsigned			num_channels;
	const unsigned long		*scan_masks;
	unsigned int			max_rate;
	struct iio_chan_spec		channel[4];
};

struct axiadc_state {
	struct iio_info			iio_info;
	void __iomem			*regs;
	unsigned int			pcore_version;
};


static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}


static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}


static int axiadc_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	//axiadc_write(st, ADI_REG_STATUS, ~0);
	//axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

	return 0;
}

// cand vreau sa fac un transfer; implementeaza submit si abort
static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit = axiadc_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int axiadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
			&axiadc_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static void axiadc_unconfigure_ring_stream(struct iio_dev *indio_dev)
{
	iio_dmaengine_buffer_free(indio_dev->buffer);
}

static int axiadc_reg_access(struct iio_dev *indio_dev,
			     unsigned reg, unsigned writeval,
			     unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		axiadc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

/* enabling channels to be monitored by axi_trigger ip  */
static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;
pr_err("%s: scan_mask = 0x%lx\n", __FUNCTION__, *scan_mask);
	axiadc_write(st, ADI_REG_VALID_PROBES, *scan_mask);

#if 0
	for (i = 0; i < indio_dev->masklength; i++) {
		//ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		//axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}
#endif
	return 0;
}

static const struct iio_info axiadc_info = {
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)		  \
	{ .type = IIO_VOLTAGE,					  \
	  .indexed = 1,						 \
	  .channel = _chan,					 \
	  .scan_index = _si,						\
	  .scan_type = {				\
		.sign = _sign,				\
		.realbits = _bits,			\
		.storagebits = 16,			\
		.shift = 0,				\
	  },						\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD_MC_ADC] = {
		.name = "AD-MC-ADC",
		.max_rate = 1000000UL,
		.num_channels = 4,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 12, 's'),
			AIM_CHAN_NOCALIB(1, 1, 12, 's'),
			AIM_CHAN_NOCALIB(2, 2, 12, 's'),
			AIM_CHAN_NOCALIB(3, 3, 12, 's'),
			//AIM_CHAN_NOCALIB(4, 4, 8, 'u'),
			//AIM_CHAN_NOCALIB(5, 5, 8, 'u'),
			//AIM_CHAN_NOCALIB(6, 6, 8, 'u'),
			//AIM_CHAN_NOCALIB(7, 7, 8, 'u'),
			//AIM_CHAN_NOCALIB(8, 8, 8, 'u'),
			//AIM_CHAN_NOCALIB(9, 9, 8, 'u'),
			//AIM_CHAN_NOCALIB(10, 10, 8, 'u'),
			//AIM_CHAN_NOCALIB(11, 11, 8, 'u'),
			//AIM_CHAN_NOCALIB(12, 12, 8, 'u'),
			//AIM_CHAN_NOCALIB(13, 13, 8, 'u'),
			//AIM_CHAN_NOCALIB(14, 14, 8, 'u'),
			//AIM_CHAN_NOCALIB(15, 15, 8, 'u'),
		},
	},
};

static int axiadc_probe(struct platform_device *pdev)
{
	const struct axiadc_chip_info *chip_info;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	chip_info = &axiadc_chip_info_tbl[ID_AD_MC_ADC];

	platform_set_drvdata(pdev, indio_dev);

	/* Reset all HDL Cores */
	//axiadc_write(st, ADI_REG_RSTN, 0);
	//axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	//st->pcore_version = axiadc_read(st, ADI_AXI_REG_VERSION);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	st->iio_info = axiadc_info;
	indio_dev->info = &st->iio_info;
pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	// searches in the devicetree, the dma called dma_trigger
	ret = axiadc_configure_ring_stream(indio_dev, "dma-trigger");
	if (ret < 0)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)mem->start, st->regs, chip_info->name,
		 //axiadc_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");
		 "MASTER");

	return 0;

err_unconfigure_ring:
	axiadc_unconfigure_ring_stream(indio_dev);

	return ret;
}

static int axiadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	axiadc_unconfigure_ring_stream(indio_dev);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-adc-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe	  = axiadc_probe,
	.remove	 = axiadc_remove,
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-ADC");
MODULE_LICENSE("GPL v2");
