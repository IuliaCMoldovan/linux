/*
 * Analog Devices AXI TRIGGER Module
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
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

/* AXI TRIGGER REGS */
#define ADI_REG_VERSION                 0X000
#define ADI_REG_SCRATCH                 0x004
#define ADI_REG_VALID_PROBES            0x008

#define ADI_REG_TRIGGERS_REL            0x00C
#define ADI_REG_TRIGGER_TYPE            0x010

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

#define ID_AXI_TRIGGER   1

struct axi_trigger_chip_info {
	char				*name;
	unsigned			num_channels;
	const unsigned long		*scan_masks;
	unsigned int			max_rate;
	struct iio_chan_spec		channel[4];
};

struct axi_trigger_state {
	struct iio_info			iio_info;
	void __iomem			*regs;
	unsigned int			pcore_version;
	unsigned int			test[4];
};


static inline void axi_trigger_write(struct axi_trigger_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axi_trigger_read(struct axi_trigger_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int axi_trigger_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axi_trigger_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);
	return 0;
}

static const struct iio_dma_buffer_ops axi_trigger_dma_buffer_ops = {
	.submit = axi_trigger_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int axi_trigger_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
			&axi_trigger_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static void axi_trigger_unconfigure_ring_stream(struct iio_dev *indio_dev)
{
	iio_dmaengine_buffer_free(indio_dev->buffer);
}

static int axi_trigger_reg_access(struct iio_dev *indio_dev,
			     unsigned reg, unsigned writeval,
			     unsigned *readval)
{
	struct axi_trigger_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		axi_trigger_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axi_trigger_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static int axi_trigger_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct axi_trigger_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;

	axi_trigger_write(st, ADI_REG_VALID_PROBES, *scan_mask);

	return 0;
}

static int axi_trigger_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct axi_trigger_state *st = iio_priv(indio_dev);

	switch (info) {
	// LIMIT
	case IIO_CHAN_INFO_RAW:
		*val = st->test[chan->channel];
		if (chan->channel == 0) {
			*val = axi_trigger_read(st, ADI_REG_LIMIT_0 & 0xFFFF);
			pr_err("IIO_CHAN_INFO_RAW(0):\n__%s__: linia %d: citesc %d de la ADI_REG_LIMIT_0\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 1) {
			*val = axi_trigger_read(st, ADI_REG_LIMIT_1 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_RAW(1):\n__%s__: linia %d: citesc %d de la ADI_REG_LIMIT_1\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 2) {
			*val = axi_trigger_read(st, ADI_REG_LIMIT_2 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_RAW(2):\n__%s__: linia %d: citesc %d de la ADI_REG_LIMIT_2\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 3) {
			*val = axi_trigger_read(st, ADI_REG_LIMIT_3 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_RAW(3):\n__%s__: linia %d: citesc %d de la ADI_REG_LIMIT_3\n", __FUNCTION__, __LINE__, *val);
		}
		return IIO_VAL_INT;
	// TRIGGER TYPE: 0 - analog trigger, 1 - digital trigger
	case IIO_CHAN_INFO_ENABLE: 
		//*val = st->test[chan->channel];
		*val = axi_trigger_read(st, ADI_REG_TRIGGER_TYPE & 0xFFFF);
		pr_err("IIO_CHAN_INFO_ENABLE:\n__%s__: linia %d, citesc %d de la ADI_REG_TRIGGER_TYPE\n", __FUNCTION__, __LINE__, *val);
		return IIO_VAL_INT;
	// VALID PROBES
	case IIO_CHAN_INFO_OFFSET: 
		*val = axi_trigger_read(st, ADI_REG_VALID_PROBES & 0xFFFF);
		pr_err("IIO_CHAN_INFO_OFFSET:\n__%s__: linia %d, citesc %d de la ADI_REG_VALID_PROBES\n", __FUNCTION__, __LINE__, *val);
		return IIO_VAL_INT;
	// TRIGGER ADC 
	case IIO_CHAN_INFO_SCALE:
		*val = st->test[chan->channel];
		if (chan->channel == 0) {
			*val = axi_trigger_read(st, ADI_REG_TRIGGER_ADC_0 & 0xFFFF);
			pr_err("IIO_CHAN_INFO_SCALE(0):\n__%s__: linia %d: citesc %d de la ADI_REG_TRIGGER_ADC_0\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 1) {
			*val = axi_trigger_read(st, ADI_REG_TRIGGER_ADC_1 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_SCALE(1):\n__%s__: linia %d: citesc %d de la ADI_REG_TRIGGER_ADC_1\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 2) {
			*val = axi_trigger_read(st, ADI_REG_TRIGGER_ADC_2 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_SCALE(2):\n__%s__: linia %d: citesc %d de la ADI_REG_TRIGGER_ADC_2\n", __FUNCTION__, __LINE__, *val);
		} else if (chan->channel == 3) {
			*val = axi_trigger_read(st, ADI_REG_TRIGGER_ADC_3 & 0xFFFF);	
			pr_err("IIO_CHAN_INFO_SCALE(3):\n__%s__: linia %d: citesc %d de la ADI_REG_TRIGGER_ADC_3\n", __FUNCTION__, __LINE__, *val);
		}
		return IIO_VAL_INT;
	
	default:
		return -EINVAL;
	}
}

static int axi_trigger_write_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int val, int val2, long info)
{
	struct axi_trigger_state *st = iio_priv(indio_dev);

        switch (info) {
	// LIMIT
        case IIO_CHAN_INFO_RAW: 
                st->test[chan->channel] =  val;

		if (chan->channel == 0) {
			axi_trigger_write(st, ADI_REG_LIMIT_0, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_LIMIT_0\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 1) {
			axi_trigger_write(st, ADI_REG_LIMIT_0, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_LIMIT_1\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 2) {
			axi_trigger_write(st, ADI_REG_LIMIT_2, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_LIMIT_2\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 3) {
			axi_trigger_write(st, ADI_REG_LIMIT_3, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_LIMIT_3\n", __FUNCTION__, __LINE__, val);
		}
                return 0;
	// TRIGGER TYPE
	case IIO_CHAN_INFO_ENABLE:
		axi_trigger_write(st, ADI_REG_TRIGGER_TYPE, val);
		pr_err("%s: %d -> scriu %d in ADI_REG_TRIGGER_TYPE\n", __FUNCTION__, __LINE__, val);
		return 0;
	// VALID PROBES
	case IIO_CHAN_INFO_OFFSET: 
		axi_trigger_write(st, ADI_REG_VALID_PROBES, val);
		pr_err("%s: %d -> scriu %x in ADI_REG_VALID_PROBES\n", __FUNCTION__, __LINE__, val);
		return 0;
	// TRIGGER ADC 
	case IIO_CHAN_INFO_SCALE:
                st->test[chan->channel] =  val;

		if (chan->channel == 0) {
			axi_trigger_write(st, ADI_REG_TRIGGER_ADC_0, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_TRIGGER_ADC_0\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 1) {
			axi_trigger_write(st, ADI_REG_TRIGGER_ADC_1, val);	
			pr_err("%s: %d -> scriu %d in ADI_REG_TRIGGER_ADC_1\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 2) {
			axi_trigger_write(st, ADI_REG_TRIGGER_ADC_2, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_TRIGGER_ADC_2\n", __FUNCTION__, __LINE__, val);
		} else if (chan->channel == 3) {
			axi_trigger_write(st, ADI_REG_TRIGGER_ADC_3, val);
			pr_err("%s: %d -> scriu %d in ADI_REG_TRIGGER_ADC_3\n", __FUNCTION__, __LINE__, val);
		}
		return 0;
	
	default:
		return -EINVAL;
	}
}

static const struct iio_info axi_trigger_info = {
	.debugfs_reg_access = &axi_trigger_reg_access,
	.update_scan_mode = &axi_trigger_update_scan_mode,
	.read_raw = axi_trigger_read_raw,
	.write_raw = axi_trigger_write_raw,
};
#if 0
static int axi_trig_set_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trigger_state *st = iio_priv(indio_dev);
	
	// lock
	mutex_lock(&indio_dev->mlock);
	
	if (chan->address == 0) 
	{
		// determin ce atribut e selectat si in functie de ala, scriu in registrul care trebuie
		if ()
		{
			axi_trigger_write(st, ADI_REG_EDGE_DETECT_EN_0, 
		}
	} else if (chan->address == 1) {

	} else if (chan->address == 2) {

	} else if (chan->address == 3) {

	}
	




	// unlock
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static int axi_trig_get_trigger(struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan) 
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return axi_trig->trigger_pin_config[chan->address];
}

static const char * const axi_trig_trigger_items[] = {
	"edge-any",
	"edge-rising",
	"edge-falling",
	"level-low",
	"level-high",
	"limit",
	"histerezis",
	"trigger-adc",
};

static const struct iio_enum axi_trig_trigger_enum = {
	.items = axi_trig_trigger_items,
	.num_items = ARRAY_SIZE(axi_trig_trigger_items),
	.set = axi_trig_set_trigger,
	.get = axi_trig_get_trigger,
};

static const struct iio_chan_spec_ext_info axi_trig_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &axi_trig_trigger_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger", &axi_trig_trigger_enum),
	{}
};
#endif

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)	\
	{ .type = IIO_VOLTAGE,				\
	  .indexed = 1,					\
	  .channel = _chan,				\
	  .scan_index = _si,				\
	  .scan_type = {				\
		.sign = _sign,				\
		.realbits = _bits,			\
		.storagebits = 16,			\
		.shift = 0,				\
	  },						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_ENABLE) | BIT(IIO_CHAN_INFO_OFFSET) | BIT(IIO_CHAN_INFO_SCALE) \
	}

static const struct axi_trigger_chip_info axi_trigger_chip_info_tbl[] = {
	[ID_AXI_TRIGGER] = {
		.name = "AXI-TRIGGER",
		.max_rate = 1000000UL,
		.num_channels = 4,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 12, 's'),
			AIM_CHAN_NOCALIB(1, 1, 12, 's'),
			AIM_CHAN_NOCALIB(2, 2, 12, 's'),
			AIM_CHAN_NOCALIB(3, 3, 12, 's'),
		},
	},
};

static int axi_trigger_probe(struct platform_device *pdev)
{
	const struct axi_trigger_chip_info *chip_info;
	struct iio_dev *indio_dev;
	struct axi_trigger_state *st;
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

	chip_info = &axi_trigger_chip_info_tbl[ID_AXI_TRIGGER];

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	st->iio_info = axi_trigger_info;
	indio_dev->info = &st->iio_info;

	ret = axi_trigger_configure_ring_stream(indio_dev, "dma-trigger");
	if (ret < 0)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)mem->start, st->regs, chip_info->name,
		 //axi_trigger_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");
		 "MASTER");

	return 0;

err_unconfigure_ring:
	axi_trigger_unconfigure_ring_stream(indio_dev);

	return ret;
}

static int axi_trigger_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	axi_trigger_unconfigure_ring_stream(indio_dev);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axi_trigger_of_match[] = {
	{ .compatible = "xlnx,axi-trigger", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axi_trigger_of_match);

static struct platform_driver axi_trigger_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axi_trigger_of_match,
	},
	.probe	  = axi_trigger_probe,
	.remove	 = axi_trigger_remove,
};

module_platform_driver(axi_trigger_driver);

MODULE_AUTHOR("Iulia Moldovan <iulia.moldovan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI TRIGGER");
MODULE_LICENSE("GPL v2");
