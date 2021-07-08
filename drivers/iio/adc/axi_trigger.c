/*
 * AXI Trigger Driver
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
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define AXI_ADC_TRIG_REG_TRIGGER_O		0x08
#define AXI_ADC_TRIG_REG_IO_SELECTION		0x0C
#define AXI_ADC_TRIG_REG_CONFIG_TRIGGER		0x10

#define AXI_ADC_TRIG_REG_LIMIT(x)		(0x14 + ((x) * 0x10))
#define AXI_ADC_TRIG_REG_FUNCTION(x)		(0x18 + ((x) * 0x10))
#define AXI_ADC_TRIG_REG_HYSTERESIS(x)		(0x1C + ((x) * 0x10))
#define AXI_ADC_TRIG_REG_TRIGGER_MIX(x)		(0x20 + ((x) * 0x10))
#define AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL	0x34
#define AXI_ADC_TRIG_REG_FIFO_DEPTH		0x38
#define AXI_ADC_TRIG_REG_TRIGGERED		0x3c
#define AXI_ADC_TRIG_REG_DELAY			0x40


/* AXI TRIGGER REGS */
#define AXI_TRIG_REG_VERSION                 0X000
#define AXI_TRIG_REG_SCRATCH                 0x004
#define AXI_TRIG_REG_VALID_PROBES            0x008

#define AXI_TRIG_REG_TRIGGERS_REL            0x00C
#define AXI_TRIG_REG_TRIGGER_TYPE            0x010
#define AXI_TRIG_REG_FIFO_DEPTH              0x014

/* channel 0 registers */
#define AXI_TRIG_REG_EDGE_DETECT_EN_0        0x040
#define AXI_TRIG_REG_RISE_EDGE_EN_0          0x044
#define AXI_TRIG_REG_FALL_EDGE_EN_0          0x048
#define AXI_TRIG_REG_LOW_LVL_EN_0            0x04C
#define AXI_TRIG_REG_HIGH_LVL_EN_0           0x050
#define AXI_TRIG_REG_LIMIT_0                 0x054
#define AXI_TRIG_REG_HYSTERESIS_0            0x058 
#define AXI_TRIG_REG_TRIGGER_ADC_0           0x05C

/* channel 1 registers */
#define AXI_TRIG_REG_EDGE_DETECT_EN_1        0x060
#define AXI_TRIG_REG_RISE_EDGE_EN_1          0x064
#define AXI_TRIG_REG_FALL_EDGE_EN_1          0x068
#define AXI_TRIG_REG_LOW_LVL_EN_1            0x06C
#define AXI_TRIG_REG_HIGH_LVL_EN_1           0x070
#define AXI_TRIG_REG_LIMIT_1                 0x074
#define AXI_TRIG_REG_HYSTERESIS_1            0x078 
#define AXI_TRIG_REG_TRIGGER_ADC_1           0x07C

/* channel 2 registers */
#define AXI_TRIG_REG_EDGE_DETECT_EN_2        0x080
#define AXI_TRIG_REG_RISE_EDGE_EN_2          0x084
#define AXI_TRIG_REG_FALL_EDGE_EN_2          0x088
#define AXI_TRIG_REG_LOW_LVL_EN_2            0x08C
#define AXI_TRIG_REG_HIGH_LVL_EN_2           0x090
#define AXI_TRIG_REG_LIMIT_2                 0x094
#define AXI_TRIG_REG_HYSTERESIS_2            0x098
#define AXI_TRIG_REG_TRIGGER_ADC_2           0x09C

/* channel 3 registers */
#define AXI_TRIG_REG_EDGE_DETECT_EN_3        0x0A0
#define AXI_TRIG_REG_RISE_EDGE_EN_3          0x0A4
#define AXI_TRIG_REG_FALL_EDGE_EN_3          0x0A8
#define AXI_TRIG_REG_LOW_LVL_EN_3            0x0AC
#define AXI_TRIG_REG_HIGH_LVL_EN_3           0x0B0
#define AXI_TRIG_REG_LIMIT_3                 0x0B4
#define AXI_TRIG_REG_HYSTERESIS_3            0x0B8
#define AXI_TRIG_REG_TRIGGER_ADC_3           0x0BC


/* AXI_ADC_TRIG_REG_CONFIG_TRIGGER */
#define CONF_LOW_LEVEL				0
#define CONF_HIGH_LEVEL				1
#define CONF_ANY_EDGE				2
#define CONF_RISING_EDGE			3
#define CONF_FALLING_EDGE			4

#define CONFIG_PIN_TRIGGER(t0_conf, t1_conf)	\
	((1 << ((t0_conf) * 2)) | (2 << ((t1_conf) * 2)))
#define CONFIG_IO_SELECTION(pin_nr, val)	((val & 0x7) << (3*pin_nr + 2))

#define TRIGGER_PIN_CHAN			2
#define TRIGGER_ADC_CHAN			2
#define TRIGGER_MIX_CHAN			2

#define TRIGGER_HOLDOFF_MASK			GENMASK(31, 0)

#define IIO_ENUM_AVAILABLE_SEPARATE(_name, _e) \
{ \
	.name = (_name "_available"), \
	.shared = IIO_SEPARATE, \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

enum trig_ext_info {
	TRIG_LEVEL,
	TRIG_HYST,
	TRIG_ITEMS
};

struct axi_trig {
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
	unsigned int trigger_pin_config[TRIGGER_PIN_CHAN];
	int trigger_ext_info[TRIG_ITEMS][TRIGGER_ADC_CHAN];
};

static void axi_trig_write(struct axi_trig *axi_trig,
			       unsigned int reg,
	unsigned int val)
{
	writel_relaxed(val, axi_trig->regs + reg);
}

static unsigned int axi_trig_read(struct axi_trig *axi_trig,
				      unsigned int reg)
{
	return readl_relaxed(axi_trig->regs + reg);
}

static int axi_trig_reg_access(struct iio_dev *indio_dev,
				   unsigned int reg, unsigned int writeval,
				   unsigned int *readval)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	
	// sincronizez accesul la registru
	mutex_lock(&axi_trig->lock);
	if (readval == NULL)
		axi_trig_write(axi_trig, reg & 0xFF, writeval);
	else
		*readval = axi_trig_read(axi_trig, reg & 0xFF);

	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(axi_trig->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/* Trigger_O selection */

static int axi_trig_set_out_pin_select(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&axi_trig->lock);
	mask = axi_trig_read(axi_trig, AXI_ADC_TRIG_REG_IO_SELECTION);
	mask &= ~GENMASK((3 * chan->address + 2) + 2, (3 * chan->address + 2));
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_IO_SELECTION,
			   mask | CONFIG_IO_SELECTION(chan->address, val));
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_get_out_pin_select(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val, mask;

	mask = GENMASK((3 * chan->address + 2) + 2, (3 * chan->address + 2));
	val = axi_trig_read(axi_trig,
				AXI_ADC_TRIG_REG_IO_SELECTION) & mask;

	return val >> (3 * chan->address + 2);
}

static const char * const axi_trig_out_pin_select_items[] = {
	"sw-trigger",
	"trigger-i-same-chan",
	"trigger-i-swap-chan",
	"trigger-adc",
	"trigger-in",
};

static const struct iio_enum axi_trig_out_pin_select_enum = {
	.items = axi_trig_out_pin_select_items,
	.num_items = ARRAY_SIZE(axi_trig_out_pin_select_items),
	.set = axi_trig_set_out_pin_select,
	.get = axi_trig_get_out_pin_select,
};

static int axi_trig_set_io_direction(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&axi_trig->lock);
	mask = axi_trig_read(axi_trig, AXI_ADC_TRIG_REG_IO_SELECTION);
	mask &= ~BIT(chan->address);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_IO_SELECTION,
			   mask | (val << chan->address));
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_get_io_direction(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val, mask;

	mask = BIT(chan->address);
	val = axi_trig_read(axi_trig,
				AXI_ADC_TRIG_REG_IO_SELECTION) & mask;

	return val >> chan->address;
}

static const char * const axi_trig_io_direction_items[] = {
	"in",
};

static const struct iio_enum axi_trig_io_direction_enum = {
	.items = axi_trig_io_direction_items,
	.num_items = ARRAY_SIZE(axi_trig_io_direction_items),
	.set = axi_trig_set_io_direction,
	.get = axi_trig_get_io_direction,
};

/* PIN/Digital trigger */

static int axi_trig_pin_set_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	mutex_lock(&axi_trig->lock);
	axi_trig->trigger_pin_config[chan->address] = val;

	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_CONFIG_TRIGGER,
			CONFIG_PIN_TRIGGER(axi_trig->trigger_pin_config[0],
				axi_trig->trigger_pin_config[1]));
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_pin_get_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return axi_trig->trigger_pin_config[chan->address];
}

static const char * const axi_trig_pin_trigger_items[] = {
	"edge-any",
	"edge-rising",
	"edge-falling",
	"level-low",
	"level-high",
};

static const struct iio_enum axi_trig_trigger_pin_enum = {
	.items = axi_trig_pin_trigger_items,
	.num_items = ARRAY_SIZE(axi_trig_pin_trigger_items),
	.set = axi_trig_pin_set_trigger,
	.get = axi_trig_pin_get_trigger,
};

/* Analog ADC trigger */
// setarea conditiilor de trigger
static int axi_trig_analog_set_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	mutex_lock(&axi_trig->lock);
	if (chan
	axi_trig_write(axi_trig,
			   AXI_ADC_TRIG_REG_FUNCTION(chan->address), val);
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_analog_get_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return axi_trig_read(axi_trig,
			   AXI_ADC_TRIG_REG_FUNCTION(chan->address));
}

static const char * const axi_trig_analog_trigger_items[] = {
	"edge-any",
	"edge-rising",
	"edge-falling",
	"level-low",
	"level-high",
};

static const struct iio_enum axi_trig_trigger_analog_enum = {
	.items = axi_trig_analog_trigger_items,
	.num_items = ARRAY_SIZE(axi_trig_analog_trigger_items),
	.set = axi_trig_analog_set_trigger,
	.get = axi_trig_analog_get_trigger,
};

/* Trigger MIX */

static int axi_trig_set_trigger_mix_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	mutex_lock(&axi_trig->lock);
	axi_trig_write(axi_trig,
			   AXI_ADC_TRIG_REG_TRIGGER_MIX(chan->address), val);
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_get_trigger_mix_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return axi_trig_read(axi_trig,
				 AXI_ADC_TRIG_REG_TRIGGER_MIX(chan->address));
}

static const char * const axi_trig_trigger_mix_mode_items[] = {
	"analog",
	"digital",
};

static const struct iio_enum axi_trig_trigger_mix_mode_enum = {
	.items = axi_trig_trigger_mix_mode_items,
	.num_items = ARRAY_SIZE(axi_trig_trigger_mix_mode_items),
	.set = axi_trig_set_trigger_mix_mode,
	.get = axi_trig_get_trigger_mix_mode,
};

/* Trigger OUT MIX */

static int axi_trig_set_trigger_out_mix_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&axi_trig->lock);
	mask = axi_trig_read(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL);
	mask &= ~GENMASK(3, 0);
	axi_trig_write(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL, mask | val);
	mutex_unlock(&axi_trig->lock);

	return 0;
}

static int axi_trig_get_trigger_out_mix_mode(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return axi_trig_read(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL) & GENMASK(3, 0);

}

static const char * const axi_trig_trigger_out_mix_mode_items[] = {
	"int",
	"ext",
	"int_AND_ext",
	"int_OR_ext",
	"int_XOR_ext",
	"disabled"
};

static const struct iio_enum axi_trig_trigger_out_mix_mode_enum = {
	.items = axi_trig_trigger_out_mix_mode_items,
	.num_items = ARRAY_SIZE(axi_trig_trigger_out_mix_mode_items),
	.set = axi_trig_set_trigger_out_mix_mode,
	.get = axi_trig_get_trigger_out_mix_mode,
};

static ssize_t axi_trig_set_extinfo(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	int ret, val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&axi_trig->lock);

	switch (priv) {
	case TRIG_LEVEL:
		/* FIXME add check for lvl + hyst */
		axi_trig->trigger_ext_info[priv][chan->address] = val;
		axi_trig_write(axi_trig,
				   AXI_ADC_TRIG_REG_LIMIT(chan->address), val);
		break;
	case TRIG_HYST:
		/* FIXME add check for lvl + hyst */
		axi_trig->trigger_ext_info[priv][chan->address] = val;
		axi_trig_write(axi_trig,
				   AXI_ADC_TRIG_REG_HYSTERESIS(chan->address),
				   val);
		break;
	default:
		ret = -EINVAL;
	}

out_unlock:
	mutex_unlock(&axi_trig->lock);

	return len;
}

static ssize_t axi_trig_get_extinfo(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 axi_trig->trigger_ext_info[priv][chan->address]);
}

static ssize_t axi_trig_get_triggered(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val;

	val = axi_trig_read(axi_trig, AXI_ADC_TRIG_REG_TRIGGERED) & 1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t axi_trig_set_triggered(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_TRIGGERED, val);

	return len;
}

static ssize_t axi_trig_get_embedded_trigger(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val;

	val = axi_trig_read(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL) >> 16;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val & 1);
}

static ssize_t axi_trig_set_embedded_trigger(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct axi_trig *axi_trig = iio_priv(indio_dev);
	unsigned int val, mask;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	mask = axi_trig_read(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL);

	if (val)
		mask |= BIT(16);
	else
		mask &= ~BIT(16);

	axi_trig_write(axi_trig,
		AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL, mask);

	return len;
}

static const struct iio_chan_spec_ext_info axi_trig_analog_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &axi_trig_trigger_analog_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger",
				    &axi_trig_trigger_analog_enum),
	{
		.name = "trigger_level",
		.shared = IIO_SEPARATE,
		.write = axi_trig_set_extinfo,
		.read = axi_trig_get_extinfo,
		.private = TRIG_LEVEL,
	},
	{
		.name = "trigger_hysteresis",
		.shared = IIO_SEPARATE,
		.write = axi_trig_set_extinfo,
		.read = axi_trig_get_extinfo,
		.private = TRIG_HYST,
	},
	{}
};

static const struct iio_chan_spec_ext_info axi_trig_digital_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &axi_trig_trigger_pin_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger", &axi_trig_trigger_pin_enum),
	{}
};

static const struct iio_chan_spec_ext_info axi_trig_adc_pin_mix[] = {
	IIO_ENUM("trigger_logic_mode", IIO_SEPARATE,
		&axi_trig_trigger_mix_mode_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_mode",
		&axi_trig_trigger_mix_mode_enum),
	IIO_ENUM("trigger_logic_out_select", IIO_SEPARATE,
		&axi_trig_out_pin_select_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_out_select",
		&axi_trig_out_pin_select_enum),
	IIO_ENUM("trigger_logic_out_direction", IIO_SEPARATE,
		&axi_trig_io_direction_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_out_direction",
		&axi_trig_io_direction_enum),
	{}
};

static const struct iio_chan_spec_ext_info axi_trig_a_b_mix[] = {
	IIO_ENUM("trigger_logic_mode", IIO_SEPARATE,
		&axi_trig_trigger_out_mix_mode_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_mode",
		&axi_trig_trigger_out_mix_mode_enum),
	{
		.name = "trigger_embedded",
		.shared = IIO_SEPARATE,
		.write = axi_trig_set_embedded_trigger,
		.read = axi_trig_get_embedded_trigger,
	},
	{}
};

#define AXI_TRIG_CHAN_ADC(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = axi_trig_analog_info, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_PIN(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = axi_trig_digital_info, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_ADC_X_PIN(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = axi_trig_adc_pin_mix, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_OUT_A_X_B(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = axi_trig_a_b_mix, \
	.extend_name = n,\
}

static const struct iio_chan_spec axi_trig_chan_spec[] = {
	AXI_TRIG_CHAN_ADC(0, 0, NULL),
	AXI_TRIG_CHAN_ADC(1, 1, NULL),
	AXI_TRIG_CHAN_PIN(2, 0, NULL),
	AXI_TRIG_CHAN_PIN(3, 1, NULL),
	AXI_TRIG_CHAN_ADC_X_PIN(4, 0, NULL),
	AXI_TRIG_CHAN_ADC_X_PIN(5, 1, NULL),
	AXI_TRIG_CHAN_OUT_A_X_B(6, 0, NULL),
};

static const struct iio_info axi_trig_iio_info = {
	.read_raw = axi_trig_read_raw,
	.debugfs_reg_access = axi_trig_reg_access,
};

static void axi_trigger_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static int axi_trig_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axi_trig *axi_trig;
	struct resource *mem;
	int ret;

	// aloc spatiu pentru un iio device
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*axi_trig));
	if (!indio_dev)
		return -ENOMEM;

	// il preiau in structura mea de axi_trig
	axi_trig = iio_priv(indio_dev);
	// sincronizez accesul
	mutex_init(&axi_trig->lock);

	// asignez la atributele structurii
	axi_trig->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(axi_trig->clk))
		return PTR_ERR(axi_trig->clk);

	ret = clk_prepare_enable(axi_trig->clk);
	if (ret < 0)
		return -EINVAL;

	ret = devm_add_action_or_reset(&pdev->dev, axi_trigger_clk_disable,
				       axi_trig->clk);
	if (ret)
		return ret;

	// asignez la atributele structurii
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	axi_trig->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(axi_trig->regs))
		return PTR_ERR(axi_trig->regs);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	// atributele specifice canalelor 
	indio_dev->info = &axi_trig_iio_info;
	// canalele in sine 
	indio_dev->channels = axi_trig_chan_spec,
	// numarul de canale 
	indio_dev->num_channels = ARRAY_SIZE(axi_trig_chan_spec);

	// resetez toti registrii
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_FUNCTION(0), 2);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_FUNCTION(1), 2);

	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_TRIGGER_MIX(0), 0);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_TRIGGER_MIX(1), 0);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_TRIGGER_OUT_CTRL, 0);

	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_HYSTERESIS(0), 1);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_HYSTERESIS(1), 1);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_LIMIT(0), 0);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_LIMIT(1), 0);
	axi_trig_write(axi_trig, AXI_ADC_TRIG_REG_DELAY, 0);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

// creez compatible-ul pe care o sa il folosesc in devicetree
static const struct of_device_id axi_trig_of_match[] = {
	{ .compatible = "adi,axi-trigger" },
	{},
};

// setez ce functie si ce driver trebuie utilizate cand creez un driver dinasta
static struct platform_driver axi_trig_driver = {
	.driver = {
		.name = "axi-trigger",
		.of_match_table = axi_trig_of_match,
	},
	.probe = axi_trig_probe,
};
module_platform_driver(axi_trig_driver);

MODULE_AUTHOR("Iulia Moldovan <iulia.moldovan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI trigger driver");
MODULE_LICENSE("GPL v2");