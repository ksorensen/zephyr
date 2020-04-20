/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright (c) 2019, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_lpi2c

#include <errno.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/clock_control.h>
#include <fsl_lpi2c.h>

#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
#include <drivers/pinmux.h>
#include <fsl_port.h>
#include "i2c_bitbang.h"
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(mcux_lpi2c);

#include "i2c-priv.h"

struct mcux_lpi2c_recovery_config {
	char *scl_gpio_name;
	gpio_pin_t scl_gpio_pin;
	gpio_flags_t scl_gpio_flags;
	char *sda_gpio_name;
	gpio_pin_t sda_gpio_pin;
	gpio_flags_t sda_gpio_flags;
	char *scl_mux_name;
	char *sda_mux_name;
};

struct mcux_lpi2c_config {
	LPI2C_Type *base;
	char *clock_name;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(struct device *dev);
	const struct mcux_lpi2c_recovery_config *recovery;
	u32_t bitrate;
	u32_t bus_idle_timeout_ns;
};

struct mcux_lpi2c_data {
	lpi2c_master_handle_t handle;
	struct k_sem device_sync_sem;
	status_t callback_status;
#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
	struct device *scl_gpio_dev;
	struct device *sda_gpio_dev;
	struct device *scl_mux_dev;
	struct device *sda_mux_dev;
#endif /* CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS */
};

static int mcux_lpi2c_configure(struct device *dev, u32_t dev_config_raw)
{
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	LPI2C_Type *base = config->base;
	struct device *clock_dev;
	u32_t clock_freq;
	u32_t baudrate;

	if (!(I2C_MODE_MASTER & dev_config_raw)) {
		return -EINVAL;
	}

	if (I2C_ADDR_10_BITS & dev_config_raw) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		baudrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		baudrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		baudrate = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	LPI2C_MasterSetBaudRate(base, clock_freq, baudrate);

	return 0;
}

static void mcux_lpi2c_master_transfer_callback(LPI2C_Type *base,
		lpi2c_master_handle_t *handle, status_t status, void *userData)
{
	struct device *dev = userData;
	struct mcux_lpi2c_data *data = dev->driver_data;

	ARG_UNUSED(handle);
	ARG_UNUSED(base);

	data->callback_status = status;
	k_sem_give(&data->device_sync_sem);
}

static u32_t mcux_lpi2c_convert_flags(int msg_flags)
{
	u32_t flags = 0U;

	if (!(msg_flags & I2C_MSG_STOP)) {
		flags |= kLPI2C_TransferNoStopFlag;
	}

	if (msg_flags & I2C_MSG_RESTART) {
		flags |= kLPI2C_TransferRepeatedStartFlag;
	}

	return flags;
}

static int mcux_lpi2c_transfer(struct device *dev, struct i2c_msg *msgs,
		u8_t num_msgs, u16_t addr)
{
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;
	LPI2C_Type *base = config->base;
	lpi2c_master_transfer_t transfer;
	status_t status;

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		if (I2C_MSG_ADDR_10_BITS & msgs->flags) {
			return -ENOTSUP;
		}

		/* Initialize the transfer descriptor */
		transfer.flags = mcux_lpi2c_convert_flags(msgs->flags);

		/* Prevent the controller to send a start condition between
		 * messages, except if explicitly requested.
		 */
		if (i != 0 && !(msgs->flags & I2C_MSG_RESTART)) {
			transfer.flags |= kLPI2C_TransferNoStartFlag;
		}

		transfer.slaveAddress = addr;
		transfer.direction = (msgs->flags & I2C_MSG_READ)
			? kLPI2C_Read : kLPI2C_Write;
		transfer.subaddress = 0;
		transfer.subaddressSize = 0;
		transfer.data = msgs->buf;
		transfer.dataSize = msgs->len;

		/* Start the transfer */
		status = LPI2C_MasterTransferNonBlocking(base,
				&data->handle, &transfer);

		/* Return an error if the transfer didn't start successfully
		 * e.g., if the bus was busy
		 */
		if (status != kStatus_Success) {
			return -EIO;
		}

		/* Wait for the transfer to complete */
		k_sem_take(&data->device_sync_sem, K_FOREVER);

		/* Return an error if the transfer didn't complete
		 * successfully. e.g., nak, timeout, lost arbitration
		 */
		if (data->callback_status != kStatus_Success) {
			return -EIO;
		}

		/* Move to the next message */
		msgs++;
	}

	return 0;
}

#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
static void mcux_lpi2c_set_scl(void *io_context, int state)
{
	struct device *dev = io_context;
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;
	int err;

	err = gpio_pin_set_raw(data->scl_gpio_dev,
			       config->recovery->scl_gpio_pin, state);
	if (err) {
		LOG_ERR("failed to set SCL to %d (err %d)", state, err);
	}
}

static void mcux_lpi2c_set_sda(void *io_context, int state)
{
	/**
	 * Not all Kinetis devices support open-collector/open-drain,
	 * so we leave out setting SDA.
	 */
}

static int mcux_lpi2c_get_sda(void *io_context)
{
	struct device *dev = io_context;
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;

	return gpio_pin_get_raw(data->sda_gpio_dev,
				config->recovery->sda_gpio_pin);
}

static int mcux_lpi2c_recover_bus(struct device *dev)
{
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;
	const struct i2c_bitbang_io io = {
		.set_scl = mcux_lpi2c_set_scl,
		.set_sda = mcux_lpi2c_set_sda,
		.get_sda = mcux_lpi2c_get_sda,
	};
	struct i2c_bitbang bitbang;
	u32_t scl_func;
	u32_t sda_func;
	int err;

	if (!data->scl_mux_dev || !data->sda_mux_dev) {
		return -ENOTSUP;
	}

	/* Save current pinmux function */
	pinmux_pin_get(data->scl_mux_dev, config->recovery->scl_gpio_pin,
		       &scl_func);
	pinmux_pin_get(data->sda_mux_dev, config->recovery->sda_gpio_pin,
		       &sda_func);

	/* Configure SCL and SDA as GPIO */
	pinmux_pin_set(data->scl_mux_dev, config->recovery->scl_gpio_pin,
		       PORT_PCR_MUX(kPORT_MuxAsGpio));
	pinmux_pin_set(data->sda_mux_dev, config->recovery->sda_gpio_pin,
		       PORT_PCR_MUX(kPORT_MuxAsGpio));

	/* Attempt to recover bus using GPIO bitbang */
	i2c_bitbang_init(&bitbang, &io, dev);
	err = i2c_bitbang_recover_bus(&bitbang);

	/* Restore original pinmux function */
	pinmux_pin_set(data->scl_mux_dev, config->recovery->scl_gpio_pin,
		       scl_func);
	pinmux_pin_set(data->sda_mux_dev, config->recovery->sda_gpio_pin,
		       sda_func);

	/* TODO: clear all errors here? Check bus clear? */

	return err;
}
#endif /* CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS */

static void mcux_lpi2c_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;
	LPI2C_Type *base = config->base;

	LPI2C_MasterTransferHandleIRQ(base, &data->handle);
}

static int mcux_lpi2c_init(struct device *dev)
{
	const struct mcux_lpi2c_config *config = dev->config->config_info;
	struct mcux_lpi2c_data *data = dev->driver_data;
	LPI2C_Type *base = config->base;
	struct device *clock_dev;
	u32_t clock_freq, bitrate_cfg;
	lpi2c_master_config_t master_config;
	int error;

	k_sem_init(&data->device_sync_sem, 0, UINT_MAX);

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
	const struct mcux_lpi2c_recovery_config *recovery = config->recovery;

	if (recovery->scl_gpio_name && recovery->sda_gpio_name) {
		data->scl_gpio_dev =
			device_get_binding(recovery->scl_gpio_name);
		if (!data->scl_gpio_dev) {
			LOG_ERR("failed to get SCL GPIO device");
			return -EINVAL;
		}

		data->scl_mux_dev = device_get_binding(recovery->scl_mux_name);
		if (!data->scl_mux_dev) {
			LOG_ERR("failed to get SCL pinmux device");
			return -EINVAL;
		}

		data->sda_gpio_dev =
			device_get_binding(recovery->sda_gpio_name);
		if (!data->sda_gpio_dev) {
			LOG_ERR("failed to get SDA GPIO device");
			return -EINVAL;
		}

		data->sda_mux_dev = device_get_binding(recovery->sda_mux_name);
		if (!data->sda_mux_dev) {
			LOG_ERR("failed to get SDA pinmux device");
			return -EINVAL;
		}

		error = gpio_config(data->scl_gpio_dev, recovery->scl_gpio_pin,
				    recovery->scl_gpio_flags |
				    GPIO_OUTPUT_HIGH);
		if (error) {
			LOG_ERR("failed to configure SCL GPIO (err %d)", error);
			return error;
		}

		error = gpio_config(data->sda_gpio_dev, recovery->sda_gpio_pin,
				    recovery->sda_gpio_flags | GPIO_INPUT);
		if (error) {
			LOG_ERR("failed to configure SDA GPIO (err %d)", error);
			return error;
		}
	}
#endif /* CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS */

	LPI2C_MasterGetDefaultConfig(&master_config);
	master_config.busIdleTimeout_ns = config->bus_idle_timeout_ns;
	LPI2C_MasterInit(base, &master_config, clock_freq);
	LPI2C_MasterTransferCreateHandle(base, &data->handle,
			mcux_lpi2c_master_transfer_callback, dev);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);

	error = mcux_lpi2c_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (error) {
		return error;
	}

	config->irq_config_func(dev);

	return 0;
}

static const struct i2c_driver_api mcux_lpi2c_driver_api = {
	.configure = mcux_lpi2c_configure,
	.transfer = mcux_lpi2c_transfer,
#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
	.recover_bus = mcux_lpi2c_recover_bus,
#endif /* CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS */
};

#ifdef CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS
#define I2C_MCUX_LPI2C_RECOVERY_INIT(n)					\
	static const struct mcux_lpi2c_recovery_config			\
	mcux_lpi2c_recovery_config_##n = {				\
		.scl_gpio_name = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, scl_gpios),		\
			DT_INST_GPIO_LABEL(n, scl_gpios)),		\
		.scl_gpio_pin = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, scl_gpios),		\
			DT_INST_GPIO_PIN(n, scl_gpios)),		\
		.scl_gpio_flags = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, scl_gpios),		\
			DT_INST_GPIO_FLAGS(n, scl_gpios)),		\
		.sda_gpio_name = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, sda_gpios),		\
			DT_INST_GPIO_LABEL(n, sda_gpios)),		\
		.sda_gpio_pin = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, sda_gpios),		\
			DT_INST_GPIO_PIN(n, sda_gpios)),		\
		.sda_gpio_flags = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, sda_gpios),		\
			DT_INST_GPIO_FLAGS(n, sda_gpios)),		\
		.scl_mux_name = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, scl_gpios),		\
			DT_PROP_BY_PHANDLE(DT_INST_PHANDLE(n, scl_gpios), \
					   nxp_kinetis_port, label)),	\
		.sda_mux_name = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, sda_gpios),		\
			DT_PROP_BY_PHANDLE(DT_INST_PHANDLE(n, sda_gpios), \
					   nxp_kinetis_port, label)),	\
	}
#else
#define I2C_MCUX_LPI2C_RECOVERY_INIT(n) \
	static const struct mcux_lpi2c_recovery_config	\
	mcux_lpi2c_recovery_config_##n = NULL
#endif /* CONFIG_I2C_MCUX_LPI2C_RECOVER_BUS */

#define I2C_MCUX_LPI2C_INIT(n)						\
	I2C_MCUX_LPI2C_RECOVERY_INIT(n);				\
	static void mcux_lpi2c_config_func_##n(struct device *dev);	\
									\
	static const struct mcux_lpi2c_config mcux_lpi2c_config_##n = {	\
		.base = (LPI2C_Type *)DT_INST_REG_ADDR(n),		\
		.clock_name = DT_INST_CLOCKS_LABEL(n),			\
		.clock_subsys =						\
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
		.irq_config_func = mcux_lpi2c_config_func_##n,		\
		.recovery = &mcux_lpi2c_recovery_config_##n,		\
		.bitrate = DT_INST_PROP(n, clock_frequency),		\
		.bus_idle_timeout_ns =					\
			UTIL_AND(DT_INST_NODE_HAS_PROP(n, bus_idle_timeout),\
				 DT_INST_PROP(n, bus_idle_timeout)),	\
	};								\
									\
	static struct mcux_lpi2c_data mcux_lpi2c_data_##n;		\
									\
	DEVICE_AND_API_INIT(mcux_lpi2c_##n, DT_INST_LABEL(n),		\
			    &mcux_lpi2c_init, &mcux_lpi2c_data_##n,	\
			    &mcux_lpi2c_config_##n, POST_KERNEL,	\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &mcux_lpi2c_driver_api);			\
									\
	static void mcux_lpi2c_config_func_##n(struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    mcux_lpi2c_isr,				\
			    DEVICE_GET(mcux_lpi2c_##n), 0);		\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH(I2C_MCUX_LPI2C_INIT)
