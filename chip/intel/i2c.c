/* --------------------------------------------------------------------
**
** Synopsys DesignWare AMBA Software Driver Kit and
** documentation (hereinafter, "Software") is an Unsupported
** proprietary work of Synopsys, Inc. unless otherwise expressly
** agreed to in writing between Synopsys and you.
**
** The Software IS NOT an item of Licensed Software or Licensed
** Product under any End User Software License Agreement or Agreement
** for Licensed Product with Synopsys or any supplement thereto. You
** are permitted to use and redistribute this Software in source and
** binary forms, with or without modification, provided that
** redistributions of source code must retain this notice. You may not
** view, use, disclose, copy or distribute this file or any information
** contained herein except pursuant to this license grant from Synopsys.
** If you do not agree with this notice, including the disclaimer
** below, then you are not authorized to use the Software.
**
** THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
** BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
** FOR A PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL
** SYNOPSYS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
** EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
** PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
** PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
** OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
** USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
** DAMAGE.
**
** --------------------------------------------------------------------
*/

/*******************************************************************************
 *
 * Modifications Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 ******************************************************************************/

#include <i2c.h>

#define MS_100 100
#define MS_10 10

typedef struct {
	uint32_t BASE;          /* base address of device register set */
	uint16_t fifo_depth;
	/* Transmitted bytes. */
	uint32_t total_read_bytes;
	uint32_t total_write_bytes;
	uint32_t tx_len;
	uint32_t rx_len;
	uint32_t rx_tx_len;     /* tx_len + rx_len */
	uint8_t *i2c_write_buff;
	uint8_t *i2c_read_buff;
	volatile uint8_t tx_watermark;  /* TX watermark level */
	volatile uint8_t rx_watermark;  /* RX watermark level */
	/* Config params */
	I2C_SPEED speed;
	I2C_ADDR_MODE addr_mode;
	I2C_MODE_TYPE mode;
	uint32_t slave_addr;
	/* Slave specific */
	SOC_I2C_SLAVE_MODE slave_mode;
} i2c_internal_data_t;

/* device config keeper */
static i2c_internal_data_t devices[2];

#ifdef CONFIG_I2C_READ
static void soc_recv_data(i2c_internal_data_t *dev)
{
	uint32_t i, rx_cnt = 0;

	if (!dev->rx_len) {
		return;
	}

	rx_cnt = MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_RXFLR);

	if (rx_cnt > dev->rx_len) {
		rx_cnt = dev->rx_len;
	}
	for (i = 0; i < rx_cnt; i++) {
		dev->i2c_read_buff[i] =
			MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_DATA_CMD);
	}
	dev->i2c_read_buff += i;
	dev->rx_len -= i;
	dev->total_read_bytes += i;
}
#endif /* CONFIG_I2C_READ */

static DRIVER_API_RC i2c_fill_fifo(i2c_internal_data_t *dev)
{
	uint32_t i, j = 0, tx_cnt, data = DRV_RC_OK;

	if (!dev->rx_tx_len) {
		return DRV_RC_FAIL;
	}

	tx_cnt = dev->fifo_depth - MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TXFLR);
	if (tx_cnt > dev->rx_tx_len) {
		tx_cnt = dev->rx_tx_len;
	}

	for (i = 0; i < tx_cnt; i++) {
		if (dev->tx_len > 0) {  /* something to transmit */
			data = dev->i2c_write_buff[i];

			if (dev->tx_len == 1) { /* last byte to write */
				/* repeated start if something to read after */
				if (dev->rx_len > 0)
					data |= IC_RESTART_BIT;
				else {
					data |= IC_STOP_BIT;
				}
			}
			dev->tx_len -= 1;
			dev->total_write_bytes += 1;
		} else {        /* something to read */
			data = IC_CMD_BIT;
			if (dev->rx_tx_len == 1) {      /* last dummy byte to write */
				data |= IC_STOP_BIT;
			}
		}
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_DATA_CMD) = data;
		if (data & IC_STOP_BIT) {
			while ((MMIO_REG_VAL_FROM_BASE
					(dev->BASE, IC_RAW_INTR_STAT)
				& IC_STOP_DET) != IC_STOP_DET) {
				j++;
				mdelay(1);
				if (j >= MS_100) {
					return DRV_RC_TIMEOUT;
				}
			}
		}
		dev->rx_tx_len -= 1;
	}
	dev->i2c_write_buff += i;
	return DRV_RC_OK;
}

DRIVER_API_RC i2c_set_config(SOC_I2C_CONTROLLER controller_id,
			     i2c_cfg_data_t *	config)
{
	i2c_internal_data_t *dev;

	/* set current base based on config */
	if (controller_id == SOC_I2C_0) {
		dev = &devices[0];
		dev->BASE = SOC_I2C_0_BASE;
	} else {
		dev = &devices[1];
		dev->BASE = SOC_I2C_1_BASE;
	}

	/* Copy passed in config data locally */
	dev->speed = config->speed;
	dev->addr_mode = config->addressing_mode;
	dev->mode = config->mode_type;

	/* Set reset values (moved from reset dev - was called only once) */
	dev->rx_watermark = 7;  /* TODO test different watermark levels */
	dev->tx_watermark = 2;
	dev->fifo_depth = 16;

	/* Clear interrupts */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

	return DRV_RC_OK;
}

static DRIVER_API_RC i2c_setup_device(i2c_internal_data_t *dev)
{
	volatile uint32_t ic_con = 0;
	DRIVER_API_RC rc = DRV_RC_OK;
	uint32_t i = 0;

	/* Setup IC_CON */
	/* Set master or slave mode - (initialisation = slave) */
	if (I2C_MASTER == dev->mode) {
		ic_con |= IC_SLAVE_DISABLE_BIT; /* SET TRUE */
		ic_con |= IC_MASTER_EN_BIT;
	} else
		return DRV_RC_FAIL;

	/* Set restart - so far compile time option - (initialisation = OFF) */
	ic_con |= IC_RESTART_EN_BIT;

	/* Set addressing mode - (initialisation = 7 bit) */
	if (I2C_10_Bit == dev->addr_mode) {
		ic_con |= IC_MASTER_ADDR_MODE_BIT;
	}

	/* Set speed */
	ic_con |= (dev->speed << 1);

	/* Set the IC_CON register */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) = ic_con;

	/* Wait for register to set */
	while (MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON) != ic_con) {
		i++;
		mdelay(1);
		if (i >= MS_10) {
			rc = DRV_RC_FAIL;
		}               /* Registers wasn't set successfuly - indicate I2C malfunction */
	}

	/* END of setup IC_CON */
	/* This is setter so prefering readability above speed */
	if (I2C_SLOW == dev->speed) {
		/* Set HCNT */
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_HCNT) =
			I2C_STD_HCNT;
		/* Set LCNT */
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STD_SCL_LCNT) =
			I2C_STD_LCNT;
	} else if (I2C_FAST == dev->speed) {
		/* Set HCNT */
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_HCNT) = I2C_FS_HCNT;
		/* Set LCNT */
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_FS_SCL_LCNT) = I2C_FS_LCNT;
	} else if (I2C_HS == dev->speed) {
		/* TODO change */
		rc = DRV_RC_INVALID_CONFIG;
	} else {
		rc = DRV_RC_FAIL;
	}

	/* Set RX fifo threshold level */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_RX_TL) = dev->rx_watermark;
	/* Set TX fifo threshold level */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TX_TL) = dev->tx_watermark;

	return rc;
}

DRIVER_API_RC i2c_write(SOC_I2C_CONTROLLER controller_id, uint8_t *data,
			uint32_t data_len, uint32_t slave_addr)
{
	return i2c_transfer(controller_id, data, data_len, 0, 0, slave_addr);
}

#ifdef CONFIG_I2C_READ
DRIVER_API_RC i2c_read(SOC_I2C_CONTROLLER controller_id, uint8_t *data,
		       uint32_t data_len, uint32_t slave_addr)
{
	return i2c_transfer(controller_id, 0, 0, data, data_len, slave_addr);
}
#endif

DRIVER_API_RC i2c_transfer(SOC_I2C_CONTROLLER controller_id,
			   uint8_t *data_write, uint32_t data_write_len,
			   uint8_t *data_read, uint32_t data_read_len,
			   uint32_t slave_addr)
{
	uint32_t i = 0;
	i2c_internal_data_t *dev = NULL;

	/* Controller we are using */
	if (controller_id == SOC_I2C_0) {
		dev = &devices[0];
	} else if (controller_id == SOC_I2C_1) {
		dev = &devices[1];
	} else {
		return DRV_RC_FAIL;
	}

	/* Check for activity */
	if (IC_ACTIVITY == (MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_STATUS)
			    & IC_ACTIVITY)) {
		return DRV_RC_FAIL;
	}

	dev->rx_len = data_read_len;
	dev->tx_len = data_write_len;
	dev->rx_tx_len = data_read_len + data_write_len;
	dev->i2c_write_buff = data_write;
	dev->i2c_read_buff = data_read;
	dev->total_read_bytes = 0;
	dev->total_write_bytes = 0;

	/* Disable device */
	/* Disable controller to be able to set TAR */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE) &= ~IC_ENABLE_BIT;

	while ((MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE_STATUS)
		& IC_ENABLE_BIT) == IC_ENABLE_BIT) {
		i++;
		mdelay(1);
		if (i >= MS_10) {
			return DRV_RC_FAIL;
		}               /* Registers wasn't set successfuly - indicate I2C malfunction */
	}

	/* Set config params */
	if (i2c_setup_device(dev))
		return DRV_RC_FAIL;

	/* Set slave address */
	if (I2C_MASTER == dev->mode) {
		MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_TAR) = slave_addr;
		dev->slave_mode = SLAVE_WRITE;
	} else {
		return DRV_RC_FAIL;
	}

	/* Disable interrupts */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_INTR_MASK) =
		SOC_DISABLE_ALL_I2C_INT;

	/* Enable controller */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE) |= IC_ENABLE_BIT;

	/* Wait for IC_ENABLE to set if necessary */
	while ((MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_ENABLE_STATUS)
		& IC_ENABLE_BIT) != IC_ENABLE_BIT) {
		i++;
		mdelay(1);
		if (i >= MS_10) {
			return DRV_RC_FAIL;
		}               /* Registers wasn't set successfuly - indicate I2C malfunction */
	}
	/* Clear interrupts */
	MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CLR_INTR);

	while (1) {
		if ((dev->rx_tx_len == 0) && (dev->rx_len == 0)
		    && (dev->rx_len == 0))
			break;
		/* Are we a master device? */
		if (IC_MASTER_EN_BIT ==
		    (MMIO_REG_VAL_FROM_BASE(dev->BASE, IC_CON)
		     & IC_MASTER_EN_BIT)) {
			/* Master TX */
			/* check with the threshold */
			if (IC_TXFLR != 0) {
				if (i2c_fill_fifo(dev))
					return DRV_RC_TIMEOUT;
			}
#ifdef CONFIG_I2C_READ
			if (IC_RXFLR != 0) {
				soc_recv_data(dev);
			}
#endif
		}
	}
	return DRV_RC_OK;
}
