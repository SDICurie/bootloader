/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Intel common I2C header
 */

#ifndef __I2C_H__
#define __I2C_H__


#define INT_I2C_0_MASK          (0x448)
#define INT_I2C_1_MASK          (0x44C)

#define SCSS_BASE               SCSS_REGISTER_BASE

#define INT_I2C_MASK_RW         (uint32_t)(0xb0800448)

/* QRK interrupt values */
#define QRK_I2C_INT_MASK        (0x01010100)

/* QRK interrupt values */

#define SOC_ENABLE_RX_TX_INT_I2C    (RX_DATA_NOT_READY | RX_OVER | RX_FULL | \
				     RX_FULL | TX_OVER | TX_EMPTY | TX_EMPTY | \
				     TX_ABRT | STOP_DET)
#define SOC_ENABLE_TX_INT_I2C       (TX_OVER | TX_EMPTY | TX_EMPTY | TX_ABRT | \
				     STOP_DET)

#define SOC_ENABLE_TX_INT_I2C_SLAVE (0x00000260)
#define SOC_ENABLE_RX_INT_I2C_SLAVE (0x00000204)
#define SOC_DISABLE_ALL_I2C_INT     (0x00000000)

/* IC_CON speed settings (bit 1:2) */

#define I2C_STD_SPEED   01
#define I2C_FAST_SPEED  10
#define I2C_HIGH_SPEED  11

/* IC_CON addressing settings (bit 4) */

#define I2C_7BIT_ADDR   0
#define I2C_10BIT_ADDR  1

/* IC_CON Low count and high count default values */
// TODO verify values for high and fast speed
#define I2C_STD_HCNT    (CONFIG_CLOCK_SPEED * 4)
#define I2C_STD_LCNT    (CONFIG_CLOCK_SPEED * 5)
#define I2C_FS_HCNT     ((CONFIG_CLOCK_SPEED * 6) / 8)
#define I2C_FS_LCNT     ((CONFIG_CLOCK_SPEED * 7) / 8)

/* IC_DATA_CMD Data transfer mode settings (bit 8) */
#define I2C_STATE_READY                 (0)
#define I2C_CMD_SEND                    (1 << 0)
#define I2C_CMD_RECV                    (1 << 1)
#define I2C_CMD_ERROR                   (1 << 2)

/* Reset vectors for configuration registers */
#define IC_CON_RST      ((uint32_t)0x7e)
#define IC_TAR_RST      ((uint32_t)0x55)

/* FIFO size */
#define FIFO_SIZE       16

/* APB I2C register offsets */
#define IC_CON                          (0x00)
#define IC_TAR                          (0x04)
#define IC_SAR                          (0x08)
#define IC_HS_MADDR                     (0x0c)
#define IC_DATA_CMD                     (0x10)
#define IC_STD_SCL_HCNT                 (0x14)
#define IC_STD_SCL_LCNT                 (0x18)
#define IC_FS_SCL_HCNT                  (0x1c)
#define IC_FS_SCL_LCNT                  (0x20)
#define IC_HS_SCL_HCNT                  (0x24)
#define IC_HS_SCL_LCNT                  (0x28)
#define IC_INTR_STAT                    (0x2c)
#define IC_INTR_MASK                    (0x30)
#define IC_RAW_INTR_STAT                (0x34)
#define IC_RX_TL                        (0x38)
#define IC_TX_TL                        (0x3c)
#define IC_CLR_INTR                     (0x40)
#define IC_CLR_RX_UNDER                 (0x44)
#define IC_CLR_RX_OVER                  (0x48)
#define IC_CLR_TX_OVER                  (0x4c)
#define IC_CLR_RD_REQ                   (0x50)
#define IC_CLR_TX_ABRT                  (0x54)
#define IC_CLR_RX_DONE                  (0x58)
#define IC_CLR_ACTIVITY                 (0x5c)
#define IC_CLR_STOP_DET                 (0x60)
#define IC_CLR_START_DET                (0x64)
#define IC_CLR_GEN_CALL                 (0x68)
#define IC_ENABLE                       (0x6c)
#define IC_STATUS                       (0x70)
#define IC_TXFLR                        (0x74)
#define IC_RXFLR                        (0x78)
#define IC_SDA_HOLD                     (0x7c)
#define IC_TX_ABRT_SOURCE               (0x80)
#define IC_SLV_DATA_NACK_ONLY           (0x84)
#define IC_DMA_CR                       (0x88)
#define IC_DMA_TDLR                     (0x8c)
#define IC_DMA_RDLR                     (0x90)
#define IC_SDA_SETUP                    (0x94)
#define IC_ACK_GENERAL_CALL             (0x98)
#define IC_ENABLE_STATUS                (0x9c)
#define IC_FS_SPKLEN                    (0xa0)
#define IC_HS_SPKLEN                    (0xa4)
#define IC_CLR_RESTART_DET              (0xa8)
#define IC_COMP_PARAM_1                 (0xf4)
#define IC_COMP_VERSION                 (0xf8)
#define IC_COMP_TYPE                    (0xfc)
// WARNING TODO Check whether this will be compile time or driver option
#define RESTART_ALLOWED                     0

/* Specific bits used to set certain regs */
#define IC_RESTART_BIT                  (1 << 10)
#define IC_CMD_BIT                      (1 << 8)        /* part of IC_DATA_CMD register
	                                                 * , sets direction of current byte
	                                                 * - set (1) = read, unset (0) =
	                                                 * write */
#define IC_STOP_BIT                     (1 << 9)        /* part of IC_DATA_CMD, by
	                                                 * setting this bit last byte
	                                                 * of transfer is indicated */
#define IC_TX_INTR_MODE                 (1 << 8)        /* part of IC_CON registers
	                                                 * - set TX interrupt mode */
#define IC_ENABLE_BIT                   (1 << 0)
#define IC_SLAVE_DISABLE_BIT            (1 << 6)
#define IC_MASTER_EN_BIT                (1 << 0)
#define IC_RESTART_EN_BIT               (1 << 5)
#define IC_MASTER_ADDR_MODE_BIT         (1 << 4)
#define IC_SLAVE_ADDR_MODE_BIT          (1 << 3)
#define IC_FIFO_RFF                     (1 << 4)        /* Receive FIFO completely full */
#define IC_FIFO_RFNE                    (1 << 3)        /* Receive FIFO not empty */
#define IC_FIFO_TFE                     (1 << 2)        /* Transmit FIFO completely empty */
#define IC_FIFO_TFNF                    (1 << 1)        /* Transmit FIFO not full */
#define IC_ACTIVITY                     (1 << 0)
#define IC_STOP_DET                     (1 << 9)

/* Out of convention */
#define IC_SPEED_POS                    2

#define ZERO_REG                        ((uint32_t)(0x00000000))

/* Clock gating */
#define CLK_I2C_0_ENABLE                (1 << 19)
#define CLK_I2C_1_ENABLE                (1 << 20)
#define CLK_I2C_0_DISABLE               (~CLK_I2C_0_ENABLE)
#define CLK_I2C_1_DISABLE               (~CLK_I2C_1_ENABLE)

/* Interrupt handler statuses - bit masking for IC_RAW_INTR_STATUS register -
 * passed by callbacks */
#define RX_DATA_NOT_READY  (1 << 0)     /* RX fifo not yet ready */
#define RX_OVER            (1 << 1)     /* RX fifo overflow */
#define RX_FULL            (1 << 2)     /* RX fifo full */
#define RX_READY           RX_FULL      /* ^ also means RX ready */
#define TX_OVER            (1 << 3)     /* TX fifo overflow */
#define TX_EMPTY           (1 << 4)     /* TX fifo empty */
#define TX_READY           TX_EMPTY     /* ^ also means TX ready */
#define RD_REQ             (1 << 5)     /* SLAVE - read request received */
#define TX_ABRT            (1 << 6)     /* TX aborted - TODO reason */
#define RX_DONE            (1 << 7)     /* SLAVE - read on master over */
#define ACTIVITY           (1 << 8)     /* Activity on I2C - automatically
	                                 * cleared by ISR */
#define STOP_DET           (1 << 9)     /* STOP condition on line */
#define START_DET          (1 << 10)    /* START condition on line */
#define GEN_CALL           (1 << 11)    /* General call issued - disabled */
#define RESTART_DET        (1 << 12)    /* SLAVE - restart condition -
	                                 * disabled */
#define MST_ON_HOLD        (1 << 13)    /* Master on hold - disabled */


#include <stdint.h>
#include <scss_registers.h>
#include <utils.h>

/**
 * @defgroup common_driver_i2c Common I2C
 * Inter Integrated Communication bus drivers API.
 * @ingroup common_drivers
 * @{
 */

/* I2C */
/*!
 * List of all controllers in system ( IA and SS )
 */

typedef enum {
	SOC_I2C_0 = 0,          /*!< General Purpose I2C controller 0, accessible by both
	                         * processing entities */
	SOC_I2C_1,              /*!< General Purpose I2C controller 1, accessible by
	                         * both processing entities */
} SOC_I2C_CONTROLLER_PF;

/**
 * API types.
 */
typedef enum {
	I2C_OK = 0,             /*!< I2C BUS OK */
	I2C_BUSY,               /*!< I2C BUS BUSY */
	I2C_TX_ABORT,           /*!< I2C BUS TX ABORT */
	I2C_TX_OVER,            /*!< I2C BUS TX OVER */
	I2C_RX_OVER,            /*!< I2C BUS RX OVER */
	I2C_RX_UNDER,           /*!< I2C BUS RX UNDER */
} DRIVER_I2C_STATUS_CODE;

/**
 * I2C speeds.
 */
typedef enum {
	I2C_SLOW = 1,           /*!< (1) 0-100 Khz - note: Starts at 1 to remove
	                         * need to translate before hardware write */
	I2C_FAST = 2,           /*!< (2) 400 Khz */
	I2C_HS = 3              /*!< (3) 3400 kbit/s mode added support for
	                         * HS mode available only in SoC block */
} I2C_SPEED;

/**
 * I2C addressing modes.
 */
typedef enum {
	I2C_7_Bit = 0,          /*!< (0) 7 bit  */
	I2C_10_Bit,             /*!< (1) 10 bit */
} I2C_ADDR_MODE;

/**
 * I2C mode types.
 */
typedef enum {
	I2C_MASTER = 0,         /*!< MASTER WRITE MODE */
	I2C_SLAVE               /*!< MASTER READ MODE */
} I2C_MODE_TYPE;

/**
 *  I2C controller configuration.
 *
 *  Driver instantiates one of these with given parameters for each I2C
 *  controller configured using the "ss_i2c_set_config" function.
 */
typedef struct i2c_cfg_data {
	I2C_SPEED speed;
	I2C_ADDR_MODE addressing_mode;  /*!< 7 bit / 10 bit addressing */
	I2C_MODE_TYPE mode_type;        /*!< Master or Slave */
} i2c_cfg_data_t;

#define SOC_I2C_CONTROLLER int

typedef enum {
	SLAVE_WRITE = 0,        /*!< SLAVE WRITE MODE */
	SLAVE_READ,             /*!< SLAVE READ MODE */
} SOC_I2C_SLAVE_MODE;

/**
 *  Function to configure specified I2C controller.
 *
 *  Configuration parameters must be valid or an error is returned -
 *  see return values below.
 *
 *  @param   controller_id   : I2C  controller_id identifier
 *  @param   config          : pointer to configuration structure
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED    - if device type is not
 *                                                   supported by this controller
 *           - DRV_RC_INVALID_CONFIG               - if any configuration
 *                                                   parameters are not valid
 *           - DRV_RC_CONTROLLER_IN_USE,           - if controller is in use
 *           - DRV_RC_CONTROLLER_NOT_ACCESSIBLE    - if controller is not
 *                                                   accessible from this core
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC i2c_set_config(SOC_I2C_CONTROLLER controller_id,
			     i2c_cfg_data_t *	config);

/**
 *  Function to retrieve configuration of specified I2C controller
 *
 *  @param   controller_id   : I2C controller_id identifier
 *  @param   config          : pointer to configuration structure to store
 *                             current setup
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC i2c_get_config(SOC_I2C_CONTROLLER controller_id,
			     i2c_cfg_data_t *	config);

/**
 *  Function to transmit a block of data to the specified I2C slave.
 *
 *  @param   controller_id   : I2C controller_id identifier
 *  @param   data            : pointer to data to write
 *  @param   data_len        : length of data to write
 *  @param   slave_addr      : I2C address of the slave to write data to
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC i2c_write(SOC_I2C_CONTROLLER controller_id, uint8_t *data,
			uint32_t data_len,
			uint32_t slave_addr);

/**
 *  Function to receive a block of data
 *
 *  If set as a master, this will receive 'data_len' bytes transmitted from slave.
 *  If set as a slave, this will receive any data sent by a master addressed to
 *  the this I2C address as configured in the "slave_adr" for this controller
 *  configuration, in which case 'data_len' indicates the amount of data received
 *  and 'data' holds the data.
 *
 *  @param   controller_id   : I2C controller_id identifier
 *  @param   data            : pointer to data to read
 *  @param   data_len        : length of data to read
 *  @param   slave_addr      : I2C address of the slave to read from
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC i2c_read(SOC_I2C_CONTROLLER controller_id, uint8_t *data,
		       uint32_t data_len,
		       uint32_t slave_addr);

/**
 *  Function to transmit and receive a block of data to the specified I2C slave
 *  with repeated start
 *
 *  @param   controller_id   : I2C controller_id identifier
 *  @param   data_write      : pointer to data to write
 *  @param   data_write_len  : length of data to write
 *  @param   data_read       : pointer to data to read
 *  @param   data_read_len   : length of data to read
 *  @param   slave_addr      : I2C address of the slave to write data to
 *
 *  @return
 *           - DRV_RC_OK on success
 *           - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC i2c_transfer(SOC_I2C_CONTROLLER controller_id,
			   uint8_t *data_write, uint32_t data_write_len,
			   uint8_t *data_read,
			   uint32_t data_read_len,
			   uint32_t slave_addr);

/**
 *  Function to determine controllers current state
 *
 *  @param   controller_id   : I2C controller identifier
 *
 *  @return
 *           - I2C_OK   - controller ready
 *           - I2C_BUSY - controller busy
 */
DRIVER_I2C_STATUS_CODE i2c_status(SOC_I2C_CONTROLLER controller_id);

/** @} */


#endif /* __I2C_H__ */
