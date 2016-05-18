/* INTEL CONFIDENTIAL Copyright 2014 Intel Corporation All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "lp5562/lp5562.h"
#include <i2c.h>

#define LP5562_ADDRESS 0x30     /* Can be 0x30, 0x31, 0x32, 0x33 according to ADDR_SEL0 and ADDR_SEL1 pins wiring */
#define LP5562_SRAM_SIZE        16

#define LP5562_RAMP_WAIT_MASK 0x0000
#define LP5562_SET_PWM_MASK 0x4000
#define LP5562_BRANCH_MASK 0xA000
#define LP5562_END_MASK 0xC000
#define LP5562_TRIGGER_MASK 0xE000
#define LP5562_GOTO_START_CMD 0x0000

#define LP5562_REG_ENABLE       0x00
#define LP5562_REG_OP_MODE      0x01
#define LP5562_REG_CONFIG       0x08
#define LP5562_REG_LED_MAP      0x70
#define LP5562_REG_BASE_COLOR 0x02
#define LP5562_REG_BASE_CURRENT 0x05
#define LP5562_REG_WHITE_COLOR  0x0E
#define LP5562_REG_WHITE_CURRENT        0x0F
#define LP5562_REG_ENG_PROG(n)  (0x10 + ((n) - 1) * 0x20)

#define LP5562_CMD_EXEC 2
#define LP5562_ENG_CMD(engine, cmd)     cmd << (6 - (engine) * 2)
#define LP5562_ENG_EXEC_ALL     (LP5562_ENG_CMD(LP_ENGINE_1, \
						LP5562_CMD_EXEC) | \
				 LP5562_ENG_CMD(LP_ENGINE_2, \
						LP5562_CMD_EXEC) | \
				 LP5562_ENG_CMD(LP_ENGINE_3, LP5562_CMD_EXEC))
#define LP5562_ENG_HALT_ALL     0

#define lp_write(data_ptr, size)        i2c_write(CONFIG_LP5562_I2C, data_ptr, \
						  size,	\
						  LP5562_ADDRESS)

enum lp_engine_ctrl {
	LP_ENGINE_CTRL_DISABLE = 0,
	LP_ENGINE_CTRL_LOAD,
	LP_ENGINE_CTRL_RUN,
	LP_ENGINE_CTRL_PWM_CTRL
};

struct s_cmd_ramp_wait {
	union {
		uint16_t cmd;
		struct {
			unsigned increment : 7;   /**< 0 is wait, 127 is maximum */
			unsigned sign : 1;                /**< Increment or decrement */
			unsigned step_time : 6;   /**< Limited to 63 */
			unsigned prescale : 1;    /**< Time prescaler */
			unsigned reserved : 1;
		};
	} u_rw_bits;
};

struct s_cmd_set_pwm {
	union {
		uint16_t cmd;
		struct {
			unsigned pwm_value : 8;
			unsigned reserved : 8;
		};
	} u_sp_bits;
};

struct s_cmd_branch {
	union {
		uint16_t cmd;
		struct {
			unsigned step_nb : 6;             /**< Pointer to the SRAM command number to be executed */
			unsigned reserved_0 : 1;
			unsigned loop_cnt : 6;            /**< Number of times we loop from the step_nb */
			unsigned reserved_1 : 3;
		};
	} u_b_bits;
};

struct s_cmd_end {
	union {
		uint16_t cmd;
		struct {
			unsigned reserved_0 : 11;
			unsigned reset : 1;
			unsigned interrupt : 1;
			unsigned reserved_1 : 3;
		};
	} u_e_bits;
};

struct s_cmd_trigger {
	union {
		uint16_t cmd;
		struct {
			unsigned reserved_0 : 1;
			unsigned send_trigger_engine : 3;
			unsigned reserved_1 : 3;
			unsigned wait_trigger_engine : 3;
			unsigned reserved_2 : 6;
		};
	} u_t_bits;
};

static uint8_t enable_state = 0;
static uint8_t mode_state = 0;

static void lp5562_poweron(void);
static void lp5562_poweroff(void);
static void lp5562_write(uint8_t addr, uint8_t val);

#if defined(CONFIG_LP5562_PATTERN)
static void lp5562_engine_ctrl(enum lp_engine _engine, enum lp_engine_ctrl _cmd);

void lp5562_run_patterns(enum lp_engine _engine)
{
	enable_state |= LP5562_ENG_EXEC_ALL;

	lp5562_write(LP5562_REG_ENABLE, enable_state);
	lp5562_engine_ctrl(_engine, LP_ENGINE_CTRL_RUN);
}

void lp5562_engine_load(enum lp_engine _engine, uint16_t *pattern_ptr,
			uint8_t _pattern_cnt)
{
	uint8_t addr;
	uint8_t *octet_ptr = (uint8_t *)pattern_ptr;

	lp5562_engine_ctrl(_engine, LP_ENGINE_CTRL_LOAD);

	addr = LP5562_REG_ENG_PROG(_engine);
	for (_pattern_cnt = _pattern_cnt * 2; _pattern_cnt;
	     _pattern_cnt -= 2, addr += 2, octet_ptr += 2) {
		lp5562_write(addr, *(octet_ptr + 1));
		lp5562_write(addr + 1, *(octet_ptr));
	}
}

uint16_t lp5562_fill_ramp_wait_pattern(enum lp_prescale _prescale,
				       uint8_t _increment, uint8_t _step_time,
				       enum lp_sign _sign)
{
	struct s_cmd_ramp_wait pattern;

	pattern.u_rw_bits.increment = _increment;
	pattern.u_rw_bits.prescale = _prescale;
	pattern.u_rw_bits.step_time = _step_time;
	pattern.u_rw_bits.sign = _sign;
	pattern.u_rw_bits.cmd |= LP5562_RAMP_WAIT_MASK;

	return (uint16_t)pattern.u_rw_bits.cmd;
}

uint16_t lp5562_fill_branch_pattern(uint8_t _step_nb, uint8_t _loop_cnt)
{
	struct s_cmd_branch pattern;

	pattern.u_b_bits.step_nb = _step_nb;
	pattern.u_b_bits.loop_cnt = _loop_cnt;
	pattern.u_b_bits.cmd |= LP5562_BRANCH_MASK;

	return (uint16_t)pattern.u_b_bits.cmd;
}

uint16_t lp5562_fill_pwm_pattern(uint8_t _pwm_value)
{
	struct s_cmd_set_pwm pattern;

	pattern.u_sp_bits.pwm_value = _pwm_value;
	pattern.u_sp_bits.cmd |= LP5562_SET_PWM_MASK;

	return (uint16_t)pattern.u_sp_bits.cmd;
}

static void lp5562_engine_ctrl(enum lp_engine		_engine,
			       enum lp_engine_ctrl	_cmd)
{
	/* clear the engine */
	mode_state &= ~LP5562_ENG_CMD(_engine, 0x03);
	/* write the engine value */
	mode_state |= LP5562_ENG_CMD(_engine, _cmd);

	lp5562_write(LP5562_REG_OP_MODE, mode_state);
}

#endif /* CONFIG_LP5562_PATTERN */

void lp5562_set_color(uint32_t _color)
{
	unsigned int i;
	uint8_t *color_ptr = (uint8_t *)&_color;
	uint8_t reg_addr = LP5562_REG_BASE_COLOR;

	/* Sets all the engines to direct PWM control */
	mode_state = 0;
	for (i = LP_ENGINE_1; i < LP_ENGINE_3 + 1; i++)
		mode_state |= LP5562_ENG_CMD(i, LP_ENGINE_CTRL_PWM_CTRL);
	lp5562_write(LP5562_REG_OP_MODE, mode_state);

	/* write RGB colors */
	for (i = 3; i; i--, color_ptr++, reg_addr++)
		lp5562_write(reg_addr, *color_ptr);
}

void lp5562_poweron(void)
{
	enable_state = 0x40;
	lp5562_write(LP5562_REG_ENABLE, 0x40);
	mdelay(1);
}

void lp5562_poweroff(void)
{
	enable_state = 0x0;
	lp5562_write(LP5562_REG_ENABLE, 0x0);
}

void led_init(void)
{
	lp5562_poweroff();
	lp5562_poweron();
}

void lp5562_notify_boot(void)
{
#if defined(CONFIG_LP5562_PATTERN)
	uint16_t pattern[3];

	/* PATTERN TEST */
	/* 500 ms fix pattern */
	pattern[0] = lp5562_fill_pwm_pattern(127);
	pattern[1] =
		lp5562_fill_ramp_wait_pattern(LP_PRESCALE_15600_US, 0, 32,
					      LP_SIGN_INCREASE);
	/* Repete 6 previous command makes a 3s LED ON */
	pattern[2] = lp5562_fill_branch_pattern(1, 6);

	/* engine 3 is RED */
	lp5562_engine_load(LP_ENGINE_3, (uint16_t *)&pattern, 3);
	lp5562_run_patterns(LP_ENGINE_3);
	mdelay(2000);
#endif /* CONFIG_LP5562_PATTERN */
}

static void lp5562_write(uint8_t addr, uint8_t val)
{
	uint8_t lp_cmd[2] = {
		addr,
		val
	};

	lp_write(lp_cmd, sizeof(lp_cmd));
}
