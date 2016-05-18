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
#ifndef __LP5562_H_
#define __LP5562_H_

#include <stdint.h>
#include <stdbool.h>

enum lp_prescale {
	LP_PRESCALE_490_US = 0,
	LP_PRESCALE_15600_US = 1
};

enum lp_sign {
	LP_SIGN_INCREASE = 0,
	LP_SIGN_DECREASE
};

enum lp_engine {
	LP_ENGINE_1 = 1,        /**< BLUE ENGINE */
	LP_ENGINE_2,            /**< GREEN ENGINE */
	LP_ENGINE_3             /**< RED ENGINE */
};

/*! \brief	Sets the WRGB color value.
 *  \details For white color, we need to remap white PWM - with LED MAPPING register - to one of the 3 engines.
 * \param	_color	Red/Green/Blue color (0x00RRGGBB)
 */
void lp5562_set_color(uint32_t _color);

/*! \brief	Execute the pre-loaded pattern.
 * \param	_engine	the engine number where the pattern will be executed.
 */
void lp5562_run_patterns(enum lp_engine _engine);

/*! \brief	Load a pattern to the specified engine.
 * \param	_engine		The engine number where the pattern will be loaded.
 * \param	pattern_ptr	Pattern data pointer.
 * \param	_pattern_cnt	Number of patterns.
 */
void lp5562_engine_load(enum lp_engine _engine, uint16_t *pattern_ptr,
			uint8_t _pattern_cnt);

/*! \brief	Fills the 16 data with the RAMP_WAIT pattern.
 * \param	_prescale	Prescale to 490 us or 15.6 ms CLK cycle.
 * \param	_increment	Step number and pwm increment number.
 * \param	_step_time	Time between steps = prescale CLK cycle * _step_time.
 * \param	_sign		Increase or Decrease.
 */
uint16_t lp5562_fill_ramp_wait_pattern(enum lp_prescale _prescale,
				       uint8_t _increment, uint8_t _step_time,
				       enum lp_sign _sign);

/*! \brief	Fills the 16 data with the BRANCH pattern.
 * \param	_step_nb	Index to the SRAM command to loop.
 * \param	_loop_cnt	Iterations number.
 */
uint16_t lp5562_fill_branch_pattern(uint8_t _step_nb, uint8_t _loop_cnt);

/*! \brief	Fills the 16 data with the SET_PWM pattern.
 * \details	Will be Red Green or Blue depending on the engine where the command will be executed.
 * \param	_PWM_value	The RGB value.
 */
uint16_t lp5562_fill_pwm_pattern(uint8_t _PWM_value);

/*! \brief	Resets and configures the LP5562 device.
 */
void led_init(void);

/*! \brief	Sends a pattern of 3 second fix RED colored LED.
 */
void lp5562_notify_boot(void);

#endif /* __LP5562_H_ */
