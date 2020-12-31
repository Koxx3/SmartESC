/*
 * This file implements FOC motor control.
 * This control method offers superior performanace
 * compared to previous cummutation method. The new method features:
 * ► reduced noise and vibrations
 * ► smooth torque output
 * ► improved motor efficiency -> lower energy consumption
 *
 * Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "main.h"
#include "debug.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

RT_MODEL rtM_Motor_; /* Real-time model */
RT_MODEL *const rtM_Motor = &rtM_Motor_;
extern P rtP_Left; /* Block parameters (auto storage) */
DW rtDW_Motor; /* Observable states */
ExtU rtU_Motor; /* External inputs */
ExtY rtY_Motor; /* External outputs */

int16_t curr_a_cnt_max = 0;

uint32_t counter = 0;
uint32_t counterLock = 0;
int16_t enableLock = 0;

// ###############################################################################

#if KX
static int16_t pwm_margin = 110;        /* This margin allows to always have a window in the PWM signal for proper Phase currents measurement */
                                        /* official firmware value */
#else
static int16_t pwm_margin = 110; // Xiaomi firmware value
#endif

analog_t analog;

extern TIM_HandleTypeDef htim1;

extern int16_t speedAvgAbs;
extern uint8_t ctrlModReq;
int32_t curDC_max = I_DC_MAX * 340;

volatile int pwm = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t enable = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin = 0;

static const uint16_t pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

static uint16_t offsetcount = 0;
static int offset_curr_a = 2000;
static int offset_curr_b = 2000;
static int offset_curr_c = 2000;
static int offset_volt_a = 0;
static int offset_volt_b = 0;
static int offset_volt_c = 0;

#define TEST_NB_SAMPLES 1000

#if BLDC_STORE_CURRENT_PH_A
int16_t adb_buffer_storage_phA[TEST_NB_SAMPLES];
int16_t adb_buffer_storage_phB[TEST_NB_SAMPLES];
int16_t adb_buffer_storage_phC[TEST_NB_SAMPLES];
uint16_t cpt = 0;
uint16_t captureSpeed = 1200;
#endif

uint8_T errCodeLeft;
int16_T motSpeedLeft;

uint32_t timeBegin, timeEnd, timeInDma;

int16_t voltageTimer = 0;
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt = (400 * BAT_CELLS * BAT_CALIB_ADC)
		/ BAT_CALIB_REAL_VOLTAGE << 16; // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

// =================================
// Init motor params
// =================================
void BLDC_Init(void) {
	/* Set BLDC controller parameters */
	rtP_Left.b_angleMeasEna = 0; // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
	rtP_Left.z_selPhaCurMeasABC = 0; // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
	rtP_Left.z_ctrlTypSel = CTRL_TYP_SEL;
	rtP_Left.b_diagEna = DIAG_ENA;
	rtP_Left.i_max = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
	rtP_Left.n_max = N_MOT_MAX << 4;                       // fixdt(1,16,4)
	rtP_Left.b_fieldWeakEna = FIELD_WEAK_ENA;
	rtP_Left.id_fieldWeakMax = (FIELD_WEAK_MAX * A2BIT_CONV) << 4; // fixdt(1,16,4)
	rtP_Left.a_phaAdvMax = PHASE_ADV_MAX << 4;                  // fixdt(1,16,4)
	rtP_Left.r_fieldWeakHi = FIELD_WEAK_HI << 4;                // fixdt(1,16,4)
	rtP_Left.r_fieldWeakLo = FIELD_WEAK_LO << 4;                // fixdt(1,16,4)

	/* Pack LEFT motor data into RTM */
	rtM_Motor->defaultParam = &rtP_Left;
	rtM_Motor->dwork = &rtDW_Motor;
	rtM_Motor->inputs = &rtU_Motor;
	rtM_Motor->outputs = &rtY_Motor;

	/* Initialize BLDC controllers */
	BLDC_controller_initialize(rtM_Motor);
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) {

	DMA1->IFCR = DMA_IFCR_CTCIF1;

	timeBegin = HAL_GetTick();

#if DEBUG_LED == BLDC_DMA
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
#endif

	// HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

	if (offsetcount < 1000) {  // calibrate ADC offsets
		offsetcount++;
		offset_curr_a = (adc_buffer.curr_a + offset_curr_a) / 2;
		offset_curr_b = (adc_buffer.curr_b + offset_curr_b) / 2;
		offset_curr_c = (adc_buffer.curr_c + offset_curr_c) / 2;
		offset_volt_a = (adc_buffer.volt_a + offset_volt_a) / 2;
		offset_volt_b = (adc_buffer.volt_b + offset_volt_b) / 2;
		offset_volt_c = (adc_buffer.volt_c + offset_volt_c) / 2;
		return;
	}

	// Get motor currents
	analog.curr_a_cnt = (offset_curr_a - adc_buffer.curr_a);
	analog.curr_b_cnt = (offset_curr_b - adc_buffer.curr_b);
	analog.curr_c_cnt = (offset_curr_c - adc_buffer.curr_c);
	analog.curr_a = analog.curr_a_cnt * A2BIT_CONV;
	analog.curr_b = analog.curr_b_cnt * A2BIT_CONV;
	analog.curr_c = analog.curr_c_cnt * A2BIT_CONV;

#if BLDC_STORE_CURRENT_PH_A
	if (speedAvgAbs > captureSpeed && cpt < TEST_NB_SAMPLES) {
		adb_buffer_storage_phA[cpt] = analog.curr_a;
		adb_buffer_storage_phB[cpt] = analog.curr_b;
		adb_buffer_storage_phC[cpt] = analog.curr_c;
		cpt++;
	}
#endif

	// compute DC current
	static int32_t filter_buffer;
	filtLowPass32(
			(ABS(analog.curr_a_cnt) + ABS(analog.curr_b_cnt)
					+ ABS(analog.curr_c_cnt)) / 3, 20, &filter_buffer);

	// curr_dc in mA
	analog.curr_dc_raw = (filter_buffer >> 16);
	analog.curr_dc = analog.curr_dc_raw * A2BIT_CONV / 9;

	// store max phase A current (raw data)
	if (analog.curr_a_cnt > curr_a_cnt_max)
		curr_a_cnt_max = analog.curr_a_cnt;

	// compute DC voltage
	voltageTimer++;
	if (voltageTimer % 1000 == 0) { // Filter battery voltage at a slower sampling rate
		filtLowPass32(adc_buffer.vbat, BAT_FILT_COEF, &batVoltageFixdt);
		batVoltage = (int16_t) (batVoltageFixdt >> 16); // convert fixed-point to integer
	}
	// Upper Voltage Cut Protection
	if ((adc_buffer.vbat >= BAT_MAX_VOLTAGE) && (!enableLock)) {
		enable = 0;
		enableLock = 1;

		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		counter = 0;
		counterLock = counter;
	} else if ((adc_buffer.vbat < (int16_t) BAT_MAX_VOLTAGE) && (enableLock) && (speedAvgAbs <= 0)) {
		enable = 1;
		enableLock = 0;

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		counterLock = 0;
	}

#if BLDC_CURRENT_LIMIT
	// Disable PWM when current limit is reached (current chopping)
	// This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
	// curDC_max 340 = 1A
	// curL_DC in mA
	if ((ABS(analog.curr_dc) > (curDC_max)) || enable == 0) {
		TIM1->BDTR &= ~TIM_BDTR_MOE;
	} else {
		TIM1->BDTR |= TIM_BDTR_MOE;
	}
#endif

	// ############################### MOTOR CONTROL ###############################

	int ul, vl, wl;
	static boolean_T OverrunFlag = false;

#if BLDC_ENABLE_LOOP

	/* Check for overrun */
	if (OverrunFlag) {
#if DEBUG_LED == BLDC_DMA
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
#endif
		return;
	}
	OverrunFlag = true;
#endif

	/* Make sure to stop BOTH motors in case of an error */
	enableFin = enable && !rtY_Motor.z_errCode && !enableLock;

	// ========================= LEFT MOTOR ============================
	// Get hall sensors values
	uint8_t hall_ul = !(HALL_A_GPIO_Port->IDR & HALL_A_Pin);
	uint8_t hall_vl = !(HALL_B_GPIO_Port->IDR & HALL_B_Pin);
	uint8_t hall_wl = !(HALL_C_GPIO_Port->IDR & HALL_C_Pin);

	/* Set motor inputs here */
	rtU_Motor.b_motEna = enableFin;
	rtU_Motor.z_ctrlModReq = ctrlModReq;
	rtU_Motor.r_inpTgt = pwm;
	rtU_Motor.b_hallA = hall_ul;
	rtU_Motor.b_hallB = hall_vl;
	rtU_Motor.b_hallC = hall_wl;
	rtU_Motor.i_phaAB = analog.curr_a_cnt;
	rtU_Motor.i_phaBC = analog.curr_b_cnt;
	// rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`

	/* Step the controller */
	BLDC_controller_step(rtM_Motor);

	/* Get motor outputs here */
	ul = rtY_Motor.DC_phaA;
	vl = rtY_Motor.DC_phaB;
	wl = rtY_Motor.DC_phaC;
	errCodeLeft = rtY_Motor.z_errCode;
	motSpeedLeft = rtY_Motor.n_mot;

#if BLDC_ENABLE_LOOP

	if (!enableLock) {

		/* Apply commands */
		TIM1->CCR1 = (uint16_t) CLAMP(ul + pwm_res / 2, pwm_margin,
				pwm_res - pwm_margin);
		TIM1->CCR2 = (uint16_t) CLAMP(vl + pwm_res / 2, pwm_margin,
				pwm_res - pwm_margin);
		TIM1->CCR3 = (uint16_t) CLAMP(wl + pwm_res / 2, pwm_margin,
				pwm_res - pwm_margin);
	}
#endif

#if DEBUG_LED == BLDC_DMA
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
#endif

	timeEnd = HAL_GetTick();
	timeInDma = timeEnd - timeBegin;

	counter++;
	/* Indicate task complete */
	OverrunFlag = false;

	// ###############################################################################

}
