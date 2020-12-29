/**
 * This file is part of the hoverboard-firmware-hack project.
 *
 * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
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

// Includes
#include <stdlib.h> // for abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "main.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;
extern UART_HandleTypeDef huart3;

extern uint8_t enable;                  // global variable for motor enable
extern volatile uint32_t timeoutCnt; // global variable for general timeout counter
extern int16_t batVoltage;              // global variable for battery voltage
extern int16_t board_temp_deg_c;
extern int16_t speedMotor;
extern int16_t curr_a_cnt_max;

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------

int16_t cmdBrake;                       // normalized input value. -1000 to 1000
int16_t cmdThrottle;                    // normalized input value. -1000 to 1000
int16_t inputBrake;                        // Non normalized input value
int16_t inputThrottle;                        // Non normalized input value

int16_t speedAvg;                      // average measured speed
int16_t speedAvgAbs;                   // average measured speed in absolute
uint8_t timeoutFlagADC = 0; // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
uint8_t timeoutFlagSerial = 0; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq = CTRL_MOD_REQ;  // Final control mode request

uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x1300, 1301, 1302, 1303, 1304, 1305,
		1306, 1307, 1308, 1309, 1310 };

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t inputMax;             // [-] Input target maximum limitation
static int16_t inputMin;             // [-] Input target minimum limitation

static uint8_t cur_spd_valid = 0;
static uint8_t inp_cal_valid = 0;

static uint8_t rx_buffer_R[SERIAL_BUFFER_SIZE]; // USART Rx DMA circular buffer
static uint32_t rx_buffer_R_len = ARRAY_LEN(rx_buffer_R);

static uint16_t timeoutCntSerial_R = 0; // Timeout counter for Rx Serial command
static uint8_t timeoutFlagSerial_R = 0; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

static SerialFromEscToDisplay feedback;
static SerialFromDisplayToEsc command;
static SerialFromDisplayToEsc command_raw;
static uint32_t command_len = sizeof(command);

static uint8_t brakePressed;

static uint8_t cruiseCtrlAcv = 0;
static uint8_t standstillAcv = 0;

// Matlab defines - from auto-code generation
//---------------
extern RT_MODEL *const rtM_Motor;
extern P rtP_Left; /* Block parameters (auto storage) */
extern DW rtDW_Motor; /* Observable states */
extern ExtU rtU_Motor; /* External inputs */
extern ExtY rtY_Motor; /* External outputs */

/* =========================== Initialization Functions =========================== */

void Input_Lim_Init(void) {     // Input Limitations - ! Do NOT touch !
	if (rtP_Left.b_fieldWeakEna) {
		inputMax = MAX(INPUT1_MAX, FIELD_WEAK_HI);
		inputMin = MIN(-INPUT1_MIN, -FIELD_WEAK_HI);
	} else {
		inputMax = INPUT1_MAX;
		inputMin = INPUT1_MIN;
	}
}

void Input_Init(void) {

	HAL_UART_Receive_DMA(&huart3, (uint8_t*) rx_buffer_R, sizeof(rx_buffer_R));
	UART_DisableRxErrors(&huart3);

#if KX
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    uint16_t writeCheck, i_max, n_max;
    HAL_FLASH_Unlock();
    EE_Init();            /* EEPROM Init */
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    if (writeCheck == FLASH_WRITE_KEY) {
      EE_ReadVariable(VirtAddVarTab[1] , &i_max);
      EE_ReadVariable(VirtAddVarTab[2], &n_max);
      rtP_Left.i_max  = i_max;
      rtP_Left.n_max  = n_max;
    } else { // Else If Input type is 3 (auto), identify the input type based on the values from config.h
      if (INPUT1_TYPE == 3) { INPUT1_TYP_CAL = checkInputType(INPUT1_MIN, INPUT1_MID, INPUT1_MAX); }
      if (INPUT2_TYPE == 3) { INPUT2_TYP_CAL = checkInputType(INPUT2_MIN, INPUT2_MID, INPUT2_MAX); }
    }
    HAL_FLASH_Lock();
  #endif
#endif

}

/**
 * @brief  Disable Rx Errors detection interrupts on UART peripheral (since we do not want DMA to be stopped)
 *         The incorrect data will be filtered based on the START_FRAME and checksum.
 * @param  huart: UART handle.
 * @retval None
 */
void UART_DisableRxErrors(UART_HandleTypeDef *huart) {
	CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE); /* Disable PE (Parity Error) interrupts */
	CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE); /* Disable EIE (Frame error, noise error, overrun error) interrupts */
}

/* =========================== General Functions =========================== */

void calcAvgSpeed(void) {

	// Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
	speedAvg = rtY_Motor.n_mot; // double because of the 32KHz freq

	// Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
	if (SPEED_COEFFICIENT & (1 << 16)) {
		speedAvg = -speedAvg;
	}
	speedAvgAbs = abs(speedAvg);
}

/*
 * Add Dead-band to a signal
 * This function realizes a dead-band around 0 and scales the input between [out_min, out_max]
 */
int addDeadBand(int16_t u, int16_t type, int16_t deadBand, int16_t in_min,
		int16_t in_mid, int16_t in_max, int16_t out_min, int16_t out_max) {
	switch (type) {
	case 0: // Input is ignored
		return 0;
	case 1: // Input is a normal pot
		return CLAMP(MAP(u, in_min, in_max, 0, out_max), 0, out_max);
	case 2: // Input is a mid resting pot
		if (u > in_mid - deadBand && u < in_mid + deadBand) {
			return 0;
		} else if (u > in_mid) {
			return CLAMP(MAP(u, in_mid + deadBand, in_max, 0, out_max), 0,
					out_max);
		} else {
			return CLAMP(MAP(u, in_mid - deadBand, in_min, 0, out_min), out_min,
					0);
		}
	default:
		return 0;
	}
}

/*
 * Update Maximum Motor Current Limit (via ADC1) and Maximum Speed Limit (via ADC2)
 * Procedure:
 * - press the power button for more than 5 sec and immediatelly after the beep sound press one more time shortly
 * - move and hold the pots to a desired limit position for Current and Speed
 * - press the power button to confirm or wait for the 10 sec timeout
 */
void updateCurSpdLim(void) {
#if KX
  if (speedAvgAbs > 5) {    // do not enter this mode if motors are spinning
    return;
  }

  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
  consoleLog("Torque and Speed limits update started...\r\n");

  int32_t  input1_fixdt = inputBrake << 16;
  int32_t  input2_fixdt = inputThrottle << 16;  
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // Wait for the power button press
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 sec timeout
    readInput();
    filtLowPass32(inputBrake, FILTER, &input1_fixdt);
    filtLowPass32(inputThrottle, FILTER, &input2_fixdt);
    HAL_Delay(5);
  }
  // Calculate scaling factors
  cur_factor = CLAMP((input1_fixdt - ((int16_t)INPUT1_MIN_CAL << 16)) / ((int16_t)INPUT1_MAX_CAL - (int16_t)INPUT1_MIN_CAL), 6553, 65535);    // ADC1, MIN_cur(10%) = 1.5 A 
  spd_factor = CLAMP((input2_fixdt - ((int16_t)INPUT2_MIN_CAL << 16)) / ((int16_t)INPUT2_MAX_CAL - (int16_t)INPUT2_MIN_CAL), 3276, 65535);    // ADC2, MIN_spd(5%)  = 50 rpm
      
  if (INPUT1_TYP_CAL != 0){
    // Update current limit
    rtP_Left.i_max = rtP_Right.i_max  = (int16_t)((I_MOT_MAX * A2BIT_CONV * cur_factor) >> 12);    // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid   = 1;  // Mark update to be saved in Flash at shutdown
  }

  if (INPUT2_TYP_CAL != 0){
    // Update speed limit
    rtP_Left.n_max = rtP_Right.n_max  = (int16_t)((N_MOT_MAX * spd_factor) >> 12);                 // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid  += 2;  // Mark update to be saved in Flash at shutdown
  }
  
  consoleLog("Limits: "); HAL_Delay(10);
  setScopeChannel(0, (int16_t)cur_spd_valid);     // 0 = No limit changed, 1 = Current limit changed, 2 = Speed limit changed, 3 = Both limits changed
  setScopeChannel(1, (int16_t)input1_fixdt);
  setScopeChannel(2, (int16_t)cur_factor);
  setScopeChannel(3, (int16_t)rtP_Left.i_max);
  setScopeChannel(4, (int16_t)0);
  setScopeChannel(5, (int16_t)input2_fixdt);
  setScopeChannel(6, (int16_t)spd_factor);
  setScopeChannel(7, (int16_t)rtP_Left.n_max);
  consoleScope();

#endif
#endif
}

/*
 * Save Configuration to Flash
 * This function makes sure data is not lost after power-off
 */
void saveConfig() {
	if (inp_cal_valid || cur_spd_valid) {
		HAL_FLASH_Unlock();
		EE_WriteVariable(VirtAddVarTab[0], FLASH_WRITE_KEY);
		EE_WriteVariable(VirtAddVarTab[1], rtP_Left.i_max);
		EE_WriteVariable(VirtAddVarTab[2], rtP_Left.n_max);
		HAL_FLASH_Lock();
	}
}

/*
 * Standstill Hold Function
 * This function uses Cruise Control to provide an anti-roll functionality at standstill.
 * Only available and makes sense for FOC VOLTAGE or FOC TORQUE mode.
 * 
 * Input:  none
 * Output: standstillAcv
 */
void standstillHold(void) {
#if defined(STANDSTILL_HOLD_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ != SPD_MODE)
    if (!rtP_Left.b_cruiseCtrlEna) {                          // If Stanstill in NOT Active -> try Activation
      if (((cmdBrake > 50 || cmdThrottle < -50) && speedAvgAbs < 30)     // Check if Brake is pressed AND measured speed is small
          || (cmdThrottle < 20 && speedAvgAbs < 5)) {                // OR Throttle is small AND measured speed is very small
        rtP_Left.n_cruiseMotTgt   = 0;
        rtP_Right.n_cruiseMotTgt  = 0;
        rtP_Left.b_cruiseCtrlEna  = 1;
        rtP_Right.b_cruiseCtrlEna = 1;
        standstillAcv = 1;
      } 
    }
    else {                                                    // If Stanstill is Active -> try Deactivation
      if (cmdBrake < 20 && cmdThrottle > 50 && !cruiseCtrlAcv) {         // Check if Brake is released AND Throttle is pressed AND no Cruise Control
        rtP_Left.b_cruiseCtrlEna  = 0;
        rtP_Right.b_cruiseCtrlEna = 0;
        standstillAcv = 0;
      }
    }
#endif
}

/*
 * Electric Brake Function
 * In case of TORQUE mode, this function replaces the motor "freewheel" with a constant braking when the input torque request is 0.
 * This is useful when a small amount of motor braking is desired instead of "freewheel".
 * 
 * Input: speedBlend = fixdt(0,16,15)
 * Output: cmd2 (Throtle) with brake component included
 */
void electricBrake(uint16_t speedBlend) {
#if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
	int16_t brakeVal;

	// Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving)
	if (speedAvg > 0) {
		brakeVal = (int16_t) ((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
	} else {
		brakeVal = (int16_t) (( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
	}

	// Calculate the new cmd2 with brake component included
	if (cmdThrottle >= 0 && cmdThrottle < ELECTRIC_BRAKE_THRES) {
		cmdThrottle =
				MAX(brakeVal,
						((ELECTRIC_BRAKE_THRES - cmdThrottle) * brakeVal) / ELECTRIC_BRAKE_THRES);
	} else if (cmdThrottle >= -ELECTRIC_BRAKE_THRES && cmdThrottle < 0) {
		cmdThrottle =
				MIN(brakeVal,
						((ELECTRIC_BRAKE_THRES + cmdThrottle) * brakeVal) / ELECTRIC_BRAKE_THRES);
	} else if (cmdThrottle >= ELECTRIC_BRAKE_THRES) {
		cmdThrottle =
				MAX(brakeVal,
						((cmdThrottle - ELECTRIC_BRAKE_THRES) * inputMax) / (inputMax - ELECTRIC_BRAKE_THRES));
	} else {  // when (cmd2 < -ELECTRIC_BRAKE_THRES)
		cmdThrottle =
				MIN(brakeVal,
						((cmdThrottle + ELECTRIC_BRAKE_THRES) * inputMin) / (inputMin + ELECTRIC_BRAKE_THRES));
	}
#endif
}

/*
 * Cruise Control Function
 * This function activates/deactivates cruise control.
 * 
 * Input: button (as a pulse)
 * Output: cruiseCtrlAcv
 */
void cruiseControl(uint8_t button) {
#ifdef CRUISE_CONTROL_SUPPORT
	if (button && !rtP_Left.b_cruiseCtrlEna) {       // Cruise control activated
		rtP_Left.n_cruiseMotTgt = rtY_Motor.n_mot;
		rtP_Left.b_cruiseCtrlEna = 1;
		cruiseCtrlAcv = 1;
	} else if (button && rtP_Left.b_cruiseCtrlEna && !standstillAcv) { // Cruise control deactivated if no Standstill Hold is active
		rtP_Left.b_cruiseCtrlEna = 0;
		cruiseCtrlAcv = 0;
	}
#endif
}

/* =========================== Poweroff Functions =========================== */

void poweroff(void) {
	enable = 0;
	HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
	while (1) {
	}
}

void poweroffPressCheck(void) {
	if (HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) {
		uint16_t cnt_press = 0;
		while (HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) {
			HAL_Delay(10);
			cnt_press++;
		}
		if (cnt_press
				>= 1 * 100) {               // Check if press is more than 2 sec
#if KX
        HAL_Delay(1000);
        if (HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) {  // Double press: Adjust Max Current, Max Speed
          while(HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) { HAL_Delay(10); }
          updateCurSpdLim();
        } else {                                          // Long press: Calibrate ADC Limits
          adcCalibLim();
        }
      } else {
#endif
			// Short press: power off
			enable = 0;
			HAL_Delay(10);
			poweroff();
		}
	}
}


/* =========================== Read Functions =========================== */

/*
 * Function to read the raw Input values from various input devices
 */
void readInput(void) {

	// throttle / brake commands
	inputBrake = command.Brake << 2;
	inputThrottle = command.Throttle << 2;

	// speed limiter
	uint16_t speed_limit = command.Speed_limit;
	if (speed_limit > 0)
		rtP_Left.n_max = (speed_limit * 10) << 4;

	// WARNING -- NOT final usage -- test only
	if (command.Ligth_power == 1)
		ctrlModReq = VLT_MODE;
	else if (command.Ligth_power == 2)
		ctrlModReq = SPD_MODE; // can use SPD_MODE or VLT_MODE
	else
		ctrlModReq = TRQ_MODE;

	if (command.Power_ON == 0x01)
		poweroff();

	//TIM1->CNT = command.Lock * 20;

	timeoutCnt = 0;

}

/*
 * Function to calculate the command to the motors. This function also manages:
 * - timeout detection
 * - MIN/MAX limitations and deadband
 */
void readCommand(void) {
	readInput();

	if (timeoutCntSerial_R++ >= SERIAL_TIMEOUT) {     // Timeout qualification
		timeoutFlagSerial_R = 1;                        // Timeout detected
		timeoutCntSerial_R = SERIAL_TIMEOUT;       // Limit timout counter value
	}
	timeoutFlagSerial = timeoutFlagSerial_R;

	cmdBrake = inputBrake;
	cmdThrottle = inputThrottle;

	brakePressed = (uint8_t) (cmdBrake > 50);

	if (timeoutFlagADC || timeoutFlagSerial || timeoutCnt > TIMEOUT) { // In case of timeout bring the system to a Safe State
		// commented : not safe with escooters
		//ctrlModReq = OPEN_MODE; // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way

		cmdBrake = 0;
		cmdThrottle = 0;
	} else {
		// commented : not safe with escooters
		//ctrlModReq = ctrlModReqRaw;                   // Follow the Mode request
	}

#if defined(CRUISE_CONTROL_SUPPORT) && (defined(SUPPORT_BUTTONS) || defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT))
      cruiseControl(button1);                                           // Cruise control activation/deactivation
#endif
}

/*
 * Check for new data received on USART3 with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart3_rx_check(void) {

	static uint32_t old_pos;
	uint32_t pos;
	pos = rx_buffer_R_len - __HAL_DMA_GET_COUNTER(huart3.hdmarx); // Calculate current position in buffer

	uint8_t *ptr;
	if (pos != old_pos) {                       // Check change in received data
		ptr = (uint8_t*) &command_raw; // Initialize the pointer with command_raw address
		if (pos > old_pos && (pos - old_pos) == command_len) { // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
			memcpy(ptr, &rx_buffer_R[old_pos], command_len); // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
			usart_process_command(&command_raw, &command, 3);    // Process data
		} else if ((rx_buffer_R_len - old_pos + pos) == command_len) { // "Overflow" buffer mode: check if data length equals expected length
			memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos); // First copy data from the end of buffer
			if (pos > 0) {        // Check and continue with beginning of buffer
				ptr += rx_buffer_R_len - old_pos; // Move to correct position in command_raw
				memcpy(ptr, &rx_buffer_R[0], pos);        // Copy remaining data
			}
			usart_process_command(&command_raw, &command, 3);    // Process data
		}
	}

	old_pos = pos;                                        // Update old position
	if (old_pos == rx_buffer_R_len) { // Check and manually update if we reached end of buffer
		old_pos = 0;
	}

}

/*
 * Process command Rx data
 * - if the command_in data is valid (correct START_FRAME and checksum) copy the command_in to command_out
 */
void usart_process_command(SerialFromDisplayToEsc *command_in,
		SerialFromDisplayToEsc *command_out, uint8_t usart_idx) {

	uint8_t checksum;
	if (command_in->Frame_start == SERIAL_START_FRAME_DISPLAY_TO_ESC) {
		checksum = (uint16_t) (command_in->Frame_start //
		//
				^ command_in->Type                        //
				^ command_in->Destination                 //
				^ command_in->Number_of_ESC               //
				^ command_in->BMS_protocol                //
				^ command_in->ESC_Jumps                   //
				^ command_in->Display_Version_Maj         //
				^ command_in->Display_Version_Main        //
				^ command_in->Power_ON                    //
				^ command_in->Throttle                    //
				^ command_in->Brake                       //
				^ command_in->Torque                      //
				^ command_in->Brake_torque                //
				^ command_in->Lock                        //
				^ command_in->Regulator                   //
				^ command_in->Motor_direction             //
				^ command_in->Hall_sensors_direction      //
				^ command_in->Ligth_power                 //
				^ command_in->Max_temperature_reduce      //
				^ command_in->Max_temperature_shutdown    //
				^ command_in->Speed_limit                 //
				^ command_in->Motor_start_speed           //
		);
		if (command_in->CRC8 == checksum) {
			*command_out = *command_in;
			if (usart_idx == 3) {      // Sideboard USART3
				timeoutCntSerial_R = 0;        // Reset timeout counter
				timeoutFlagSerial_R = 0;        // Clear timeout flag
			}
		}
	}
}

void usart_send_from_esc_to_display() {
	/*
	 feedback.start = (uint16_t) SERIAL_START_FRAME_ESC_TO_DISPLAY;
	 feedback.cmd1 = (int16_t) cmd1;
	 feedback.cmd2 = (int16_t) cmd2;
	 feedback.currDC = (int16_t) analog.curr_dc;
	 feedback.speedMeas = (int16_t) rtY_Motor.n_mot * 2; // dirty fix for PWM running at 32KHz
	 feedback.batVoltage = (int16_t) (batVoltage * BAT_CALIB_REAL_VOLTAGE
	 / BAT_CALIB_ADC);
	 feedback.boardTemp = (int16_t) board_temp_deg_c;
	 feedback.currPhA = (int16_t) curr_a_cnt_max;
	 feedback.speedMotor = (int16_t) speedMotor;
	 */

	int16_t batVoltageMillivolts = (int16_t) (batVoltage
			* BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC);
	uint16_t rpm = rtY_Motor.n_mot;

	feedback.Frame_start = (uint16_t) SERIAL_START_FRAME_ESC_TO_DISPLAY;
	feedback.Type = 0x01;
	feedback.ESC_Version_Maj = 0x00;
	feedback.ESC_Version_Min = 0x01;
	feedback.Throttle = cmdThrottle >> 2;
	feedback.Brake = cmdBrake >> 2;
	feedback.Controller_Voltage_LSB = batVoltageMillivolts & 0xff;
	feedback.Controller_Voltage_MSB = (batVoltageMillivolts >> 8) & 0xff;
	feedback.Controller_Current_LSB = analog.curr_dc & 0xff;
	feedback.Controller_Current_MSB = (analog.curr_dc >> 8) & 0xff;
	feedback.MOSFET_temperature = board_temp_deg_c / 10;
	feedback.ERPM_LSB = rpm & 0xff;
	feedback.ERPM_MSB = (rpm >> 8) & 0xff;
	//feedback.Lock_status                                  ;
	//feedback.Ligth_status                                 ;
	//feedback.Regulator_status                             ;
	//feedback.Phase_1_current_max_LSB                      ;
	//feedback.Phase_1_current_max_MSB                      ;
	//feedback.Phase_1_voltage_max_LSB                      ;
	//feedback.Phase_1_voltage_max_MSB                      ;
	//feedback.BMS_Version_Maj                              ;
	//feedback.BMS_Version_Min                              ;
	//feedback.BMS_voltage_LSB                              ;
	//feedback.BMS_voltage_MSB                              ;
	//feedback.BMS_Current_LSB                              ;
	//feedback.BMS_Current_MSB                              ;

	//if (__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {

	feedback.CRC8 = (uint8_t) (
	//
			feedback.Frame_start	//
			^ feedback.Type	//
					^ feedback.ESC_Version_Maj	//
					^ feedback.ESC_Version_Min	//
					^ feedback.Throttle	//
					^ feedback.Brake	//
					^ feedback.Controller_Voltage_LSB	//
					^ feedback.Controller_Voltage_MSB	//
					^ feedback.Controller_Current_LSB	//
					^ feedback.Controller_Current_MSB	//
					^ feedback.MOSFET_temperature	//
					^ feedback.ERPM_LSB	//
					^ feedback.ERPM_MSB	//
					^ feedback.Lock_status	//
					^ feedback.Ligth_status	//
					^ feedback.Regulator_status	//
					^ feedback.Phase_1_current_max_LSB	//
					^ feedback.Phase_1_current_max_MSB	//
					^ feedback.Phase_1_voltage_max_LSB	//
					^ feedback.Phase_1_voltage_max_MSB	//
					^ feedback.BMS_Version_Maj	//
					^ feedback.BMS_Version_Min	//
					^ feedback.BMS_voltage_LSB	//
					^ feedback.BMS_voltage_MSB	//
					^ feedback.BMS_Current_LSB	//
					^ feedback.BMS_Current_MSB	//
					^ feedback.BMS_Cells_status_group_1	//
					^ feedback.BMS_Cells_status_group_2	//
					^ feedback.BMS_Cells_status_group_3	//
					^ feedback.BMS_Cells_status_group_4	//
					^ feedback.BMS_Cells_status_group_5	//
					^ feedback.BMS_Cells_status_group_6	//
					^ feedback.BMS_Cells_status_group_7	//
					^ feedback.BMS_Cells_status_group_8	//
					^ feedback.BMS_Cells_status_group_9	//
					^ feedback.BMS_Cells_status_group_10	//
					^ feedback.BMS_Cells_status_group_11	//
					^ feedback.BMS_Cells_status_group_12	//
					^ feedback.BMS_Cells_status_group_13	//
					^ feedback.BMS_Cells_status_group_14	//
					^ feedback.BMS_Cells_status_group_15	//
					^ feedback.BMS_Cells_status_group_16	//
					^ feedback.BMS_Cells_status_group_17	//
					^ feedback.BMS_Cells_status_group_18	//
					^ feedback.BMS_Cells_status_group_19	//
					^ feedback.BMS_Cells_status_group_20	//
					^ feedback.BMS_Cells_status_group_21	//
					^ feedback.BMS_Cells_status_group_22	//
					^ feedback.BMS_Cells_status_group_23	//
					^ feedback.BMS_Cells_status_group_24	//
					^ feedback.BMS_Battery_tempature_1	//
					^ feedback.BMS_Battery_tempature_2	//
					^ feedback.BMS_Charge_cycles_full_LSB	//
					^ feedback.BMS_Charge_cycles_full_MSB	//
					^ feedback.BMS_Charge_cycles_partial_LSB	//
					^ feedback.BMS_Charge_cycles_partial_MSB	//
					^ feedback.Errors_LSB	//
					^ feedback.Errors_MSB	//
			);

	HAL_UART_Transmit_DMA(&huart3, (uint8_t*) &feedback, sizeof(feedback));
}

/* =========================== Filtering Functions =========================== */

/* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
 * Max:  32767.99998474121
 * Min: -32768
 * Res:  1.52587890625e-05
 *
 * Inputs:       u     = int16 or int32
 * Outputs:      y     = fixdt(1,32,16)
 * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
 *
 * Example:
 * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
 * filtLowPass16(u, 52429, &y);
 * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
 */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
	int64_t tmp;
	tmp = ((int64_t) ((u << 4) - (*y >> 12)) * coef) >> 4;
	tmp = CLAMP(tmp, -2147483648LL, 2147483647LL); // Overflow protection: 2147483647LL = 2^31 - 1
	*y = (int32_t) tmp + (*y);
}

/* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
 * Inputs:       u     = int16
 * Outputs:      y     = fixdt(1,16,4)
 * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
 */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
	int16_t q0;
	int16_t q1;

	q0 = (u << 4) - *y;

	if (q0 > rate) {
		q0 = rate;
	} else {
		q1 = -rate;
		if (q0 < q1) {
			q0 = q1;
		}
	}

	*y = q0 + *y;
}

/* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL);
 * Inputs:       rtu_speed, rtu_steer                  = fixdt(1,16,4)
 * Outputs:      rty_speedL                = int16_t
 * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
 */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedL) {
	int16_t prodSpeed;
	int16_t prodSteer;
	int32_t tmp;

	prodSpeed = (int16_t) ((rtu_speed * (int16_t) SPEED_COEFFICIENT) >> 14);
	prodSteer = (int16_t) ((rtu_steer * (int16_t) STEER_COEFFICIENT) >> 14);

	tmp = prodSpeed - prodSteer;
	tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
	*rty_speedL = (int16_t) (tmp >> 4);       // Convert from fixed-point to int
	*rty_speedL = CLAMP(*rty_speedL, inputMin, inputMax);
}

/**
 * @brief Overclock ADC to 16MHz
 * @retval None
 */
void OverclockADC(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		__disable_irq();
		while (1) {
		}
	}
}
