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
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "setup.h"

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;
extern UART_HandleTypeDef huart3;

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerCount; // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq; // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern uint8_t nunchuk_data[6];
extern volatile uint32_t timeoutCnt; // global variable for general timeout counter
extern volatile uint32_t main_loop_counter;

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
// Matlab defines - from auto-code generation
//---------------
RT_MODEL rtM_Left_; /* Real-time model */
RT_MODEL *const rtM_Left = &rtM_Left_;

extern P rtP_Left; /* Block parameters (auto storage) */
DW rtDW_Left; /* Observable states */
ExtU rtU_Left; /* External inputs */
ExtY rtY_Left; /* External outputs */

//---------------

int16_t cmd1;                          // normalized input value. -1000 to 1000
int16_t cmd2;                          // normalized input value. -1000 to 1000
int16_t input1;                        // Non normalized input value
int16_t input2;                        // Non normalized input value

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
static int16_t INPUT_MAX;             // [-] Input target maximum limitation
static int16_t INPUT_MIN;             // [-] Input target minimum limitation

static uint8_t cur_spd_valid = 0;
static uint8_t inp_cal_valid = 0;
static uint16_t INPUT1_TYP_CAL = INPUT1_TYPE;
static uint16_t INPUT1_MIN_CAL = INPUT1_MIN;
static uint16_t INPUT1_MID_CAL = INPUT1_MID;
static uint16_t INPUT1_MAX_CAL = INPUT1_MAX;
static uint16_t INPUT2_TYP_CAL = INPUT2_TYPE;
static uint16_t INPUT2_MIN_CAL = INPUT2_MIN;
static uint16_t INPUT2_MID_CAL = INPUT2_MID;
static uint16_t INPUT2_MAX_CAL = INPUT2_MAX;

static uint8_t rx_buffer_R[SERIAL_BUFFER_SIZE]; // USART Rx DMA circular buffer
static uint32_t rx_buffer_R_len = ARRAY_LEN(rx_buffer_R);

static uint16_t timeoutCntSerial_R = 0; // Timeout counter for Rx Serial command
static uint8_t timeoutFlagSerial_R = 0; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

static SerialCommand command;
static SerialCommand command_raw;
static uint32_t command_len = sizeof(command);

static uint8_t brakePressed;

static uint8_t cruiseCtrlAcv = 0;
static uint8_t standstillAcv = 0;

/* =========================== Initialization Functions =========================== */

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

	/* Pack motor data into RTM */
	rtM_Left->defaultParam = &rtP_Left;
	rtM_Left->dwork = &rtDW_Left;
	rtM_Left->inputs = &rtU_Left;
	rtM_Left->outputs = &rtY_Left;

	/* Initialize BLDC controllers */
	BLDC_controller_initialize(rtM_Left);
}

void Input_Lim_Init(void) {     // Input Limitations - ! Do NOT touch !    
	if (rtP_Left.b_fieldWeakEna) {
		INPUT_MAX = MAX(1000, FIELD_WEAK_HI);
		INPUT_MIN = MIN(-1000, -FIELD_WEAK_HI);
	} else {
		INPUT_MAX = 1000;
		INPUT_MIN = -1000;
	}
}

void Input_Init(void) {
	UART3_Init();
	HAL_UART_Receive_DMA(&huart3, (uint8_t*) rx_buffer_R, sizeof(rx_buffer_R));
	UART_DisableRxErrors(&huart3);

#if KX
  #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    uint16_t writeCheck, i_max, n_max;
    HAL_FLASH_Unlock();
    EE_Init();            /* EEPROM Init */
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    if (writeCheck == FLASH_WRITE_KEY) {
      EE_ReadVariable(VirtAddVarTab[1] , &INPUT1_TYP_CAL);
      EE_ReadVariable(VirtAddVarTab[2] , &INPUT1_MIN_CAL);
      EE_ReadVariable(VirtAddVarTab[3] , &INPUT1_MID_CAL);
      EE_ReadVariable(VirtAddVarTab[4] , &INPUT1_MAX_CAL);
      EE_ReadVariable(VirtAddVarTab[5] , &INPUT2_TYP_CAL);
      EE_ReadVariable(VirtAddVarTab[6] , &INPUT2_MIN_CAL);
      EE_ReadVariable(VirtAddVarTab[7] , &INPUT2_MID_CAL);
      EE_ReadVariable(VirtAddVarTab[8] , &INPUT2_MAX_CAL);
      EE_ReadVariable(VirtAddVarTab[9] , &i_max);
      EE_ReadVariable(VirtAddVarTab[10], &n_max);
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
	speedAvg = rtY_Left.n_mot;

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

  int32_t  input1_fixdt = input1 << 16;
  int32_t  input2_fixdt = input2 << 16;  
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // Wait for the power button press
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 sec timeout
    readInput();
    filtLowPass32(input1, FILTER, &input1_fixdt);
    filtLowPass32(input2, FILTER, &input2_fixdt);
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
#ifdef VARIANT_TRANSPOTTER
    if (saveValue_valid) {
      HAL_FLASH_Unlock();
      EE_WriteVariable(VirtAddVarTab[0], saveValue);
      HAL_FLASH_Lock();
    }
  #endif
#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
	if (inp_cal_valid || cur_spd_valid) {
		HAL_FLASH_Unlock();
		EE_WriteVariable(VirtAddVarTab[0], FLASH_WRITE_KEY);
		EE_WriteVariable(VirtAddVarTab[1], INPUT1_TYP_CAL);
		EE_WriteVariable(VirtAddVarTab[2], INPUT1_MIN_CAL);
		EE_WriteVariable(VirtAddVarTab[3], INPUT1_MID_CAL);
		EE_WriteVariable(VirtAddVarTab[4], INPUT1_MAX_CAL);
		EE_WriteVariable(VirtAddVarTab[5], INPUT2_TYP_CAL);
		EE_WriteVariable(VirtAddVarTab[6], INPUT2_MIN_CAL);
		EE_WriteVariable(VirtAddVarTab[7], INPUT2_MID_CAL);
		EE_WriteVariable(VirtAddVarTab[8], INPUT2_MAX_CAL);
		EE_WriteVariable(VirtAddVarTab[9], rtP_Left.i_max);
		EE_WriteVariable(VirtAddVarTab[10], rtP_Left.n_max);
		HAL_FLASH_Lock();
	}
#endif
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
      if (((cmd1 > 50 || cmd2 < -50) && speedAvgAbs < 30)     // Check if Brake is pressed AND measured speed is small
          || (cmd2 < 20 && speedAvgAbs < 5)) {                // OR Throttle is small AND measured speed is very small
        rtP_Left.n_cruiseMotTgt   = 0;
        rtP_Right.n_cruiseMotTgt  = 0;
        rtP_Left.b_cruiseCtrlEna  = 1;
        rtP_Right.b_cruiseCtrlEna = 1;
        standstillAcv = 1;
      } 
    }
    else {                                                    // If Stanstill is Active -> try Deactivation
      if (cmd1 < 20 && cmd2 > 50 && !cruiseCtrlAcv) {         // Check if Brake is released AND Throttle is pressed AND no Cruise Control
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
 * Input: speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * Output: cmd2 (Throtle) with brake component included
 */
void electricBrake(uint16_t speedBlend, uint8_t reverseDir) {
#if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
    int16_t brakeVal;

    // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving) 
    if (speedAvg > 0) {
      brakeVal = (int16_t)((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
    } else {
      brakeVal = (int16_t)(( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);          
    }

    // Check if direction is reversed
    if (reverseDir) {
      brakeVal = -brakeVal;
    }

    // Calculate the new cmd2 with brake component included
    if (cmd2 >= 0 && cmd2 < ELECTRIC_BRAKE_THRES) {
      cmd2 = MAX(brakeVal, ((ELECTRIC_BRAKE_THRES - cmd2) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (cmd2 >= -ELECTRIC_BRAKE_THRES && cmd2 < 0) {
      cmd2 = MIN(brakeVal, ((ELECTRIC_BRAKE_THRES + cmd2) * brakeVal) / ELECTRIC_BRAKE_THRES);
    } else if (cmd2 >= ELECTRIC_BRAKE_THRES) {
      cmd2 = MAX(brakeVal, ((cmd2 - ELECTRIC_BRAKE_THRES) * INPUT_MAX) / (INPUT_MAX - ELECTRIC_BRAKE_THRES));
    } else {  // when (cmd2 < -ELECTRIC_BRAKE_THRES)
      cmd2 = MIN(brakeVal, ((cmd2 + ELECTRIC_BRAKE_THRES) * INPUT_MIN) / (INPUT_MIN + ELECTRIC_BRAKE_THRES));
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
		rtP_Left.n_cruiseMotTgt = rtY_Left.n_mot;
		rtP_Left.b_cruiseCtrlEna = 1;
		cruiseCtrlAcv = 1;
#if KX
      shortBeepMany(2, 1);                                              // 200 ms beep delay. Acts as a debounce also.
#endif
	} else if (button && rtP_Left.b_cruiseCtrlEna && !standstillAcv) { // Cruise control deactivated if no Standstill Hold is active
		rtP_Left.b_cruiseCtrlEna = 0;
		cruiseCtrlAcv = 0;
#if KX
      shortBeepMany(2, -1);
#endif
	}
#endif
}

/* =========================== Poweroff Functions =========================== */

void poweroff(void) {
#if KX
  enable = 0;
  consoleLog("-- Motors disabled --\r\n");
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  saveConfig();
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
#endif
	while (1) {
	}
}

void poweroffPressCheck(void) {
#if KX
	#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      uint16_t cnt_press = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        HAL_Delay(10);
        if (cnt_press++ == 5 * 100) { beepShort(5); }
      }
      if (cnt_press >= 5 * 100) {                         // Check if press is more than 5 sec
        HAL_Delay(1000);
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  // Double press: Adjust Max Current, Max Speed
          while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
          beepLong(8);
          updateCurSpdLim();
          beepShort(5);
        } else {                                          // Long press: Calibrate ADC Limits
          beepLong(16); 
          adcCalibLim();
          beepShort(5);
        }
      } else {                                            // Short press: power off
        poweroff();
      }
    }
  #elif defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
      beepShort(5);
      HAL_Delay(300);
      if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
        beepLong(5);
        HAL_Delay(350);
        poweroff();
      } else {
        setDistance += 0.25;
        if (setDistance > 2.6) {
          setDistance = 0.5;
        }
        beepShort(setDistance / 0.25);
        saveValue = setDistance * 1000;
        saveValue_valid = 1;
      }
    }
  #else
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;                                             // disable motors
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // wait until button is released
      poweroff();                                             // release power-latch
    }
  #endif
#endif
}

/* =========================== Read Functions =========================== */

/*
 * Function to read the raw Input values from various input devices
 */
void readInput(void) {

	input1 = command.steer;
	input2 = command.speed;
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

#if KX
#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
  cmd1 = addDeadBand(input1, INPUT1_TYP_CAL, INPUT1_DEADBAND, INPUT1_MIN_CAL, INPUT1_MID_CAL, INPUT1_MAX_CAL, INPUT_MIN, INPUT_MAX);
  #if !defined(VARIANT_SKATEBOARD)
    cmd2 = addDeadBand(input2, INPUT2_TYP_CAL, INPUT2_DEADBAND, INPUT2_MIN_CAL, INPUT2_MID_CAL, INPUT2_MAX_CAL, INPUT_MIN, INPUT_MAX);
  #else
    cmd2 = addDeadBand(input2, INPUT2_TYP_CAL, INPUT2_DEADBAND, INPUT2_MIN_CAL, INPUT2_MID_CAL, INPUT2_MAX_CAL, INPUT2_BRAKE, INPUT_MAX);
  #endif
#endif
#else
	cmd1 = input1;
	cmd2 = input2;
#endif

	brakePressed = (uint8_t) (cmd1 > 50);

	if (timeoutFlagADC || timeoutFlagSerial || timeoutCnt > TIMEOUT) { // In case of timeout bring the system to a Safe State
		ctrlModReq = OPEN_MODE; // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
		cmd1 = 0;
		cmd2 = 0;
	} else {
		ctrlModReq = ctrlModReqRaw;                   // Follow the Mode request
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
 * Process Rx debug user command input
 */
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
void usart_process_debug(uint8_t *userCommand, uint32_t len)
{
  for (; len > 0; len--, userCommand++) {
    if (*userCommand != '\n' && *userCommand != '\r') {   // Do not accept 'new line' and 'carriage return' commands
      consoleLog("-- Command received --\r\n");
      // handle_input(*userCommand);                      // -> Create this function to handle the user commands
    }
  }
}
#endif // SERIAL_DEBUG

/*
 * Process command Rx data
 * - if the command_in data is valid (correct START_FRAME and checksum) copy the command_in to command_out
 */
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in,
		SerialCommand *command_out, uint8_t usart_idx) {
#ifdef CONTROL_IBUS
    if (command_in->start == IBUS_LENGTH && command_in->type == IBUS_COMMAND) {
      ibus_chksum = 0xFFFF - IBUS_LENGTH - IBUS_COMMAND;
      for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i++) {
        ibus_chksum -= command_in->channels[i];
      }
      if (ibus_chksum == (uint16_t)((command_in->checksumh << 8) + command_in->checksuml)) {
        *command_out = *command_in;
        if (usart_idx == 2) {             // Sideboard USART2
          #ifdef CONTROL_SERIAL_USART2
          timeoutCntSerial_L  = 0;        // Reset timeout counter
          timeoutFlagSerial_L = 0;        // Clear timeout flag
          #endif
        } else if (usart_idx == 3) {      // Sideboard USART3
          #ifdef CONTROL_SERIAL_USART3
          timeoutCntSerial_R  = 0;        // Reset timeout counter
          timeoutFlagSerial_R = 0;        // Clear timeout flag
          #endif
        }
      }
    }
  #else
	uint16_t checksum;
	if (command_in->start == SERIAL_START_FRAME) {
		checksum = (uint16_t) (command_in->start ^ command_in->steer
				^ command_in->speed);
		if (command_in->checksum == checksum) {
			*command_out = *command_in;
			if (usart_idx == 2) {             // Sideboard USART2
#ifdef CONTROL_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // Reset timeout counter
        timeoutFlagSerial_L = 0;        // Clear timeout flag
        #endif
			} else if (usart_idx == 3) {      // Sideboard USART3
#ifdef CONTROL_SERIAL_USART3
				timeoutCntSerial_R = 0;        // Reset timeout counter
				timeoutFlagSerial_R = 0;        // Clear timeout flag
#endif
			}
		}
	}
#endif
}
#endif

/*
 * Process Sideboard Rx data
 * - if the Sideboard_in data is valid (correct START_FRAME and checksum) copy the Sideboard_in to Sideboard_out
 */
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
void usart_process_sideboard(SerialSideboard *Sideboard_in, SerialSideboard *Sideboard_out, uint8_t usart_idx)
{
  uint16_t checksum;
  if (Sideboard_in->start == SERIAL_START_FRAME) {
    checksum = (uint16_t)(Sideboard_in->start ^ Sideboard_in->pitch ^ Sideboard_in->dPitch ^ Sideboard_in->cmd1 ^ Sideboard_in->cmd2 ^ Sideboard_in->sensors);
    if (Sideboard_in->checksum == checksum) {
      *Sideboard_out = *Sideboard_in;
      if (usart_idx == 2) {             // Sideboard USART2
        #ifdef SIDEBOARD_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // Reset timeout counter
        timeoutFlagSerial_L = 0;        // Clear timeout flag
        #endif
      } else if (usart_idx == 3) {      // Sideboard USART3
        #ifdef SIDEBOARD_SERIAL_USART3
        timeoutCntSerial_R  = 0;        // Reset timeout counter
        timeoutFlagSerial_R = 0;        // Clear timeout flag
        #endif
      }
    }
  }
}
#endif

/* =========================== Sideboard Functions =========================== */

/*
 * Sideboard LEDs Handling
 * This function manages the leds behavior connected to the sideboard
 */
void sideboardLeds(uint8_t *leds) {
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
    // Enable flag: use LED4 (bottom Blue)
    // enable == 1, turn on led
    // enable == 0, blink led
    if (enable) {
      *leds |= LED4_SET;
    } else if (!enable && (main_loop_counter % 20 == 0)) {
      *leds ^= LED4_SET;
    }

    // Backward Drive: use LED5 (upper Blue)
    // backwardDrive == 1, blink led
    // backwardDrive == 0, turn off led
    if (backwardDrive && (main_loop_counter % 50 == 0)) {
      *leds ^= LED5_SET;
    }

    // Brake: use LED5 (upper Blue)
    // brakePressed == 1, turn on led
    // brakePressed == 0, turn off led
    #ifdef VARIANT_HOVERCAR
      if (brakePressed) {
        *leds |= LED5_SET;
      } else if (!brakePressed && !backwardDrive) {
        *leds &= ~LED5_SET;
      }
    #endif

    // Battery Level Indicator: use LED1, LED2, LED3
    if (main_loop_counter % BAT_BLINK_INTERVAL == 0) {              //  | RED (LED1) | YELLOW (LED3) | GREEN (LED2) |
      if (batVoltage < BAT_DEAD) {                                  //  |     0      |       0       |      0       |
        *leds &= ~LED1_SET & ~LED3_SET & ~LED2_SET;          
      } else if (batVoltage < BAT_LVL1) {                           //  |     B      |       0       |      0       |
        *leds ^= LED1_SET;
        *leds &= ~LED3_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL2) {                           //  |     1      |       0       |      0       |
        *leds |= LED1_SET;
        *leds &= ~LED3_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL3) {                           //  |     0      |       B       |      0       |
        *leds ^= LED3_SET;
        *leds &= ~LED1_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL4) {                           //  |     0      |       1       |      0       |
        *leds |= LED3_SET;
        *leds &= ~LED1_SET & ~LED2_SET;
      } else if (batVoltage < BAT_LVL5) {                           //  |     0      |       0       |      B       |
        *leds ^= LED2_SET;
        *leds &= ~LED1_SET & ~LED3_SET;
      } else {                                                      //  |     0      |       0       |      1       |
        *leds |= LED2_SET;
        *leds &= ~LED1_SET & ~LED3_SET;
      }
    }

    // Error handling
    // Critical error:  LED1 on (RED)     + high pitch beep (hadled in main)
    // Soft error:      LED3 on (YELLOW)  + low  pitch beep (hadled in main)
    if (rtY_Left.z_errCode || rtY_Right.z_errCode) {
      *leds |= LED1_SET;
      *leds &= ~LED3_SET & ~LED2_SET;
    }
    if (timeoutFlagADC || timeoutFlagSerial) {
      *leds |= LED3_SET;
      *leds &= ~LED1_SET & ~LED2_SET;
    }
  #endif
}

/*
 * Sideboard Sensor Handling
 * This function manages the sideboards photo sensors.
 * In non-hoverboard variants, the sensors are used as push buttons.
 */
void sideboardSensors(uint8_t sensors) {
#if !defined(VARIANT_HOVERBOARD) && (defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3))
    static uint8_t  sensor1_prev, sensor2_prev;
    uint8_t sensor1_rising_edge, sensor2_rising_edge;
    sensor1_rising_edge  = (sensors & SENSOR1_SET) && !sensor1_prev;
    sensor2_rising_edge  = (sensors & SENSOR2_SET) && !sensor2_prev;
    sensor1_prev         =  sensors & SENSOR1_SET;
    sensor2_prev         =  sensors & SENSOR2_SET;

    // Control MODE and Control Type Handling: use Sensor1 as push button
    static uint8_t  sensor1_index;          // holds the press index number for sensor1, when used as a button
    if (sensor1_rising_edge) {
      sensor1_index++;
      if (sensor1_index > 4) { sensor1_index = 0; }
      switch (sensor1_index) {
        case 0:     // FOC VOLTAGE
          rtP_Left.z_ctrlTypSel  = FOC_CTRL;
          rtP_Right.z_ctrlTypSel = FOC_CTRL;
          ctrlModReqRaw          = VLT_MODE;
          break;
        case 1:     // FOC SPEED
          ctrlModReqRaw          = SPD_MODE;
          break;
        case 2:     // FOC TORQUE
          ctrlModReqRaw          = TRQ_MODE;
          break;
        case 3:     // SINUSOIDAL
          rtP_Left.z_ctrlTypSel  = SIN_CTRL;
          rtP_Right.z_ctrlTypSel = SIN_CTRL;
          break;
        case 4:     // COMMUTATION
          rtP_Left.z_ctrlTypSel  = COM_CTRL;
          rtP_Right.z_ctrlTypSel = COM_CTRL;
          break;    
      }
      beepShortMany(sensor1_index + 1, 1);
    }

    // Field Weakening: use Sensor2 as push button
    #ifdef CRUISE_CONTROL_SUPPORT
      if (sensor2_rising_edge) {
        cruiseControl(sensor2_rising_edge);
      }
    #else
      static uint8_t  sensor2_index;          // holds the press index number for sensor2, when used as a button
      if (sensor2_rising_edge) {
        sensor2_index++;
        if (sensor2_index > 1) { sensor2_index = 0; }
        switch (sensor2_index) {
          case 0:     // FW Disabled
            rtP_Left.b_fieldWeakEna  = 0; 
            rtP_Right.b_fieldWeakEna = 0;
            Input_Lim_Init();
            break;
          case 1:     // FW Enabled
            rtP_Left.b_fieldWeakEna  = 1; 
            rtP_Right.b_fieldWeakEna = 1;
            Input_Lim_Init();
            break; 
        }
        beepShortMany(sensor2_index + 1, 1);            
      }
    #endif  // CRUISE_CONTROL_SUPPORT
  #endif
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
// Old filter
// Inputs:       u     = int16
// Outputs:      y     = fixdt(1,32,20)
// Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
// yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
// void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
//   int32_t tmp;
//   tmp = (int16_t)(u << 4) - (*y >> 16);
//   tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
//   *y  = coef * tmp + (*y);
// }

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
 * Outputs:      rty_speedL                            = int16_t
 * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
 */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedL) {
	int16_t prodSpeed;
	int32_t tmp;

	prodSpeed = (int16_t) ((rtu_speed * (int16_t) SPEED_COEFFICIENT) >> 14);

	tmp = prodSpeed;
	tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
	*rty_speedL = (int16_t) (tmp >> 4);       // Convert from fixed-point to int
	*rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}

/* =========================== Multiple Tap Function =========================== */

/* multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
 * This function detects multiple tap presses, such as double tapping, triple tapping, etc.
 * Inputs:       u = int16_t (input signal); timeNow = uint32_t (current time)
 * Outputs:      x->b_multipleTap (get the output here)
 */
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x) {
	uint8_t b_timeout;
	uint8_t b_hyst;
	uint8_t b_pulse;
	uint8_t z_pulseCnt;
	uint8_t z_pulseCntRst;
	uint32_t t_time;

	// Detect hysteresis
	if (x->b_hysteresis) {
		b_hyst = (u > MULTIPLE_TAP_LO);
	} else {
		b_hyst = (u > MULTIPLE_TAP_HI);
	}

	// Detect pulse
	b_pulse = (b_hyst != x->b_hysteresis);

	// Save time when first pulse is detected
	if (b_hyst && b_pulse && (x->z_pulseCntPrev == 0)) {
		t_time = timeNow;
	} else {
		t_time = x->t_timePrev;
	}

	// Create timeout boolean
	b_timeout = (timeNow - t_time > MULTIPLE_TAP_TIMEOUT);

	// Create pulse counter
	if ((!b_hyst) && (x->z_pulseCntPrev == 0)) {
		z_pulseCnt = 0U;
	} else {
		z_pulseCnt = b_pulse;
	}

	// Reset counter if we detected complete tap presses OR there is a timeout
	if ((x->z_pulseCntPrev >= MULTIPLE_TAP_NR) || b_timeout) {
		z_pulseCntRst = 0U;
	} else {
		z_pulseCntRst = x->z_pulseCntPrev;
	}
	z_pulseCnt = z_pulseCnt + z_pulseCntRst;

	// Check if complete tap presses are detected AND no timeout
	if ((z_pulseCnt >= MULTIPLE_TAP_NR) && (!b_timeout)) {
		x->b_multipleTap = !x->b_multipleTap;	// Toggle output
	}

	// Update states
	x->z_pulseCntPrev = z_pulseCnt;
	x->b_hysteresis = b_hyst;
	x->t_timePrev = t_time;
}

