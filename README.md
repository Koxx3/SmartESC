# [ SmartESC ]

⚠️ **Warning : this is work in progress. I decline all responsability about using informations from this project** ⚠️

![Logo](/img/smart_esc.png)

This project is desgiend to replace the firmware inside Xiaomi M365 controller.

It is based on the hoverboard firmware with commutation, sine and FOC algorithms.
FOC is the most efficient algorithm and this is what we plan to use.

The Xiaomi controller is well designed and can already accept up 52V (2.1 & 3.0 rev) without modification.
With some modifications, it can accept up to 80V. And heavy modifications, 100V.

The controller is based on STM32F103C6T8 micro-controller.

Xiaomi M365 Controller details :
- The M365 controller exist to 3 versions and 1.4 & 2.1 have clone version more sheap
some copy are very good, and some are bad 
the bad copy have poor mosfets, poor aluminum heatskin but the pcb is OK 
good copy exist only on 1.4 Rev have all good but probably not exist today
  - Rev 1.4 have 60v regulator (maximum absolute) can be use to 12S battery pack maximum safety 
  - Rev 2.1 have 90v regulator (maximum absolute, 84V real) can be use to 19S battery pack maximum safety 
  - Rev 3.0 is exactly the same with 2.1 but design is more stronger for current than 2.1 & 1.4
- stock Voltage sending limited to 54v by divided bridge resistor, but 150v tolérant 
- stock Current reading to ~ +/- 100A per negative phase*
- stock Capacitor is 1000uf 63v in stock can be change for 100V
- NEWS we find voltage régulator 100v for remplace the 90v régulator on the board
  More détails here :
  https://docs.google.com/spreadsheets/d/1jOdUUpbCZVHQc2-eIezrbkNFHuf14I3zOeL0la91yoo/edit?usp=sharing


## Requirements
- Xiaomi controller
- [SmartDisplay](https://github.com/Koxx3/SmartController_SmartDisplay) or arduino
- Motor with hall sensors
- STLink (for flashing)
- Power (battery or power supply)

## Current status

- [X] Build hover boardfirmware
- [X] Flashing with STLInk
- [X] Modify pinout for Xiaomi controller
- [X] Spin the motor 
  - [X] in commutation mode
  - [X] in sine mode
  - [X] in FOC mode (speed, voltage & FOC)
- [X] Controller the motor with serial link (display / SmartController)
  - [X] Create a new serial link with all data
  - [X] Full-duplex
  - [ ] Half-duplex
- [X] Modes for speed limits
  - [X] configurable speed limits
- [X] Control from the [SmartDisplay](https://github.com/Koxx3/SmartController_SmartDisplay)
- [X] Process soft electric braking
  - [ ] configurable electric braking force
- [ ] Link multiple controller
- [ ] Optimize  
- [ ] Process wheel lock
- [ ] Process soft throttle release
- [ ] Communicate with BMS
  - [ ] Half-duplex
  - [ ] Full-duplex


## Controller schematics 

[Here](docs/Driver_V2.1_M365.pdf)

## Serial link

[Here](https://docs.google.com/spreadsheets/d/1SmxQHb-3iMCkWmL3f3GHKCjGlgmRvZwS34Iuaf2yz5I/edit?usp=sharing)

## Remote control from Arduino

Here is a test program :
[Here](https://github.com/Koxx3/SmartESC_ESP32_serial_control)

With this, you can remote control the SmartESC with an ESP32 :
- analog PIN 39 : analog throttle with hall sensor trigger (0.8V -> 4.1V)
- analog PIN 34 : analog brake with hall sensor trigger (0.8V -> 4.1V)
- IO PIN 27 : SERIAL_ESP_TO_CNTRL
- IO PIN 14 : SERIAL_CNTRL_TO_ESP       
- Monitor serial : 921600 bauds
- ESC serial : 115200 bauds


## Remote control from Chrome

Use a FTDI adapter and launch the webconsole in Chrome :
https://koxx3.fr.eu.org:8086/SmartESC_WebControl/


## Flashing

Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

If you have never flashed your controller before, the MCU is probably locked. To unlock the flash, check-out the wiki page [How to Unlock MCU flash](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-flash).

Do not power the mainboard from the 3.3V of your programmer! This has already killed multiple mainboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the STM might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

To build and flash, you need to use STM32CubeIDE.


---
---

# Original hoverboard-firmware-hack-FOC informations

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository implements Field Oriented Control (FOC) for stock hoverboards. Compared to the commutation method, this new FOC control method offers superior performance featuring:
 - reduced noise and vibrations 	
 - smooth torque output and improved motor efficiency. Thus, lower energy consumption
 - field weakening to increase maximum speed range


Table of Contents
=======================

* [Hardware](#hardware)
* [FOC Firmware](#foc-firmware)
* [Example Variants ](#example-variants)
* [Flashing](#flashing)
* [Troubleshooting](#troubleshooting)
* [Diagnostics](#diagnostics)
* [Projects and Links](#projects-and-links)
* [Contributions](#contributions)

#### For the hoverboard sideboard firmware, see the following repositories:
 - [hoverboard-sideboard-hack-GD](https://github.com/EmanuelFeru/hoverboard-sideboard-hack-GD)
 - [hoverboard-sideboard-hack-STM](https://github.com/EmanuelFeru/hoverboard-sideboard-hack-STM)
 
#### For the FOC controller design, see the following repository:
 - [bldc-motor-control-FOC](https://github.com/EmanuelFeru/bldc-motor-control-FOC)

#### Videos:
<table>
  <tr>
    <td><a href="https://youtu.be/IgHCcj0NgWQ" title="Hovercar" rel="noopener"><img src="/docs/pictures/videos_preview/hovercar_intro.png"></a></td>
    <td><a href="https://youtu.be/gtyqtc37r10" title="Cruise Control functionality" rel="noopener"><img src="/docs/pictures/videos_preview/cruise_control.png"></a></td>
    <td><a href="https://youtu.be/jadD0M1VBoc" title="Hovercar pedal functionality" rel="noopener"><img src="/docs/pictures/videos_preview/hovercar_pedals.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/UnlbMrCkjnE" title="Commutation vs. FOC (constant speed)" rel="noopener"><img src="/docs/pictures/videos_preview/com_foc_const.png"></a></td> 
    <td><a href="https://youtu.be/V-_L2w10wZk" title="Commutation vs. FOC (variable speed)" rel="noopener"><img src="/docs/pictures/videos_preview/com_foc_var.png"></a></td>       
    <td><a href="https://youtu.be/tVj_lpsRirA" title="Reliable Serial Communication" rel="noopener"><img src="/docs/pictures/videos_preview/serial_com.png"></a></td>
  </tr>
</table>


---
## Hardware
 
![mainboard_pinout](/docs/pictures/mainboard_pinout.png)

The original Hardware supports two 4-pin cables that originally were connected to the two sideboards. They break out GND, 12/15V and USART2&3 of the Hoverboard mainboard. Both USART2&3 support UART, PWM, PPM, and iBUS input. Additionally, the USART2 can be used as 12bit ADC, while USART3 can be used for I2C. Note that while USART3 (right sideboard cable) is 5V tolerant, USART2 (left sideboard cable) is **not** 5V tolerant.

Typically, the mainboard brain is an [STM32F103RCT6](/docs/literature/[10]_STM32F103xC_datasheet.pdf), however some mainboards feature a [GD32F103RCT6](/docs/literature/[11]_GD32F103xx-Datasheet-Rev-2.7.pdf) which is also supported by this firmware.

For the reverse-engineered schematics of the mainboard, see [20150722_hoverboard_sch.pdf](/docs/20150722_hoverboard_sch.pdf)

 
---
## FOC Firmware
 
In this firmware 3 control types are available:
- Commutation
- SIN (Sinusoidal)
- FOC (Field Oriented Control) with the following 3 control modes:
  - **VOLTAGE MODE**: in this mode the controller applies a constant Voltage to the motors. Recommended for robotics applications or applications where a fast motor response is required.
  - **SPEED MODE**: in this mode a closed-loop controller realizes the input speed target by rejecting any of the disturbance (resistive load) applied to the motor. Recommended for robotics applications or constant speed applications.
  - **TORQUE MODE**: in this mode the input torque target is realized. This mode enables motor "freewheeling" when the torque target is `0`. Recommended for most applications with a sitting human driver.
  
#### Comparison between different control methods

|Control method| Complexity | Efficiency | Smoothness | Field Weakening | Freewheeling | Standstill hold |
|--|--|--|--|--|--|--|
|Commutation| - | - | ++ | n.a. | n.a. | + |
|Sinusoidal| + | ++ | ++ | +++ | n.a. | + |
|FOC VOLTAGE| ++ | +++ | ++ | ++ | n.a. | +<sup>(2)</sup> |
|FOC SPEED| +++ | +++ | + | ++ | n.a. | +++ |
|FOC TORQUE| +++ | +++ | +++ | ++ | +++<sup>(1)</sup> | n.a<sup>(2)</sup> |

<sup>(1)</sup> By enabling `ELECTRIC_BRAKE_ENABLE` in `config.h`, the freewheeling amount can be adjusted using the `ELECTRIC_BRAKE_MAX` parameter.

<sup>(2)</sup> The standstill hold functionality can be forced by enabling `STANDSTILL_HOLD_ENABLE` in `config.h`. 


In all FOC control modes, the controller features maximum motor speed and maximum motor current protection. This brings great advantages to fulfil the needs of many robotic applications while maintaining safe operation.


### Field Weakening / Phase Advance

 - By default the Field weakening is disabled. You can enable it in config.h file by setting the FIELD_WEAK_ENA = 1 
 - The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 - The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 - The figure below shows different possible calibrations for Field Weakening / Phase Advance
 ![Field Weakening](/docs/pictures/FieldWeakening.png) 
 - If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!


### Parameters 
 - All the calibratable motor parameters can be found in the 'BLDC_controller_data.c'. I provided you with an already calibrated controller, but if you feel like fine tuning it feel free to do so 
 - The parameters are represented in Fixed-point data type for a more efficient code execution
 - For calibrating the fixed-point parameters use the [Fixed-Point Viewer](https://github.com/EmanuelFeru/FixedPointViewer) tool
 - The controller parameters are given in [this table](https://github.com/EmanuelFeru/bldc-motor-control-FOC/blob/master/02_Figures/paramTable.png)


---
## Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuk not working: Use the right one of the 2 types of nunchuks. Use i2c pullups.

Nunchuk or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabilize voltage in nunchuk or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Recommendation: Nunchuk Breakout Board https://github.com/Jan--Henrik/hoverboard-breakout

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.


---
## Diagnostics

For a detailed troubleshooting connect an [FTDI Serial adapter](https://s.click.aliexpress.com/e/_AqPOBr) or a [Bluetooth module](https://s.click.aliexpress.com/e/_A4gkMD) to the DEBUG_SERIAL cable (Left or Right) and monitor the output data using the [Hoverboard Web Serial Control](https://candas1.github.io/Hoverboard-Web-Serial-Control/) tool developed by [Candas](https://github.com/Candas1/).

---
## Projects and Links

- **Original firmware:** [https://github.com/NiklasFauth/hoverboard-firmware-hack](https://github.com/NiklasFauth/hoverboard-firmware-hack)
- **[Candas](https://github.com/Candas1/) Hoverboard Web Serial Control:** [https://candas1.github.io/Hoverboard-Web-Serial-Control/](https://candas1.github.io/Hoverboard-Web-Serial-Control/)
- **[RoboDurden's](https://github.com/RoboDurden) online compiler:** [https://pionierland.de/hoverhack/](https://pionierland.de/hoverhack/) 
- **Hoverboard hack for AT32F403RCT6 mainboards:** [https://github.com/cloidnerux/hoverboard-firmware-hack](https://github.com/cloidnerux/hoverboard-firmware-hack)
- **Hoverboard hack for split mainboards:** [https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2](https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2)
- **Hoverboard hack from BiPropellant:** [https://github.com/bipropellant](https://github.com/bipropellant)
- **Hoverboard breakout boards:** [https://github.com/Jan--Henrik/hoverboard-breakout](https://github.com/Jan--Henrik/hoverboard-breakout)

<a/>

- **Bobbycar** [https://github.com/larsmm/hoverboard-firmware-hack-bbcar](https://github.com/larsmm/hoverboard-firmware-hack-bbcar)
- **Wheel chair:** [https://github.com/Lahorde/steer_speed_ctrl](https://github.com/Lahorde/steer_speed_ctrl)
- **TranspOtterNG:** [https://github.com/Jan--Henrik/transpOtterNG](https://github.com/Jan--Henrik/transpOtterNG)
- **ST Community:** [Custom FOC motor control](https://community.st.com/s/question/0D50X0000B28qTDSQY/custom-foc-control-current-measurement-dma-timer-interrupt-needs-review)

<a/>

- **Telegram Community:** If you are an enthusiast join our [Hooover Telegram Group](https://t.me/joinchat/BHWO_RKu2LT5ZxEkvUB8uw)


---
## Contributions

Every contribution to this repository is highly appreciated! Feel free to create pull requests to improve this firmware as ultimately you are going to help everyone. 

If you want to donate to keep this firmware updated, please use the link below:

[![paypal](https://www.paypalobjects.com/en_US/NL/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=CU2SWN2XV9SCY&currency_code=EUR&source=url)


---

