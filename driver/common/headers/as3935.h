// Copyright (c) 2016, Anaren Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//		list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//		this list of conditions and the following disclaimer in the documentation
//		and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
// either expressed or implied, of the FreeBSD Project.

#ifndef AS3935_H
#define AS3935_H

#include "as3935_config.h"
#include <stdint.h>

//------------------------------------------------------------------------------
/**
 *	Defines, enumerations, and structure definitions
 */
#ifndef bool
#define bool uint8_t
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

//------------------------------------------------------------------------------
/**
 *	Defines, enumerations, and structure definitions
 */

//The MOD-1016 Board uses I2C Address 0x03 since both SD0 and SD1 pins are pulled high.
#define AS3935_I2C_BASE_ADDR					0x03

#define AS3935_PWD_AFEGB_REG_ADDR		 0x00
#define AS3935_NFLV_WDTH_REG_ADDR		 0x01
#define AS3935_MIN_LIGHT_REG_ADDR		 0x02
#define AS3935_MASK_DIST_REG_ADDR		 0x03
#define AS3935_LIGHT_LSB_REG_ADDR		 0x04
#define AS3935_LIGHT_MSB_REG_ADDR		 0x05
#define AS3935_LIGHT_MMS_REG_ADDR		 0x06
#define AS3935_DIST_ESTI_REG_ADDR		 0x07
#define AS3935_DISP_IRQ_REG_ADDR			0x08

//Lightning Detection Lookup Table
#define AS3935_LDLUT1_REG_ADDR				0x09
#define AS3935_LDLUT2_REG_ADDR				0x0A
#define AS3935_LDLUT3_REG_ADDR				0x0B
#define AS3935_LDLUT4_REG_ADDR			0x0C
#define AS3935_LDLUT5_REG_ADDR				0x0D
#define AS3935_LDLUT6_REG_ADDR				0x0E
#define AS3935_LDLUT7_REG_ADDR			0x0F
#define AS3935_LDLUT8_REG_ADDR			0x10
#define AS3935_LDLUT9_REG_ADDR			0x11
#define AS3935_LDLUT10_REG_ADDR			0x12
#define AS3935_LDLUT11_REG_ADDR			0x13
#define AS3935_LDLUT12_REG_ADDR			0x14
#define AS3935_LDLUT13_REG_ADDR			0X15
#define AS3935_LDLUT14_REG_ADDR			0X16
#define AS3935_LDLUT15_REG_ADDR			0X17
#define AS3935_LDLUT16_REG_ADDR			0X18
#define AS3935_LDLUT17_REG_ADDR			0X19
#define AS3935_LDLUT18_REG_ADDR			0X1A
#define AS3935_LDLUT19_REG_ADDR			0X1B
#define AS3935_LDLUT20_REG_ADDR			0X1C
#define AS3935_LDLUT21_REG_ADDR			0X1D
#define AS3935_LDLUT22_REG_ADDR			0X1E
#define AS3935_LDLUT23_REG_ADDR			0X1F
#define AS3935_LDLUT24_REG_ADDR			0X20
#define AS3935_LDLUT25_REG_ADDR			0X21
#define AS3935_LDLUT26_REG_ADDR			0X22
#define AS3935_LDLUT27_REG_ADDR			0X23
#define AS3935_LDLUT28_REG_ADDR			0X24
#define AS3935_LDLUT29_REG_ADDR			0X25
#define AS3935_LDLUT30_REG_ADDR			0X26
#define AS3935_LDLUT31_REG_ADDR			0X27
#define AS3935_LDLUT32_REG_ADDR			0X28
#define AS3935_LDLUT33_REG_ADDR			0X29
#define AS3935_LDLUT34_REG_ADDR			0X2A
#define AS3935_LDLUT35_REG_ADDR			0X2B
#define AS3935_LDLUT36_REG_ADDR			0X2C
#define AS3935_LDLUT37_REG_ADDR			0X2D
#define AS3935_LDLUT38_REG_ADDR			0X2E
#define AS3935_LDLUT39_REG_ADDR			0X2F
#define AS3935_LDLUT40_REG_ADDR			0X30
#define AS3935_LDLUT41_REG_ADDR			0X31
#define AS3935_LDLUT42_REG_ADDR			0X32

/**
 * It is possible to send direct commands writing 0x96 to the registers
 * REG0x3C and REG0x3D.
 */
#define AS3935_PRESET_DEF_REG_ADDR		0x3C
#define AS3935_CALIBR_RCO_REG_ADDR		0x3D
#define AS3935_DIRECT_CMD_REG_VALU		0x96


/**
 * MASKS and other values. Analog Front End operating modes are defined here.
 *
 */
#define AS3935_AFE_MASK	 0b11000001
#define AS3935_AFE_MASK_2	 0x3E
#define AS3935_AFE_OUTDOOR	 0b00100100
#define AS3935_AFE_INDOOR	 0b00011100
#define AS3935_DISTEST_MASK 0x3F
#define AS3935_INTRPT_MASK	0b11110000
#define AS3935_INTRPT_RESET 0b0000


/**
 *	eAS3935Mode - type indicating the operating mode of the AS3935 device.	The
 *	AS3935 offers three modes: Power Down, Listening, and Signal Verification.	When
 *	ultra-low power consumption is important, the application should place the
 *	device in Power Down Mode when lightning measurements are not needed.
 *
 *	In Listening Mode the Analog Front End (AFE), noise floor level generation,
 *	the bias block, the TRCO, and the voltage regulator (if enabled) are running.
 *	In this mode the system can push down the power consumption to a minimum,
 *	(60�A). In case the maximum voltage supply does not exceed 3.6V,
 *	it is possible to switch off the voltage regulator to save power.
 *
 *	The AS3935 enters Signal Verification Mode when the watchdog detects activity
 *	picked up by the antenna. The IC will leave this mode either if the incoming signal
 *	is classified as a disturber or if the analysis of the single event (lightning)
 *	is finished. If the received signal is classified as a disturber, then the AS3935
 *	will automatically go back to listening mode without any needed action from outside
 *	and an interrupt will be generated. If the received pattern matches all requirements,
 *	the energy calculation is performed and the AS3935 provides distance estimation.
 *
 */
enum eAS3935Mode
{
	AS3935_PowerDown						 = 0x0000,
	AS3935_Listening				 = 0x0001,
	AS3935_SignalVerification		= 0x7000
};

/**
Initilizes the sensor.
*/
void AS3935_Init();

/**
Write a 8-bit value to a device register.	This function does not do any
form of error checking, so trying to write to one of the read-only registers may
result in undesirable behavior.
@param addr device register address
@param data data to be written to the specified register address
*/
void AS3935_WriteReg(uint8_t addr, uint8_t data);
//MAY HAVE TO CHANGE BACK DATA TO UINT16_T
/**
Read a 8-bit value from a device register.
@param addr device register address
@return data read from the specified register address
*/
uint8_t AS3935_ReadReg(uint8_t addr);

/**
 * Get the estimated distance to the lightning storm in kilometers.
 * Values beyond 40 are out of range.
 * Values less than 5 mean that the storm is overhead
*/
uint8_t AS3935_GetDistanceEstimation();

/**
 * Calibrate the RC Oscillators automatically.
 */
void AS3935_CalibrateRCO();

void AS3935_CalibrateLCO();

/**
 * Sets all registers to default mode.
 */
void AS3935_PresetRegisterDefaults();

/*
 * Set the analog front end and watchdog operating mode.
 * Refer above for possible operating modes.
 * @param id device ID (0 to 7) on i2c bus
 * @param mode Operating Mode for AFE, can be AFE_INDOOR or AFE_OUTDOOR.
 */
void AS3935_SetAnalogFrontEnd(uint8_t mode);

/**
 * Retrieve the current operating mode of the AFE and watchdog.
 */
uint8_t AS3935_GetAnalogFrontEnd();

/**
 * Disable the disturber detection feature on the AS3935.
 */
void AS3935_DisableDisturbers();

/**
 * Enable the disturber detection feature on the AS3935.
 */
void AS3935_EnableDisturbers();

/**
 * Get the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 */
uint8_t AS3935_GetMinimumLightnings();

/**
 * Set the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 * @param minimumLightning (1 to 16)
 */
uint8_t AS3935_SetMinimumLightnings(uint8_t minimumLightning);

/**
 * Get the defined threshold for the noise floor that triggers an interrupt.
 */
uint8_t AS3935_GetNoiseFloor();

/**
 * Set the defined threshold for the noise floor that triggers an interrupt.
 * @param noiseFloor (0b000 to 0b111)
 */
uint8_t AS3935_SetNoiseFloor(int noiseFloor);

/**
 * Read the current state of the interrupt register.
 * Issue a 2 millisecond delay per the datasheet recommendations.
 */
uint8_t AS3935_ReadInterruptRegister();

/**
* Reset the interrupt register.
*/
void AS3935_ResetInterruptRegister();


#endif
