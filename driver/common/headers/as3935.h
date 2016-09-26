// Copyright (c) 2016, Anaren Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
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

#include "../i2c/i2c.h"
#include "as3935_config.h"
#include <stdint.h>

//------------------------------------------------------------------------------
/**
 *  Defines, enumerations, and structure definitions
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
 *  Defines, enumerations, and structure definitions
 */

//The MOD-1016 Board uses I2C Address 0x03 since both SD0 and SD1 pins are pulled high.
#define AS3935_I2C_BASE_ADDR      	  0x03

#define AS3935_PWD_AFEGB_REG_ADDR     0x00
#define AS3935_NFLV_WDTH_REG_ADDR     0x01
#define AS3935_MIN_LIGHT_REG_ADDR     0x02
#define AS3935_MASK_DIST_REG_ADDR     0x03
#define AS3935_LIGHT_LSB_REG_ADDR     0x04
#define AS3935_LIGHT_MSB_REG_ADDR     0x05
#define AS3935_LIGHT_MMS_REG_ADDR     0x06
#define AS3935_DIST_ESTI_REG_ADDR     0x07
#define AS3935_DISP_IRQ_REG_ADDR      0x08

//Lightning Detection Lookup Table
#define AS3935_LDLUT1_REG_ADDR        0x09
#define AS3935_LDLUT2_REG_ADDR        0x0A
#define AS3935_LDLUT3_REG_ADDR        0x0B
#define AS3935_LDLUT4_REG_ADDR		  0x0C
#define AS3935_LDLUT5_REG_ADDR        0x0D
#define AS3935_LDLUT6_REG_ADDR	      0x0E
#define AS3935_LDLUT7_REG_ADDR		  0x0F
#define AS3935_LDLUT8_REG_ADDR		  0x10
#define AS3935_LDLUT9_REG_ADDR		  0x11
#define AS3935_LDLUT10_REG_ADDR		  0x12
#define AS3935_LDLUT11_REG_ADDR		  0x13
#define AS3935_LDLUT12_REG_ADDR		  0x14
#define AS3935_LDLUT13_REG_ADDR		  0X15
#define AS3935_LDLUT14_REG_ADDR		  0X16
#define AS3935_LDLUT15_REG_ADDR		  0X17
#define AS3935_LDLUT16_REG_ADDR		  0X18
#define AS3935_LDLUT17_REG_ADDR		  0X19
#define AS3935_LDLUT18_REG_ADDR		  0X1A
#define AS3935_LDLUT19_REG_ADDR		  0X1B
#define AS3935_LDLUT20_REG_ADDR		  0X1C
#define AS3935_LDLUT21_REG_ADDR		  0X1D
#define AS3935_LDLUT22_REG_ADDR		  0X1E
#define AS3935_LDLUT23_REG_ADDR		  0X1F
#define AS3935_LDLUT24_REG_ADDR		  0X20
#define AS3935_LDLUT25_REG_ADDR		  0X21
#define AS3935_LDLUT26_REG_ADDR		  0X22
#define AS3935_LDLUT27_REG_ADDR		  0X23
#define AS3935_LDLUT28_REG_ADDR		  0X24
#define AS3935_LDLUT29_REG_ADDR		  0X25
#define AS3935_LDLUT30_REG_ADDR		  0X26
#define AS3935_LDLUT31_REG_ADDR		  0X27
#define AS3935_LDLUT32_REG_ADDR		  0X28
#define AS3935_LDLUT33_REG_ADDR		  0X29
#define AS3935_LDLUT34_REG_ADDR		  0X2A
#define AS3935_LDLUT35_REG_ADDR		  0X2B
#define AS3935_LDLUT36_REG_ADDR		  0X2C
#define AS3935_LDLUT37_REG_ADDR		  0X2D
#define AS3935_LDLUT38_REG_ADDR		  0X2E
#define AS3935_LDLUT39_REG_ADDR		  0X2F
#define AS3935_LDLUT40_REG_ADDR		  0X30
#define AS3935_LDLUT41_REG_ADDR		  0X31
#define AS3935_LDLUT42_REG_ADDR		  0X32

/**
 * It is possible to send direct commands writing 0x96 to the registers
 * REG0x3C and REG0x3D.
 */
#define AS3935_PRESET_DEF_REG_ADDR    0x3C
#define AS3935_CALIBR_RCO_REG_ADDR    0x3D
#define AS3935_DIRECT_CMD_REG_VALU    0x96


/**
 * MASKS and other values.
 *
 *
 */
#define AFE_MASK	0b11000001
#define AFE_OUTDOOR	0b00100100
#define AFE_INDOOR	0b00011100

/**
 *  eAS3935Mode - type indicating the operating mode of the AS3935 device.  The
 *  AS3935 offers three modes: Power Down, Listening, and Signal Verification.  When
 *  ultra-low power consumption is important, the application should place the
 *  device in Power Down Mode when lightning measurements are not needed.
 *
 *  In Listening Mode the Analog Front End (AFE), noise floor level generation,
 *  the bias block, the TRCO, and the voltage regulator (if enabled) are running.
 *	In this mode the system can push down the power consumption to a minimum,
 *	(60µA). In case the maximum voltage supply does not exceed 3.6V,
 *	it is possible to switch off the voltage regulator to save power.
 *
 *  The AS3935 enters Signal Verification Mode when the watchdog detects activity
 *  picked up by the antenna. The IC will leave this mode either if the incoming signal
 *  is classified as a disturber or if the analysis of the single event (lightning)
 *  is finished. If the received signal is classified as a disturber, then the AS3935
 *  will automatically go back to listening mode without any needed action from outside
 *  and an interrupt will be generated. If the received pattern matches all requirements,
 *  the energy calculation is performed and the AS3935 provides distance estimation.
 *
 */
enum eAS3935Mode
{
  AS3935_PowerDown             = 0x0000,
  AS3935_Listening			   = 0x0001,
  AS3935_SignalVerification    = 0x7000
};

/**
Write a 16-bit value to a device register.  This function does not do any
form of error checking, so trying to write to one of the read-only registers may
result in undesireable behavior.
@param id device ID (0 to 7) on i2c bus
@param addr device register address
@param data data to be written to the specified register address
*/
void AS3935_WriteReg(uint8_t id, uint8_t addr, uint8_t data);
//MAY HAVE TO CHANGE BACK DATA TO UINT16_T
/**
Read a 16-bit value from a device register.
@param id device ID (0 to 7) on i2c bus
@param addr device register address
@return data read from the specified register address
*/
uint8_t AS3935_ReadReg(uint8_t id, uint8_t addr);

/**
Issue a software reset to the sensor.
@param id device ID (0 to 7) on i2c bus
@note This is a self-clearing operation.  There is no need for software to clear
the reset condition.
*/
void AS3935_SoftwareReset(uint8_t id);

/**
Select the device operating mode.  Refer to eAS3935Mode definition for details
regarding the allowed states.
@param id device ID (0 to 7) on i2c bus
@param mode specifies the device mode of operation
*/
void AS3935_SetOperatingMode(uint8_t id, enum eAS3935Mode mode);

/**
Read the currently selected operating mode.  Refer to eAS3935Mode definition for
details regarding the available states.
@param id device ID (0 to 7) on i2c bus
@return device mode of operation
*/
enum eAS3935Mode AS3935_GetOperatingMode(uint8_t id);

#endif
