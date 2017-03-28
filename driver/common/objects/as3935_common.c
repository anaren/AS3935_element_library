/**
 *	----------------------------------------------------------------------------
 *	Copyright (c) 2016, Anaren Microwave, Inc.
 *
 *	For more information on licensing, please see Anaren Microwave, Inc's
 *	end user software licensing agreement: EULA.txt.
 *
 *	----------------------------------------------------------------------------
 *
 *	as3935_common.c - driver interface for the ams AG AS3935 Frankling Lightning
 *	Sensor.
 *
 *	@version		1.0.0
 *	@date			 19 Sep 2016
 *	@author		 Anaren, air@anaren.com
 *
 *	assumptions
 *	===========
 *	- The i2c driver provides the proper signaling sequences for read & write
 *		operations.
 *	- The i2c driver meets the timing requirements specified in the AS3935
 *		datasheet.
 *
 *	file dependency
 *	===============
 *	i2c.h : defines the i2c read & write interfaces.
 *	math.h : floating point calculations.
 *	fp_math.h : floating point calculations using fixed point math.	Use when math.h not implemented.
 *
 *	revision history
 *	================
 *	ver 1.0.00 : 19 September 2016
 *	- initial development, replace this line upon release.
 */
#include "as3935.h"
#include "../i2c/i2c.h"
#include "../gpio/gpio.h"
#include "../generic/generic.h"
#include "../pin_mapping.h"

// -----------------------------------------------------------------------------
/**
 *	Global data
 */
static int AS3935_freq = 0;
static int AS3935_mode = 0;


// -----------------------------------------------------------------------------
/**
 *	Private interface
 */



// -----------------------------------------------------------------------------
/**
 *	Public interface
 */

void AS3935_Init()
{
	AS3935_PresetRegisterDefaults();
	AS3935_CalibrateRCO();
	AS3935_SetAnalogFrontEnd(AIR_AS3935_ANALOG_FRONT_END);
	AS3935_SetNoiseFloor((0x02<<4)|(0x02<<0));
	AS3935_EnableDisturbers();
	AS3935_CalibrateLCO();
}

/**
 * Write to specified register on the AS3935.
 */
void AS3935_WriteReg(uint8_t addr, uint8_t data)
{
	uint8_t writeBytes[2];
	writeBytes[0] = addr;
	writeBytes[1] = data;
	AIR_I2C_Write(AS3935_I2C_ADDRESS, writeBytes, 2);
}

/**
 * Read data from specified register
 */
uint8_t AS3935_ReadReg(uint8_t addr)
{
	uint8_t writeBytes[1] = {0};
	uint8_t readBytes[1] = {0};
	AIR_I2C_ComboRead(AS3935_I2C_ADDRESS, writeBytes, 1, readBytes, 1);
	return readBytes[0];
}

/**
 * Get the estimated distance to the lightning storm in kilometers.
 * Also resets the contents of the Interrupt register.
 * Values beyond 40 are out of range.
 * Values less than 5 mean that the storm is overhead.
 */
uint8_t AS3935_GetDistanceEstimation()
{
	uint8_t distanceEstimate = (uint8_t)AS3935_ReadReg(AS3935_DIST_ESTI_REG_ADDR);

	return distanceEstimate & AS3935_DISTEST_MASK;
}

/**
 * Calibrate the RC Oscillators automatically.
 */
void AS3935_CalibrateRCO()
{
	AS3935_WriteReg(AS3935_CALIBR_RCO_REG_ADDR, AS3935_DIRECT_CMD_REG_VALU);
}

void AS3935_CalibrateLCOIRQHandler()
{
	if (AS3935_mode == 1) {
		AS3935_freq++;
	}
}

/**
 *
 */
void AS3935_CalibrateLCO()
{
	int i, n, m = 10000, r = 0;
	
	AIR_GPIO_RegisterInterrupt(AS3935_IRQ_PIN, AS3935_CalibrateLCOIRQHandler, AIR_GPIO_TRIGGER_FALLING_EDGE);
	for(i = 0; i < 0x10; i++)
	{
		AS3935_WriteReg(AS3935_DISP_IRQ_REG_ADDR, 0x80 | i);
		AIR_GENERIC_UDelay(10000);
		AS3935_freq = 0;
		AS3935_mode = 1;
		AIR_GENERIC_UDelay(100000);
		AS3935_mode = 0;
		n = abs(AS3935_freq - 3125);
		if (m > n) {
			r = i;
		} else {
			break;
		}
		m = n;
	}
	AIR_GPIO_UnregisterInterrupt(AS3935_IRQ_PIN);
	
	AS3935_WriteReg(AS3935_DISP_IRQ_REG_ADDR, r);
}

/**
 * Sets all registers in default mode
 */
void AS3935_PresetRegisterDefaults()
{
	AS3935_WriteReg(AS3935_PRESET_DEF_REG_ADDR, AS3935_DIRECT_CMD_REG_VALU);
}

/*
 * Set the analog front end and watchdog operating mode.
 * Refer to as3935.h for possible operating modes.
 */
void AS3935_SetAnalogFrontEnd(uint8_t mode)
{
	uint8_t newAFESetting;
	uint8_t currentAFESetting = AS3935_ReadReg(AS3935_PWD_AFEGB_REG_ADDR);
	currentAFESetting = currentAFESetting & AS3935_AFE_MASK;
	if (mode == AS3935_AFE_OUTDOOR)
	{
	newAFESetting = currentAFESetting & AS3935_AFE_OUTDOOR;
		AS3935_WriteReg(AS3935_PWD_AFEGB_REG_ADDR, newAFESetting);
	}

	if (mode == AS3935_AFE_INDOOR)
	{
	newAFESetting = currentAFESetting & AS3935_AFE_OUTDOOR;
		AS3935_WriteReg(AS3935_PWD_AFEGB_REG_ADDR, newAFESetting);
	}
}

/**
 * Retrieve the current operating mode of the AFE and watchdog.
 */
uint8_t AS3935_GetAnalogFrontEnd()
{
	return ((uint8_t)AS3935_ReadReg(AS3935_PWD_AFEGB_REG_ADDR) & AS3935_AFE_MASK_2) >> 1;
}

/**
 * Disable the disturber detection feature on the AS3935.
 */
void AS3935_DisableDisturbers()
{
	AS3935_WriteReg(AS3935_MASK_DIST_REG_ADDR, 1);
}

/**
 * Enable the disturber detection feature on the AS3935.
 */
void AS3935_EnableDisturbers()
{
	AS3935_WriteReg(AS3935_MASK_DIST_REG_ADDR, 0);
}

/**
 * Get the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 */
uint8_t AS3935_GetMinimumLightnings()
{
	return AS3935_ReadReg(AS3935_MIN_LIGHT_REG_ADDR);
}

/**
 * Set the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 */
uint8_t AS3935_SetMinimumLightnings(uint8_t minimumLightning)
{
	AS3935_WriteReg(AS3935_MIN_LIGHT_REG_ADDR, minimumLightning);
	return AS3935_GetMinimumLightnings();
}

/**
 * Get the defined threshold for the noise floor that triggers an interrupt.
 */
uint8_t AS3935_GetNoiseFloor()
{
	return AS3935_ReadReg(AS3935_NFLV_WDTH_REG_ADDR);
}

/**
 * Set the defined threshold for the noise floor that triggers an interrupt.
 */
uint8_t AS3935_SetNoiseFloor(int noiseFloor)
{
	AS3935_WriteReg(AS3935_NFLV_WDTH_REG_ADDR, noiseFloor);
	return AS3935_GetNoiseFloor();
}

/**
 * Read current state of the interrupt register.
 * Issue a 2 millisecond delay per the datasheet recommendations.
 */
uint8_t AS3935_ReadInterruptRegister()
{
	return (uint8_t)AS3935_ReadReg(AS3935_MASK_DIST_REG_ADDR);
}


