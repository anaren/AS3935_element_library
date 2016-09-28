/**
 *  ----------------------------------------------------------------------------
 *  Copyright (c) 2016, Anaren Microwave, Inc.
 *
 *  For more information on licensing, please see Anaren Microwave, Inc's
 *  end user software licensing agreement: EULA.txt.
 *
 *  ----------------------------------------------------------------------------
 *
 *  as3935_common.c - driver interface for the ams AG AS3935 Frankling Lightning
 *  Sensor.
 *
 *  @version    1.0.0
 *  @date       19 Sep 2016
 *  @author     Anaren, air@anaren.com
 *
 *  assumptions
 *  ===========
 *  - The i2c driver provides the proper signaling sequences for read & write
 *    operations.
 *  - The i2c driver meets the timing requirements specified in the AS3935
 *    datasheet.
 *
 *  file dependency
 *  ===============
 *  i2c.h : defines the i2c read & write interfaces.
 *	math.h : floating point calculations.
 *	fp_math.h : floating point calculations using fixed point math.  Use when math.h not implemented.
 *
 *  revision history
 *  ================
 *  ver 1.0.00 : 19 September 2016
 *  - initial development, replace this line upon release.
 */
#include "as3935.h"

#ifdef AIR_FLOATING_POINT_AVAILABLE
#include <math.h>
#else
#include "../fp_math/fp_math.h"
#endif

// -----------------------------------------------------------------------------
/**
 *  Global data
 */
// -----------------------------------------------------------------------------
/**
 *  Private interface
 */



// -----------------------------------------------------------------------------
/**
 *  Public interface
 */

/**
 * Write to specified register on the AS3935.
 */
void AS3935_WriteReg(uint8_t id, uint8_t addr, uint8_t data)
{
  uint8_t writeBytes[2];
  writeBytes[0] = addr;
  writeBytes[1] = data;
  AIR_I2C_WRITE(AS3935_I2C_BASE_ADDR + id, writeBytes, 2);
}

/**
 * Read data from specified register
 */
uint8_t AS3935_ReadReg(uint8_t id, uint8_t addr)
{
  uint8_t writeBytes[1] = {0};
  uint8_t readBytes[1] = {0};
  AIR_I2C_ComboRead(AS3935_I2C_BASE_ADDR + id, writeBytes, 1, readBytes, 1);
  return readBytes[0];
}

/**
 * Get the estimated distance to the lightning storm in kilometers.
 * Values beyond 40 are out of range.
 * Values less than 5 mean that the storm is overhead
 */
int AS3935_GetDistanceEstimation(uint8_t id)
{
  uint8_t distanceEstimate = (uint8_t)AS3935_ReadReg(id, AS3935_DIST_ESTI_REG_ADDR);
  return (int)distanceEstimate;
}

/**
 * Calibrate the RC Oscillators automatically.
 */
void AS3935_CalibrateRCO(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_CALIBR_RCO_REG_ADDR, AS3935_DIRECT_CMD_REG_VALU);
}

/*
 * Sets all registers in default mode
 */
void AS3935_PresetRegisterDefaults(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_PRESET_DEF_REG_ADDR, AS3935_DIRECT_CMD_REG_VALU);
}

/*
 * Set the analog front end and watchdog operating mode.
 * Refer to as3935.h for possible operating modes.
 */
void AS3935_SetAnalogFrontEnd(uint8_t id, uint8_t mode)
{
  uint8_t newAFESetting;
  uint8_t currentAFESetting = AS3935_ReadReg(id, AS3935_PWD_AFEGB_REG_ADDR);
  currentAFESetting = currentAFESetting & AFE_MASK;
  if (mode == AFE_OUTDOOR)
	  newAFESetting = currentAFESetting & AFE_OUTDOOR;
  	  AS3935_WriteReg(id, AS3935_PWD_AFEGB_REG_ADDR, newAFESetting);
  if (mode == AFE_INDOOR)
	  newAFESetting = currentAFESetting & AFE_OUTDOOR;
  	  AS3935_WriteReg(id, AS3935_PWD_AFEGB_REG_ADDR, newAFESetting);
}

/**
 * Retrieve the current operating mode of the AFE and watchdog.
 */
uint8_t AS3935_GetAnalogFrontEnd(uint8_t id)
{
  return AS3935_ReadReg(id, AS3935_PWD_AFEGB_REG_ADDR);
}

/**
 * Disable the disturber detection feature on the AS3935.
 */
void AS3935_DisableDisturbers(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_MASK_DIST_REG_ADDR, 1);
}

/**
 * Enable the disturber detection feature on the AS3935.
 */
void AS3935_EnableDisturbers(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_MASK_DIST_REG_ADDR, 0);
}

/**
 * Get the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 */
uint8_t AS3935_GetMinimumLightnings(uint8_t id)
{
  return AS3935_ReadReg(id, AS3935_MIN_LIGHT_REG_ADDR);
}

/**
 * Set the defined minimum number of lightning events in the last 17 minutes that issue a lightning interrupt.
 */
uint8_t AS3935_SetMinimumLightnings(uint8_t id, uint8_t minimumLightning)
{
  AS3935_WriteReg(id, AS3935_MIN_LIGHT_REG_ADDR, minimumLightning);
  return getMinimumLightnings();
}

/**
 * Get the defined threshold for the noise floor that triggers an interrupt.
 */
uint8_t AS3935_GetNoiseFloor(uint8_t id)
{
  return AS3935_ReadReg(id, AS3935_NFLV_WDTH_REG_ADDR);
}

/**
 * Set the defined threshold for the noise floor that triggers an interrupt.
 */
uint8_t AS3935_SetNoiseFloor(uint8_t id, int noiseFloor)
{
  AS3935_WriteReg(id, AS3935_NFLV_WDTH_REG_ADDR, noiseFloor);
  return AS3935_GetNoiseFloor(id);
}
