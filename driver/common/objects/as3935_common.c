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

void AS3935_WriteReg(uint8_t id, uint8_t addr, uint8_t data)
{
  uint8_t writeBytes[2];
  writeBytes[0] = addr;
  writeBytes[1] = data;
  AIR_I2C_WRITE(AS3935_I2C_BASE_ADDR + id, writeBytes, 2);
}

uint8_t AS3935_ReadReg(uint8_t id, uint8_t addr)
{
  uint8_t writeBytes[1] = {0};
  uint8_t readBytes[1] = {0};
  AIR_I2C_ComboRead(AS3935_I2C_BASE_ADDR + id, writeBytes, 1, readBytes, 1);
  return readBytes[0];
}

uint8_t AS3935_GetDistanceEstimation(uint8_t id)
{
  return AS3935_ReadReg(id, AS3935_DIST_ESTI_REG_ADDR);
}

void AS3935_CalibrateRCO(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_CALIBR_RCO_REG_ADDR, 0x96);
}

void AS3935_PresetRegisterDefaults(uint8_t id)
{
  AS3935_WriteReg(id, AS3935_PRESET_DEF_REG_ADDR, 0x96);
}
