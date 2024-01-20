#include "stm32f3xx_hal.h"
#include <AGS02MA.h>

uint32_t AGS02MA_getFirmwareVersion(void) {
  uint32_t vers = 0;
  if (! AGS02MA_readReg(_AGS02MA_VERSION_REG, 30, &vers)) {
    return 1;
  }
  return vers;
}


uint32_t AGS02MA_getTVOC(void) {
  uint32_t tvoc = 0;
  if (! AGS02MA_readReg(_AGS02MA_TVOCSTAT_REG, 1500, &tvoc)) {
    return 0;
  }
  return tvoc & 0xFFFFFF;
}


//static uint8_t crc8(const uint8_t *data, int len) {
  /* CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

//  uint8_t POLYNOMIAL = 0x31;
//  uint8_t crc = 0xFF;

//  for (int j = len; j; --j) {
//    crc ^= *data++;

//    for (int i = 8; i; --i) {
//      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
//    }
//  }
//  return crc;
//}

bool AGS02MA_readReg(uint8_t addr, uint16_t delayms, uint32_t *value) {
	uint8_t buf[5];
	buf[0] = addr;

	HAL_I2C_Master_Transmit(&SCD4X_I2C_PORT, AGS02MA_I2CADDR_DEFAULT, buf, 1, 100);

	//if (!i2c_dev->write(buf, 1)) {
	//return false;
	//}
	HAL_Delay(delayms);

	HAL_I2C_Master_Receive(&SCD4X_I2C_PORT, AGS02MA_I2CADDR_DEFAULT, buf, 5, 100);
	//if (!i2c_dev->read(buf, 5)) {
	//return false;
	//}

	//if (crc8(buf, 4) != buf[4]) {
	//return false;
	//}

	uint32_t temp = buf[0];
	temp <<= 8;
	temp |= buf[1];
	temp <<= 8;
	temp |= buf[2];
	temp <<= 8;
	temp |= buf[3];
	*value = temp;
	return true;
}
