#include <stdbool.h>

#ifndef INC_AGS02MA_H_
#define INC_AGS02MA_H_

#define SCD4X_I2C_PORT	hi2c1
extern I2C_HandleTypeDef SCD4X_I2C_PORT;
#ifndef AGS02MA_I2CADDR_DEFAULT
#define AGS02MA_I2CADDR_DEFAULT 0x1A << 1  ///< AHT default i2c address
#endif
#define _AGS02MA_TVOCSTAT_REG 0x00    ///< Status and TVOC reading
#define _AGS02MA_VERSION_REG 0x11     ///< Rirmware version
#define _AGS02MA_GASRES_REG 0x20      ///< Raw gas resistance
#define _AGS02MA_SETADDR_REG 0x21     ///< Change I2C addr
#define _AGS02MA_CRC8_INIT 0xFF       ///< CRC8 init val
#define _AGS02MA_CRC8_POLYNOMIAL 0x31 ///< CRC8 polynomial

uint32_t AGS02MA_getFirmwareVersion(void);
uint32_t AGS02MA_getTVOC(void);
bool AGS02MA_readReg(uint8_t addr, uint16_t delayms, uint32_t *value);

#endif /* INC_AGS02MA_H_ */
