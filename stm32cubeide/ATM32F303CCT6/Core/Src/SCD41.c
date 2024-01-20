#include "stm32f3xx_hal.h"
#include "SCD41.h"

/** Function for control SCD41
 */
void setSleepMode(uint16_t mode) {
  SCDwriteData(mode, NULL, 0);
  if(SCD4X_WAKE_UP == mode)
    HAL_Delay(20);   // Give it some time to switch mode
}

uint16_t performSelfTest(void) {
  SCDwriteData(SCD4X_PERFORM_SELF_TEST, NULL, 0);
  HAL_Delay(10000);
  uint8_t buf[3] = {0};
  return SCD4X_CONCAT_BYTES(buf[0], buf[1]);
}

void moduleReinit(void) {
  SCDwriteData(SCD4X_REINIT, NULL, 0);
  HAL_Delay(20);
}

void performFactoryReset(void) {
  SCDwriteData(SCD4X_PERFORM_FACTORY_RESET, NULL, 0);
  HAL_Delay(1200);
}

/*
 * Measurement Function for SCD41
 */

void measureSingleShot(uint16_t mode) {
  SCDwriteData(mode, NULL, 0);
  if(SCD4X_MEASURE_SINGLE_SHOT == mode) {
	  HAL_Delay(5000);
  } else if(SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY == mode) {
	  HAL_Delay(50);
  }
}

void enablePeriodMeasure(uint16_t mode) {
	uint8_t trData[2];
	trData[1] = mode & 0xFF;
	trData[0] = (mode >> 8) & 0xFF;
	HAL_I2C_Master_Transmit(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, trData, 2, 100);
  //SCDwriteData(mode, NULL, 0);
  if(SCD4X_STOP_PERIODIC_MEASURE == mode)
	HAL_Delay(500);   // Give it some time to switch mode
}

bool getDataReadyStatus(void) {
  uint8_t buf[3] = {0};
  SCDreadData(SCD4X_GET_DATA_READY_STATUS, buf, sizeof(buf));
  if(0x0000 == (SCD4X_CONCAT_BYTES(buf[0], buf[1]) & 0x7FF)){
    return false;
  }
  return true;
}

void persistSettings(void) {
  SCDwriteData(SCD4X_PERSIST_SETTINGS, NULL, 0);
  HAL_Delay(800);
}

bool getSerialNumber(uint16_t * wordBuf) {
  bool ret = true;
  uint8_t buf[9] = {0};
  if(sizeof(buf) != SCDreadData(SCD4X_GET_SERIAL_NUMBER, buf, sizeof(buf))) {
    ret = false;
  }
  wordBuf[0] = SCD4X_CONCAT_BYTES(buf[0], buf[1]);
  wordBuf[1] = SCD4X_CONCAT_BYTES(buf[3], buf[4]);
  wordBuf[2] = SCD4X_CONCAT_BYTES(buf[6], buf[7]);
  return ret;
}

/****************************** Read/Write Command Function ********************************/

void SCDwriteData(uint16_t cmd, uint8_t *pBuf, size_t size) {
  HAL_I2C_Mem_Write(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, cmd, 2, NULL, 0, 100);
}

size_t SCDreadData(uint16_t cmd, uint8_t *pBuf, size_t size) {
  HAL_I2C_Mem_Write(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, cmd, 2, NULL, 0, 100);
  HAL_I2C_Master_Receive(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, pBuf, size, 100);
  return size;
}

void readMeasurement(sSensorMeasurement_t * data) {
  uint8_t buf[9] = {0};
  SCDreadData(SCD4X_READ_MEASUREMENT, buf, sizeof(buf));
  data->CO2ppm = SCD4X_CONCAT_BYTES(buf[0], buf[1]);
  data->temp = -45 + 175 * (float)(SCD4X_CONCAT_BYTES(buf[3], buf[4])) / ((uint32_t)1 << 16);
  data->humidity = 100 * (float)(SCD4X_CONCAT_BYTES(buf[6], buf[7])) / ((uint32_t)1 << 16);
}
