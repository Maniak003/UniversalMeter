#include "stm32f3xx_hal.h"
#include "ZE08.h"

uint8_t ZE08_Buffer[9];

//uint16_t ZE08_readData(uint8_t *ZE08_Buffer, uint16_t Size) {
uint16_t ZE08_readData() {
	int Size = sizeof(ZE08_Buffer);
	for (int ii = 0; ii < Size; ii++) {
		ZE08_Buffer[ii] = 0;
	}
	uint32_t uart_tm_out = HAL_GetTick();
	while ((uart_tm_out + 2000) > HAL_GetTick()) {
		if ((HAL_UART_Receive(&ZE08_UART_PORT, ZE08_Buffer, 3, 20) == HAL_OK)
				&& (ZE08_Buffer[0] == 0xff && ZE08_Buffer[1] == 0x17 && ZE08_Buffer[2] == 0x04)) {
			if (HAL_UART_Receive(&ZE08_UART_PORT, ZE08_Buffer + 3, Size - 3, 30) == HAL_OK) {
				return  ZE08_Buffer[4] << 8 | ZE08_Buffer[5];
			} else {
				return 0;
			}
		} else {
			continue;
		}
	}
	return 0xFFFF;
}
