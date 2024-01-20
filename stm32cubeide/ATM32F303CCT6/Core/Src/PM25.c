#include "stm32f3xx_hal.h"
#include "PM25.h"
/*
   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31
  S0  S1  L0  L1  D1  D1  D2  D2  D3  D3  D4  D4  D5  D5  D6  D6  D7  D7  D8  D8  D9  D9 D10 D10 D11 D11 D12 D12 D13 D13  CS  CS
*/
uint16_t PM25_GetData(uint8_t *Data_Buffer, uint16_t Size) {
	for (int ii = 0; ii < Size; ii++) {
		Data_Buffer[ii] = 0;
	}
	uint32_t uart_tm_out = HAL_GetTick();
	while ((uart_tm_out + 2000) > HAL_GetTick()) {
		if ((HAL_UART_Receive(&PM25_UART_PORT, Data_Buffer, 4, 20) == HAL_OK)
						&& (Data_Buffer[0] == 0x42 && Data_Buffer[1] == 0x4d && Data_Buffer[2] == 0x00 && Data_Buffer[3] == 0x1c )) {
			if (HAL_UART_Receive(&PM25_UART_PORT, Data_Buffer + 4, Size - 4, 30) == HAL_OK) {
				return 1;
			}
			return 0;
		}
	}
	return 0;
}

