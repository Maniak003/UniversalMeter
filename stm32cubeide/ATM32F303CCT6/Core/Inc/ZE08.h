#include <stdbool.h>
#ifndef INC_ZE08_H_
#define INC_ZE08_H_

#define ZE08_UART_PORT	huart2
extern UART_HandleTypeDef ZE08_UART_PORT;
uint16_t ZE08_readData();
//uint16_t ZE08_readData(uint8_t *ZE08_Buffer, uint16_t Size);
#endif /* INC_ZE08_H_ */
