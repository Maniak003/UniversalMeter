#include <stdbool.h>
#ifndef INC_PM25_H_
#define INC_PM25_H_

#define PM25_UART_PORT	huart1
extern UART_HandleTypeDef PM25_UART_PORT;
uint16_t PM25_GetData(uint8_t *Data_Bufer, uint16_t Size);

#endif /* INC_PM25_H_ */
