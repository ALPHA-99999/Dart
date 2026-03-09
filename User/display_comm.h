#ifndef DISPLAY_COMM_H_
#define DISPLAY_COMM_H_

#include <stm32h7xx_hal.h>

// 显示和通信函数声明
void Send_toled(void);
uint8_t LCD_callback(uint8_t *recBuffer, uint16_t len);
void Lcd_control(void);

#endif /* DISPLAY_COMM_H_ */
