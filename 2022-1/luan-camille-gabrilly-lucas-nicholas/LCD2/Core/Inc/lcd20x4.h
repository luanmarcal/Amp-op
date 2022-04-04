/*
 * lcd1602.h
 *
 *  Created on: Jan 21, 2020
 *      Author: Controllerstech
 */

#ifndef INC_LCD20x4_H_
#define INC_LCD20x4_H_

#include "stm32g0xx.h"

/*
 * Enumerates
 */
typedef enum{
	RS,
	RW,
	E,
	D4,
	D5,
	D6,
	D7,

	LCD_QTD
}lcd_pin_e;


/*
 * Function Prototypes
 */

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_attr_gpio(lcd_pin_e lcd, GPIO_TypeDef *gpio, uint16_t pin);

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

#endif /* INC_LCD1602_H_ */
