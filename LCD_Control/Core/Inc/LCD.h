/*
	Library: 			LCD 16X2 
	Written by:  		Ozan Hamdi KAPLAN
	Date Written:		04/09/2023
	Description:		
	References**:
    
	                   INSTRUCTION          		 	 || DECIMAL || HEXADECIMAL
	Function set (8-bit interface, 2 lines, 5*7 Pixels)  ||   56    ||     38
	Clear Screen										 ||   1     ||     01
	Return Home											 ||   2     ||     02
	Scroll display one character right (all lines)		 ||   28    ||     1E
	Scroll display one character left (all lines)		 ||   24    ||     18
	Move cursor one character left					 	 ||   16    ||     10
	Move cursor one character right					 	 ||   20    ||     14
	Turn on visible blinking-block cursor			 	 ||   15    ||     0F
	Turn on visible underline  cursor				 	 ||   14    ||     0E
	Make invisible  cursor				 				 ||   12    ||     0C




*/

//-----Header Files-----//
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>

//-----List of Commands-----//
#define LCD_CLEARSCREEN                 0x01 
#define LCD_RETURNHOME                  0x02
#define LCD_FUNCTION8BIT                0x38
#define LCD_FUNCTION4BIT                0x28
#define LCD_ENTRYMODESETNM              0x06
#define LCD_SCROLL_R                    0x1E
#define LCD_SCROLL_L                    0x18
#define LCD_MOVECURSOR_L                0x10
#define LCD_MOVECURSOR_R                0x14
#define LCD_BLINKCURSOR                 0x0F
#define LCD_UNDERLINECURSOR             0x0E
#define LCD_INVISIBLECURSOR             0x0C

//-----Function Prototype-----//

// 1-)LCD set 4bits function
void LCD_Set4(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
// 2-)LCD set 8bits function
void LCD_Set8(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_LSBs0to3,
uint16_t D0, uint16_t D1, uint16_t D2, uint16_t D3, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
// 3-)Write 4 bits command, *FOR 4 BITS MODE ONLY*
void LCD_Command4Send(uint8_t nibble);
// 4-)Write command
void LCD_CommandSend(uint8_t command);
// 5-)Write Parallel interface
void LCD_write(uint8_t byte);
// 6-)RS control
void LCD_RS_State(bool state);
// 7-)Enable EN pulse
void LCD_E_State(void);
// 8-)Microsecond delay functions
void LCD_TIM_Config(void);
void LCD_TIM_MicroSecDelay(uint32_t uSecDelay);
// 9-)Set cursor position
void LCD_setCursor(uint8_t row,uint8_t col);
void LCD_1stLine(void);
void LCD_2ndLine(void);
// 10-)LCD print string
void LCD_print(char string[]);
// 11-)Write 8 bits data
void LCD_writeData(uint8_t data);
// 12-) Clear display
void LCD_Clear(void);
// 13-) Cursor ON/OFF
void LCD_cursorOff(void);
// 14) Blinking cursor
void LCD_noBlink(void);
