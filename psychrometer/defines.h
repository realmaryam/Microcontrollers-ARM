/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

/* Put your global defines for all libraries here used in your project */
/* Columns */
/* Column 1 default */
#define KEYPAD_COLUMN_1_PORT		GPIOD
#define KEYPAD_COLUMN_1_PIN		GPIO_PIN_0
/* Column 2 default */
#define KEYPAD_COLUMN_2_PORT		GPIOD
#define KEYPAD_COLUMN_2_PIN		GPIO_PIN_1
/* Column 3 default */
#define KEYPAD_COLUMN_3_PORT		GPIOD
#define KEYPAD_COLUMN_3_PIN		GPIO_PIN_2
/* Column 4 default */
#define KEYPAD_COLUMN_4_PORT		GPIOD
#define KEYPAD_COLUMN_4_PIN		GPIO_PIN_3

/* Rows */
/* Row 1 default */
#define KEYPAD_ROW_1_PORT			GPIOC
#define KEYPAD_ROW_1_PIN			GPIO_PIN_1
/* Row 2 default */
#define KEYPAD_ROW_2_PORT			GPIOC
#define KEYPAD_ROW_2_PIN			GPIO_PIN_2
/* Row 3 default */
#define KEYPAD_ROW_3_PORT			GPIOC
#define KEYPAD_ROW_3_PIN			GPIO_PIN_3
/* Row 4 default */
#define KEYPAD_ROW_4_PORT			GPIOC
#define KEYPAD_ROW_4_PIN			GPIO_PIN_5

#endif
