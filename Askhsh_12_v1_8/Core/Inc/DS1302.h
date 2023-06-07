/*
 * 	Как пользоваться:
 * 		1. Заменить заголовочный файл "stm32f3xx_hal.h" в соответствии с используемым контроллером. Я использую STM32F303VCT.
 * 		2. Переназначить порт и выводы, к которым подключена микросхема (DS1302_GPIO, DS1302_SCLK, DS1302_SDA, DS1302_RST).
 * 		3. В функции main выполнить DS1302_Init();.
 * 		4. Можно пользоваться.
 */

#ifndef _DS1302_H
#define _DS1302_H

#include "main.h"
#include "dwt_delay.h"

//----------------------------------------------------------------------------------
//Ниже приведенный порт и выводы переназначить на свои
//----------------------------------------------------------------------------------
#define DS1302_GPIO				GPIOF
#define DS1302_SCLK				RTC_CLK_Pin
#define DS1302_SDA				RTC_IO_Pin
#define DS1302_RST				RTC_RST_Pin


/* Инициализация */
/* Настройка выводов STM32, запуск таймера микросекунд и вывод часов из режима энергосбережения */
void DS1302_Init(void);

/* Reads time byte by byte to 'buf' */
void DS1302_ReadTime(uint8_t *buf);

/* Writes time byte by byte from 'buf' */
void DS1302_WriteTime(uint8_t *buf); 

/* Writes 'val' to ram address 'addr' */
/* Ram addresses range from 0 to 30 */
void DS1302_WriteRam(uint8_t addr, uint8_t val);

/* Чтение из RAM по адресу 'addr' */
uint8_t DS1302_ReadRam(uint8_t addr);

/* Очистка RAM памяти */
void DS1302_ClearRam(void);

/* Reads time in burst mode, includes control byte */
void DS1302_ReadTimeBurst(uint8_t * temp);

/* Writes time in burst mode, includes control byte */
void DS1302_WriteTimeBurst(uint8_t * buf);

/* Reads ram in burst mode 'len' bytes into 'buf' */
void DS1302_ReadRamBurst(uint8_t len, uint8_t * buf);

/* Writes ram in burst mode 'len' bytes from 'buf' */
void DS1302_WriteRamBurst(uint8_t len, uint8_t * buf);

//Запуск часов.
void DS1302_ClockStart(void);

//Остановка часов.
void DS1302_ClockStop(void);

//Сброс часов
void DS1302_ClockClear(void);

#endif //_DS1302_H
