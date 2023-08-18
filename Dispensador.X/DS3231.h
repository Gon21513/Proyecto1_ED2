#ifndef DS3231_H
#define	DS3231_H

#include <xc.h> 
#include <stdint.h>
#include "I2C.h"

// Dirección del DS3231
#define DS3231_WRITE_ADDR 0xD0
#define DS3231_READ_ADDR  0xD1

// Funciones públicas
void DS3231_Init(void);
uint8_t DS3231_ReadSeconds(void);
// Prototipos de funciones

uint8_t Read(uint8_t address);
void Read_Time(uint8_t *s, uint8_t *m, uint8_t *h);
void Read_Fecha(uint8_t *d, uint8_t *mo, uint8_t *y);
void Set_sec(uint8_t sec);
void Set_min(uint8_t min);
void Set_hour(uint8_t hour);
void Set_day_week(uint8_t day_week);
void Set_day(uint8_t day);
void Set_month(uint8_t month);
void Set_year(uint8_t year);
uint8_t Dec_to_Bcd(uint8_t dec_number);
uint8_t Bcd_to_Dec(uint8_t bcd);

#endif	/* DS3231_H */
