/*
 * BMP280.h
 *
 *  Created on: Mar 12, 2025
 *      Author: Bugrahan
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

typedef struct {
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_CalibData;

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} BMP280_TempCalibData;

extern BMP280_CalibData bmp280_pres_calib;
extern BMP280_TempCalibData bmp280_temp_calib;

uint8_t BMP280_chip_address_al();
uint8_t BMP280_default_address_al();
void BMP280_yapilandir();
float BMP280_sicaklik_al();
float BMP280_irtifa_al();
/*
static uint32_t BMP280_raw_basinc_oku();
static void BMP280_basinc_okuma_kalibrasyon();
static void BMP280_sicaklik_okuma_kalibrasyon();
static uint32_t BMP280_raw_sicaklik_oku();
static int32_t BMP280_sicaklik_hesapla();
static uint32_t bmp280_basinc_oku();
*/


#endif /* INC_BMP280_H_ */
