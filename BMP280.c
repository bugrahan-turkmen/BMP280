/*
 * BMP280.c
 *
 *  Created on: Mar 12, 2025
 *      Author: Bugrahan
 *      Kullanmadan once main fonskiyonda BMP280_yapilandir() kodunu çalistir.
 *      irtifa degeri icin BMP280_irtifa_al() kullanılabilir.
 *      sicaklik degeri icin BMP280_sicaklik_al() kullanılabilir.
 *
 *      I2C standart mode olarak calisir
 *      BMP280		STM32
 *      Vin	------>	3.3V
 *      GND	------>	GND
 *      SCK	------>	SCL(PB6)
 *      Vin	------>	3.3V
 *      SDO	------>	OPEN 		//default addres(0x77)'de kullanmak icin GND/OPEN ye baglanır. 3.3V baglanirsa 0x76 olur.
 *      SDI	------>	SDA(PB7)
 *      CS	------>	3.3V		//I2C kullanımında 3.3V a baglanir. GND baglanırsa SPI haberlesir.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "BMP280.h"
#include <math.h>
#define BME280_I2C_ADDRESS  (0x77 << 1)

extern I2C_HandleTypeDef hi2c1;	//********kullandigin I2C pinlerine gore degistir!!!!!!!!

BMP280_CalibData bmp280_pres_calib;
BMP280_TempCalibData bmp280_temp_calib;

extern volatile float sicaklik;
/*
 *BMP280 mi degil mi oldugunu anlamak icin kullanilir. chip_id = 0x58 olmalidir.
 */
uint8_t BMP280_chip_address_al(){
	  uint8_t chip_id_reg = 0xD0;
	  uint8_t chip_id;
	  HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &chip_id_reg, 1, 100);
	  HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, &chip_id, 1, 100);
	  return chip_id;
}


//  Sensörün default adresini alir yani bir nevi haberleşme var mi kontrolü. 0x77 döndürmeli

uint8_t BMP280_default_address_al() {
    HAL_StatusTypeDef res;
    uint8_t i;
    uint8_t address;
    for (i = 1; i < 127; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, HAL_MAX_DELAY);
        if (res == HAL_OK) {
            address=i;
        }
    }
    return address;
}

//mainde fonksiyonda bir kez calismasi yeterlidir yapilandirma fonskiyonudur.
void BMP280_yapilandir() {
	uint8_t configData[2];
	configData[0] = 0xF5;
	configData[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, configData, 2, 100);
	configData[0] = 0xF4;
	configData[1] = 0x2B;
	HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, configData, 2, 100);
}

static uint32_t BMP280_raw_basinc_oku() {
    uint8_t press_reg = 0xF7;
    uint8_t press_data[3];
    uint32_t press_raw;

    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &press_reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, press_data, 3, 100);

    press_raw = ((uint32_t)press_data[0] << 12) | ((uint32_t)press_data[1] << 4) | (press_data[2] >> 4);

    return press_raw;
}

//basinc okumak icin kaalibrasyon degerleri alınır.
static void BMP280_basinc_okuma_kalibrasyon() {
    uint8_t calib_data[18];
    uint8_t reg = 0x8E;
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, calib_data, 18, 100);

    bmp280_pres_calib.dig_P1 = (uint16_t)(calib_data[0] | (calib_data[1] << 8));
    bmp280_pres_calib.dig_P2 = (int16_t)(calib_data[2] | (calib_data[3] << 8));
    bmp280_pres_calib.dig_P3 = (int16_t)(calib_data[4] | (calib_data[5] << 8));
    bmp280_pres_calib.dig_P4 = (int16_t)(calib_data[6] | (calib_data[7] << 8));
    bmp280_pres_calib.dig_P5 = (int16_t)(calib_data[8] | (calib_data[9] << 8));
    bmp280_pres_calib.dig_P6 = (int16_t)(calib_data[10] | (calib_data[11] << 8));
    bmp280_pres_calib.dig_P7 = (int16_t)(calib_data[12] | (calib_data[13] << 8));
    bmp280_pres_calib.dig_P8 = (int16_t)(calib_data[14] | (calib_data[15] << 8));
    bmp280_pres_calib.dig_P9 = (int16_t)(calib_data[16] | (calib_data[17] << 8));

}

//sicaklik okumak icin kaalibrasyon degerleri alınır.
static void BMP280_sicaklik_okuma_kalibrasyon() {
    uint8_t calib_data[6];
    uint8_t reg = 0x88;

    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, 0x77 << 1, calib_data, 6, 100);

    bmp280_temp_calib.dig_T1 = (uint16_t)(calib_data[0] | (calib_data[1] << 8));
    bmp280_temp_calib.dig_T2 = (int16_t)(calib_data[2] | (calib_data[3] << 8));
    bmp280_temp_calib.dig_T3 = (int16_t)(calib_data[4] | (calib_data[5] << 8));
}

static uint32_t BMP280_raw_sicaklik_oku() {
    uint8_t temp_reg = 0xFA;
    uint8_t temp_data[3];
    uint32_t temp_raw;

    HAL_I2C_Master_Transmit(&hi2c1, BME280_I2C_ADDRESS, &temp_reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, BME280_I2C_ADDRESS, temp_data, 3, 100);

    temp_raw = ((uint32_t)temp_data[0] << 12) | ((uint32_t)temp_data[1] << 4) | (temp_data[2] >> 4);

    return temp_raw;
}

static uint32_t t_fine;

static int32_t BMP280_sicaklik_hesapla() {
	BMP280_sicaklik_okuma_kalibrasyon();
    uint32_t adc_T = BMP280_raw_sicaklik_oku();
    uint32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((uint32_t)bmp280_temp_calib.dig_T1 << 1))) * ((uint32_t)bmp280_temp_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((uint32_t)bmp280_temp_calib.dig_T1)) * ((adc_T >> 4) - ((uint32_t)bmp280_temp_calib.dig_T1))) >> 12) *
            ((uint32_t)bmp280_temp_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    T = (t_fine * 5 + 128) >> 8;
    return T;
}

float BMP280_sicaklik_al() {
    int32_t temp_C100 = BMP280_sicaklik_hesapla();
    return temp_C100 / 100.0;  // °C'ye çevir
}


static uint32_t bmp280_basinc_oku(){
	int32_t adc_P = BMP280_raw_basinc_oku();
	BMP280_basinc_okuma_kalibrasyon();
	sicaklik = BMP280_sicaklik_al();
	int64_t var1, var2, p;
	var1 = (int64_t)t_fine - 128000;
	var2 = var1 * var1 * (int64_t)bmp280_pres_calib.dig_P6;
	var2 = var2 + ((var1*(int64_t)bmp280_pres_calib.dig_P5)<<17);
	var2 = var2 + (((int64_t)bmp280_pres_calib.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp280_pres_calib.dig_P3)>>8) + ((var1 * (int64_t)bmp280_pres_calib.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp280_pres_calib.dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bmp280_pres_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bmp280_pres_calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_pres_calib.dig_P7)<<4);
	return (uint32_t)p;
}


float BMP280_irtifa_al() {
    float basinc = (float)bmp280_basinc_oku()/25600;
    if (basinc <= 0) return 0;

    //  Barometrik irtifa formülü: h = (T0 / L) * [1 - (P / P0)^(R * L / g * M)]
    const float T0 = 288.15;      // Deniz seviyesinde sıcaklık (Kelvin)
    const float L = 0.0065;       // Atmosferde sıcaklık azalım oranı (K/m)
    const float P0 = 1013.25;     // Deniz seviyesindeki standart basınç (hPa)
    const float R = 8.31432;      // Evrensel gaz sabiti (J/(mol·K))
    const float g = 9.80665;      // Yerçekimi ivmesi (m/s²)
    const float M = 0.0289644;    // Havanın mol kütlesi (kg/mol)

    return (T0 / L) * (1.0f - powf(basinc / P0, (R * L) / (g * M)));
}

