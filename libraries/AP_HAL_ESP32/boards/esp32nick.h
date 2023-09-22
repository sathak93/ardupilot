/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define HAL_ESP32_BOARD_NAME "esp32-nick"
// CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_NICK

#define TRUE						1
#define FALSE						0

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
//------------------------------------

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_NICK

//-----INS/IMU-----
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)

// -----BARO-----
#define HAL_BARO_BMP280_NAME "BMP280"
#define HAL_BARO_DEFAULT 				HAL_BARO_BMP280_I2C
#define HAL_BARO_PROBE_LIST 				PROBE_BARO_I2C(BMP280, 0, 0x76)

// allow boot without a baro
//#define HAL_BARO_ALLOW_INIT_NO_BARO 1

//-----ADC-----
#define HAL_USE_ADC TRUE
#define AP_BATTERY_ANALOG_ENABLED TRUE
# define AP_BATT_VOLTDIVIDER_DEFAULT       HAL_BATT_VOLT_SCALE
# define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  HAL_BATT_CURR_SCALE
#define HAL_BATT_VOLT_PIN (35)  // adc1_7 gpio35
#define HAL_BATT_VOLT_SCALE (3.0)
#define HAL_BATT_CURR_PIN (34)  // adc1_7 gpio3
#define HAL_BATT_CURR_SCALE (1)
#define HAL_ESP32_ADC_PINS {\
	{ADC1_GPIO35_CHANNEL, 11, 35},\
	{ADC1_GPIO34_CHANNEL, 11, 34},\
	{ADC1_GPIO39_CHANNEL, 11, 39},\
	{ADC1_GPIO36_CHANNEL, 11, 36}\
}


//-----COMPASS-----
#define AP_COMPASS_QMC5883L_ENABLED   TRUE
#define HAL_MAG_PROBE_LIST 				PROBE_MAG_I2C(QMC5883L, 0, 0x0d, true ,  ROTATION_NONE)
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1


//-----WIFI-----
#define HAL_ESP32_WIFI 1
#define WIFI_STATION 1    // 0-AP, 1-STA
#define WIFI_HOSTNAME "FCU"
#define WIFI_SSID "ap-esp32"     // SSID for AP mode
#define WIFI_SSID_STATION "REDMI_S"        // SSID for STA mode
#define WIFI_PWD "2017214007"
#define WIFI_CHANNEL 7

//-----RCOUT-----
#define HAL_ESP32_RCOUT { GPIO_NUM_25,GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32 }

//-----SPIBUS-----
#define HAL_ESP32_SPI_BUSES {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}
//#define HAL_ESP32_SPI_BUSES {.host=HSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_13, .miso=GPIO_NUM_12, .sclk=GPIO_NUM_14}
//-----SPIDEVICES-----
#define HAL_ESP32_SPI_DEVICES \
    {.name="mpu9250", .bus=0, .device=1, .cs=GPIO_NUM_5,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}

//-----I2CBUS-----
#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=GPIO_NUM_21, .scl=GPIO_NUM_22, .speed=400000, .internal=true, .soft = false}

//-----RCIN-----
#define HAL_ESP32_RCIN GPIO_NUM_4
//RMT pin number
#define HAL_ESP32_RMT_RX_PIN_NUMBER			4
//-----UARTS-----
#define HAL_ESP32_UART_DEVICES \
  {.port=UART_NUM_0, .rx=GPIO_NUM_3, .tx=GPIO_NUM_1 } \
  ,{.port=UART_NUM_1, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17 }

//FILESYSTEM SUPPORT
//#define HAVE_FILESYSTEM_SUPPORT 1
//#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
//#define HAL_ESP32_SDMMC 1
//#define HAL_ESP32_SDCARD 1
//#define HAL_ESP32_SDSPI {.host=HSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_13, .miso=GPIO_NUM_12, .sclk=GPIO_NUM_14, .cs=GPIO_NUM_15}
//#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18, .cs=GPIO_NUM_5}



//LOGGING
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define LOGGER_MAVLINK_SUPPORT 0
#define HAL_LOGGING_BACKENDS_DEFAULT 0
#define HAL_LOGGING_DATAFLASH_ENABLED			0
#define HAL_LOGGING_MAVLINK_ENABLED			0
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
//TERRAIN
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//If its will be set to 0, code will not compile! Throwing Lua related error
//#define AP_FILESYSTEM_ESP32_ENABLED 1

#define AC_PRECLAND_ENABLED FALSE
#define AP_BEACON_ENABLED FALSE
#define RANGEFINDER_ENABLED FALSE
#define HAL_PROXIMITY_ENABLED FALSE
#define MODE_SMARTRTL_ENABLED FALSE
#define AP_SERVORELAYEVENTS_ENABLED FALSE
