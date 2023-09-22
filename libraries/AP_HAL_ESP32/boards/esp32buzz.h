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

#define HAL_ESP32_BOARD_NAME "esp32-buzz"

#define TRUE 1
#define FALSE 0

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

//HARDWARE UARTS
//Default uart0 used for programming
//Default uart2	
#define HAL_ESP32_UART_DEVICES \
	{.port=UART_NUM_0, .rx=GPIO_NUM_3, .tx=GPIO_NUM_1 },\
	{.port=UART_NUM_1, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17 }

//INS choices:
#define HAL_INS_DEFAULT HAL_INS_MPU9250_I2C
//#define HAL_INS_MPU60x0_NAME "MPU6050"

//I2C bus list
//Internal means that this bus is used only for devices mounted on device pcb
#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_21, .scl=GPIO_NUM_22, .speed=400000, .internal=true, .soft = false}
	
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_NONE)

//SPI bus list
#define HAL_ESP32_SPI_BUSES {}
#define HAL_ESP32_SPI_DEVICES {}

// or this:
//#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
//#define HAL_INS_ICM20XXX_I2C_BUS 0
//#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

// BARO choices:
//#define HAL_BARO_DEFAULT HAL_BARO_NONE
#define HAL_BARO_BMP280_NAME "BMP280"
#define HAL_BARO_DEFAULT 				HAL_BARO_BMP280_I2C
#define HAL_BARO_PROBE_LIST 				PROBE_BARO_I2C(BMP280, 0, 0x76)
// or one of these:
//#define HAL_BARO_DEFAULT HAL_BARO_MS5837_I2C
// or: GPIO 34
//#define HAL_BARO_ANALOG_PIN (6)

// MAG/COMPASS choices:
// or others:
//#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
//#define HAL_COMPASS_AK09916_I2C_BUS 0
//#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)
//#define HAL_COMPASS_MAX_SENSORS 3
#define AP_COMPASS_QMC5883L_ENABLED   TRUE
//#define HAL_COMPASS_QMC5883L_I2C_ADDR 0x0D
//#define HAL_MAG_DEFAULT 				HAL_MAG_QMC5883L_I2C
#define HAL_MAG_PROBE_LIST 				PROBE_MAG_I2C(QMC5883L, 0, 0x0d, true ,  ROTATION_NONE)

#define DEFAULT_SERIAL1_PROTOCOL				SerialProtocol_MAVLink2			//C	WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
#define DEFAULT_SERIAL1_BAUD				AP_SERIALMANAGER_MAVLINK_BAUD/1000	//57600


// MAG/COMPASS probing:
//#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE));
// BARO probing:
//#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 0, 0x77)

// no airspeed sensor
#define AP_AIRSPEED_MS4525_ENABLED 0
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0



#define HAL_USE_ADC TRUE
#define AP_BATTERY_ANALOG_ENABLED TRUE
# define AP_BATT_VOLTDIVIDER_DEFAULT       HAL_BATT_VOLT_SCALE
# define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  HAL_BATT_CURR_SCALE
#define HAL_BATT_VOLT_PIN (35)  // adc1_7 gpio35
#define HAL_BATT_VOLT_SCALE (3.0)
#define HAL_BATT_CURR_PIN (34)  // adc1_7 gpio3
#define HAL_BATT_CURR_SCALE (1)
#define HAL_ESP32_ADC_PINS {\
	{ADC1_GPIO35_CHANNEL, 0.000805, 35},\
	{ADC1_GPIO34_CHANNEL, 11, 34},\
	{ADC1_GPIO39_CHANNEL, 11, 39},\
	{ADC1_GPIO36_CHANNEL, 11, 36}\
}

// the pin number, the gain/multiplier associated with it, the ardupilot name for the pin in parameter/s.
//
// two different pin numbering schemes, both are ok, but only one at a time:

// pick one:
//#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION1
//#define HAL_ESP32_ADC_PINS HAL_ESP32_ADC_PINS_OPTION2

#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1


// uncommenting one or more of these will give more console debug in certain areas.. ... 
// ...however all teh extra printf's use a lot of stack, so best to limit yourself to only uncommenting one at a time
//#define STORAGEDEBUG 1
//#define SCHEDDEBUG 1
//#define FSDEBUG 1
//#define BUSDEBUG 1 //ok
//#define WIFIDEBUG 1 //uses a lot?
//#define INS_TIMING_DEBUG 1 //define this to see all the imu-resets and temp resets and imu timing info on the console.

// 2 use udp, 1 use tcp...  for udp,client needs to connect as UDPCL in missionplanner etc to 192.168.4.1 port 14550
#define HAL_ESP32_WIFI 1
#define WIFI_STATION 1    // 0-AP, 1-STA
#define WIFI_HOSTNAME "FCU"
#define WIFI_SSID "ap-esp32"     // SSID for AP mode
#define WIFI_SSID_STATION "REDMI_S"        // SSID for STA mode
#define WIFI_PWD "2017214007"
#define WIFI_CHANNEL 7

// tip: if u are ok getting mavlink-over-tcp or mavlink-over-udp and want to disable mavlink-over-serial-usb
//then set ardupilot parameter SERIAL0_PROTOCOL = 0 and reboot.
// u also will/may want..
//LOG_BACKEND_TYPE 1
//LOG_DISARMED 1
//SERIAL0_PROTOCOL 0


// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif


// RCOUT pins for servos, motors etc.
#define HAL_ESP32_RCOUT { GPIO_NUM_25,GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32 }

// RCIN on what pin?
#define HAL_ESP32_RCIN GPIO_NUM_4


//If its will be set to 0, code will not compile! Throwing Lua related error
#define AP_FILESYSTEM_ESP32_ENABLED 1

// Do u want to use mmc or spi mode for the sd card, this is board specific ,
//  as mmc uses specific pins but is quicker,
//#define HAL_ESP32_SDMMC 0
// and spi is more flexible pinouts....  dont forget vspi/hspi should be selected to NOT conflict with SPI_BUSES above
#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18, .cs=GPIO_NUM_5}
#define HAL_ESP32_SDCARD 1
#define HAVE_FILESYSTEM_SUPPORT 1
#define HAL_OS_POSIX_IO 1
#define LOGGER_MAVLINK_SUPPORT 1
#define HAL_LOGGING_DATAFLASH_ENABLED			0
#define HAL_LOGGING_MAVLINK_ENABLED			0
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"

// this becomes the default value for the ardupilot param LOG_BACKEND_TYPE, which most ppl want to be 1, for log-to-flash
// setting to 2 means log-over-mavlink to a companion computer etc.
#define HAL_LOGGING_BACKENDS_DEFAULT 0

#define HAL_ESP32_RMT_RX_PIN_NUMBER 4

#define AC_PRECLAND_ENABLED FALSE
#define AP_BEACON_ENABLED FALSE
#define RANGEFINDER_ENABLED FALSE
#define HAL_PROXIMITY_ENABLED FALSE
#define MODE_SMARTRTL_ENABLED FALSE
#define AP_SERVORELAYEVENTS_ENABLED FALSE

