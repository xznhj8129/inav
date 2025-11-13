#pragma once

#define TARGET_BOARD_IDENTIFIER "WAF4"		
#define USBD_PRODUCT_STRING  "WEACTF405V1"	

/*** Indicators ***/
#define LED0                    PB2
#define BEEPER                  PC5
#define BEEPER_INVERTED			


// *************** UART *****************************
#define USB_IO
#define USE_VCP

#define USE_UART1
#define UART1_RX_PIN            PA10
#define UART1_TX_PIN            PA9

#define USE_UART2
#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define USE_UART4
#define UART4_RX_PIN            PA1
#define UART4_TX_PIN            PA0

#define USE_UART6
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       5

// *************** Gyro & ACC **********************
#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS

#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7


#define USE_IMU_MPU9250
#define IMU_MPU9250_ALIGN       CW0_DEG
#define MPU9250_CS_PIN          PA4
#define MPU9250_SPI_BUS         BUS_SPI1

#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW0_DEG
#define MPU6500_SPI_BUS         BUS_SPI1
#define MPU6500_CS_PIN          PA4

#define USE_IMU_MPU6000
#define IMU_MPU6000_ALIGN       CW0_DEG
#define MPU6000_SPI_BUS         BUS_SPI1
#define MPU6000_CS_PIN          PA4

// *************** I2C(Baro & I2C) **************************
#define USE_I2C
#define USE_I2C_DEVICE_2

#define I2C1_SCL                PB10
#define I2C1_SDA                PB11

#define DEFAULT_I2C_BUS         BUS_I2C2

// Baro
#define USE_BARO
#define USE_BARO_SPL06
#define USE_BARO_BMP280
#define USE_BARO_SPI_BMP280
#define BMP280_SPI_BUS          BUS_SPI1
#define BMP280_CS_PIN           PC3
#define USE_BARO_BMP085
#define BARO_I2C_BUS          	BUS_I2C2

// Mag
#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C2 
#define USE_MAG_ALL

// *************** Internal SD card **************************
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_SDIO_DMA         DMA_TAG(2,3,4)
#define SDCARD_SDIO_4BIT
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT


// *************** ADC *****************************
#define USE_ADC
#define ADC_CHANNEL_1_PIN           PC0
#define ADC_CHANNEL_2_PIN           PC1
#define ADC_CHANNEL_3_PIN           PC2

#define VBAT_ADC_CHANNEL            ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL   ADC_CHN_2
#define RSSI_ADC_CHANNEL            ADC_CHN_3

#define DEFAULT_FEATURES            (FEATURE_TX_PROF_SEL | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_VBAT |  FEATURE_BLACKBOX)
#define USE_DSHOT
#define USE_DSHOT_DMAR
// #define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define MAX_PWM_OUTPUT_PORTS        9

#define CURRENT_METER_SCALE         386

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))
