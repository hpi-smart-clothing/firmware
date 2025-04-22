#ifndef CONFIG_H
#define CONFIG_H

#define SDA_PIN D4
#define SCL_PIN D5
#define IMU_SPI_ADDRESS 0x29 // I2C address of the BNO055 sensor
#define TCA_ADDRESS 0x70 // Address of MUX
#define IMU_QUAT_DATA_ADDRESS 0x20
#define IMU_CHIP_ID 0xA0 // ID of the imu sensor in the 0x00 register
#define IMU_CHIP_ID_ADDRESS 0x00
#define IMU_PAGE_ID 0x00
#define IMU_PAGE_ID_ADDRESS 0x07
#define IMU_POWER_MODE 0x00
#define IMU_POWER_MODE_ADDRESS 0x3E
#define IMU_OPERATION_MODE_CONFIG 0x00
#define IMU_OPERATION_MODE_NDOF 0x0C
#define IMU_OPERATION_MODE_ADDRESS 0x3D
#define IMU_SYS_TRIGGER 0x80
#define IMU_SYS_TRIGGER_ADDRESS 0x3F
#define IMU_INIT_DELAY 30
#define NUMBER_IMUS 6
#define MOTOR_1_PIN D0
#define MOTOR_2_PIN D1

#endif