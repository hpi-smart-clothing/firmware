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
#define REQUIRED_NUMBER_OF_RESTARTS 50
#define SAMPLE_FREQUENCY 100 // in ms, not smaller than 50ms
#define REQUIERD_TIME_AFTER_IMU_START 420

#define MOTOR_LEFT_PIN D2
#define MOTOR_RIGHT_PIN D3
#define MOTOR_LEFT_LEDC_CHANNEL 0
#define MOTOR_RIGHT_LEDC_CHANNEL 0
#define MOTOR_PWM_FREQUENCY 5000 

#define BLE_DEVICE_NAME "BackUp"

#define READ_VOLTAGE_PIN D1
#define READ_VOLTAGE_RESISTOR1 8200.0
#define READ_VOLTAGE_RESISTOR2 18000.0
#define ADCRESOLUTION 4096.0
#define ADC_VREF 3.3
struct VoltageEntry {
    float mesuredVoltage;
    float realVoltage;
    int percentage;
};

const VoltageEntry VoltageLevel[] = {
    {4.8, 4.2, 100},
    {4.42, 3.9, 80},
    {4.3, 3.8, 60},
    {4.175, 3.7, 40},
    {4.05, 3.6, 20},
    {3.8, 3.4, 10},
    {3.6, 3.2, 5}
};

#endif