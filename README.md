
# 📦 Firmware only for Data-Collection with BNO055 with Multiplexer

## Format

This Firmware is only for collecting data, it does not work with BLE, it does not trigger the vibration motors.

It sends all Data the Sensors collect, not only the Quaternions. Its sends it in the following Format:

Data: Begin Data from IMU 5:
Data: IMU 5 Accelerometer: X: 0.00 Y: 73.62 Z: 59.75 m/s^2
Data: IMU 5 Magnetometer: X: 0.00 Y: 73.62 Z: 59.75 uT
Data: IMU 5 Gyroscope: X: 0.00 Y: 73.62 Z: 59.75 rad/s
Data: IMU 5 Quaternion: W: 0.76 X: -0.16 Y: -0.63 Z: -0.00
Data: IMU 5 Temperature: 0.00 C
Data: IMU 5 Euler: Heading: 0.00 Roll: 73.62 Pitch: 59.75 degrees
Data: End Data from IMU 5
