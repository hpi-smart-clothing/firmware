FIRMWARE FOR BNO055 WITH MULTIPLEXER

IMU DATA FORMAT:

One quaternion is stored in an 8 byte array.
A quaternion consists of 4 values. (w, x, y, z)
Each of these values is stored in 2 bytes. 
The first byte in the array is the lsb and the second one the msb.

The ESP32-C3 sends a 48 byte Array of Quaternion Data with BLE.
Array[0]-Array[7] contains quat data for imu 1.
Array[8]-Array[15] contains quat data for imu 2.
... 