#include <Wire.h>
#include <Arduino.h>
#include "imu.h"
#include "config.h"

bool selectIMU(int port);
bool initIMU();
bool startIMUs();
void scanI2C();

bool setupIMUConnection()
{
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with custom SDA and SCL pins
    delay(500);

    /*
    testing whether Devices are recognized after selecting channel on tca
    scanI2C();
    selectIMU(0);
    scanI2C();
    return false;
    */

    // If IMUs couldn't be initialized pause for 500 ms and try again
    while (!startIMUs())
    {
        delay(500);
    }
    return true;
}

// For testing only
void scanI2C()
{
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
        }
    }
}

bool restartIMU(int port)
{
    selectIMU(port);
    delay(10);
    Serial.println("restarting: " + String(port));
    return initIMU();
}

bool startIMUs()
{
    // Initialize all IMUs
    for (int i = 0; i < NUMBER_IMUS; i++)
    {
        if (!restartIMU(i))
        {
            Serial.println("Sensor " + String(i) + " didn't connect");
            return false;
        }
        delay(IMU_INIT_DELAY);
    }
    return true;
}

bool read(uint8_t *pBuffer, int registerAddress, int numberBytes)
{
    // Make sure to call read() with an array big enough to store number of Bytes
    // read() reads sequential bytes
    // opening I2C Channel to IMU
    Wire.beginTransmission(IMU_SPI_ADDRESS);

    // Selecting register Adress from which you want to read
    Wire.write(registerAddress);
    if (Wire.endTransmission(false))
    { // endTransmission(false) tells I2C that we want the connection to stay open to receive data
        Serial.print("Transmission failed");
        return false;
    }

    // Check whether enough bytes are being sent
    uint8_t byteCount = Wire.requestFrom(IMU_SPI_ADDRESS, numberBytes); // We request the next n bytes
    if (byteCount < numberBytes)
    {
        Serial.println("Not enough Bytes");
        Serial.println("requested: " + String(numberBytes) + "got: " + String(byteCount));
        return false;
    }

    // Read Bytes
    for (int i = 0; i < numberBytes && Wire.available(); i++)
    {
        pBuffer[i] = Wire.read();
    }
    return true;
}

bool read8(uint8_t *pBuffer, int registerAddress)
{
    // Loads 8 bytes
    // Make sure to call read8() with at least an 8 byte array
    return read(pBuffer, registerAddress, 8);
}

bool read1(uint8_t &buffer, int registerAddress)
{
    // Used to load only 1 byte
    return read(&buffer, registerAddress, 1);
}

bool write1(int registerAddress, uint8_t value)
{
    // Writes 1 byte
    // Opening I2C Channel to IMU
    Wire.beginTransmission(IMU_SPI_ADDRESS);

    // First write() after beginTransmission sets the register you want to read/write to
    Wire.write(registerAddress);
    // Second write() writes value into selected register
    Wire.write(value);

    return !Wire.endTransmission();
}

bool selectIMU(int port)
{
    // Check whether port is reachable
    if (port > 7)
    {
        return false;
    }
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(1 << port);
    int transmissionID = Wire.endTransmission();
    if (transmissionID)
    {
        Serial.println("Selection Error: " + String(transmissionID));
        return false;
    }
    return true;
}

bool setSysTrigger()
{
    delay(IMU_INIT_DELAY);
    return write1(IMU_SYS_TRIGGER_ADDRESS, IMU_SYS_TRIGGER);
}

bool setPowerMode()
{
    delay(IMU_INIT_DELAY);
    return write1(IMU_POWER_MODE_ADDRESS, IMU_POWER_MODE);
}

bool setPageID()
{
    delay(IMU_INIT_DELAY);
    return write1(IMU_PAGE_ID_ADDRESS, IMU_PAGE_ID);
}

bool checkChipID()
{
    delay(IMU_INIT_DELAY);
    uint8_t chip_id;
    if (read1(chip_id, IMU_CHIP_ID_ADDRESS) && chip_id == IMU_CHIP_ID)
    {
        return true;
    }
    return false;
}

bool initIMU()
{
    // Setting Operation mode to config
    write1(IMU_OPERATION_MODE_ADDRESS, IMU_OPERATION_MODE_CONFIG);

    // Setting Page ID to 0x00
    if (!setPageID())
    {
        Serial.println("Page ID couldn't been set");
        return false;
    }

    // Checking Chip ID (should be 0xA0)
    if (!checkChipID())
    {
        Serial.println("Chip ID doesn't match");
        return false;
    }

    // Setting Power mode to 0x00 (normal)
    if (!setPowerMode())
    {
        Serial.println("Couldn't set power mode");
        return false;
    }

    // Setting Sys Trigger to 0x80 (external clock)
    if (!setSysTrigger())
    {
        Serial.println("Couldn't set clock to external");
        return false;
    }

    delay(IMU_INIT_DELAY);
    // Setting Operation mode to NDOF
    write1(IMU_OPERATION_MODE_ADDRESS, IMU_OPERATION_MODE_NDOF);
    return true;
}

bool allEqual(uint8_t *pArray)
{
    // Used to check for zeroes in quat data
    for (int i = 0; i < 8; i++)
    {
        if (pArray[i] != pArray[0])
        {
            return false;
        }
    }
    return true;
}

bool readIMUData(uint8_t *pBuffer)
{
    // Quaternion Data is seperated into w, x, y, z (stored in this order)
    // each of these values is stored within 2 bytes
    // lower byte in the first register
    // higher byte in the second register
    // Quaternion Data begins in Register 0x20
    if (read8(pBuffer, IMU_QUAT_DATA_ADDRESS))
    {
        // checking for zeroes
        return !allEqual(pBuffer);
    }
    return false;
}

bool loadData(uint8_t pQuatData[NUMBER_IMUS][8])
{
    // Loads Quaternion from each IMU into the 2d Array
    for (int i = 0; i < NUMBER_IMUS; i++)
    {
        selectIMU(i);
        delay(10);
        if (!readIMUData(pQuatData[i]))
        {
            Serial.println("no data: " + String(i));
            restartIMU(i);
            return false;
        }
    }
    return true;
}