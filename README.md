
# 📦 Firmware for BNO055 with Multiplexer

## 📐 IMU Data Format (Quaternion)

This firmware reads data from multiple BNO055 IMUs and transmits it via BLE using the following format:

### 🧊 Quaternion Data Structure

Each quaternion consists of **four components**:  
**(w, x, y, z)**  
Each component is stored in **2 bytes**, totaling **8 bytes per IMU**.

- **Byte Order:** Little Endian  
  → **LSB first, then MSB** for each value.

### 📤 BLE Packet Structure (ESP32-C3)

The ESP32-C3 sends a **48-byte array** over BLE, containing quaternion data from **6 IMUs**:

| Bytes         | Content         | IMU #  |
|---------------|------------------|--------|
| `Array[0–7]`   | Quaternion wxyz   | IMU 1  |
| `Array[8–15]`  | Quaternion wxyz   | IMU 2  |
| `Array[16–23]` | Quaternion wxyz   | IMU 3  |
| `Array[24–31]` | Quaternion wxyz   | IMU 4  |
| `Array[32–39]` | Quaternion wxyz   | IMU 5  |
| `Array[40–47]` | Quaternion wxyz   | IMU 6  |

### 🧮 Example (1 Quaternion – 8 Bytes)

| Byte Index | Description      |
|------------|------------------|
| `0–1`      | w (LSB, MSB)     |
| `2–3`      | x (LSB, MSB)     |
| `4–5`      | y (LSB, MSB)     |
| `6–7`      | z (LSB, MSB)     |

---
