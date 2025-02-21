#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
    Serial.begin(115200);
    if (!bno.begin()) {
        Serial.println("No BNO055 detected, check wiring or I2C ADDR!");
        while (1); // Stop execution if the sensor is not detected
    }
    delay(1000);
    
    bno.setExtCrystalUse(true); // Use external crystal for better accuracy

    Serial.println("Setting operation mode to NDOF...");
    bno.setMode(0x0C);

    // Commented properly to avoid syntax errors
    /*
    Serial.println("Place the sensor in 6 different stable positions for calibration.");
    while (!bno.isFullyCalibrated()) {
        delay(500);
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIBRATION: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);
    }
    */
}

void loop() {
    delay(500);

    // Check calibration status
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    // Check sensor status
    uint8_t system_status, self_test_result, system_error;
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);

    Serial.print("System Status: ");
    Serial.println(system_status);
    Serial.print("Self Test Result: ");
    Serial.println(self_test_result);  // 0x0F means all good
    Serial.print("System Error: ");
    Serial.println(system_error);

    // Get Euler angles
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.println(euler.z());

    delay(100);
}