#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <BleMouse.h>

Adafruit_MPU6050 mpu;
Madgwick filter;
BleMouse bleMouse("GyroMouse", "ESP32", 100);

float roll, pitch, yaw;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.begin()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    Serial.println("MPU6050 connected!");
    filter.begin(100);  // Set filter update rate to 100Hz
    bleMouse.begin();
}

void loop() {
    if (bleMouse.isConnected()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Convert gyro data to deg/sec
        float gx_r = g.gyro.x * 180 / PI;
        float gy_r = g.gyro.y * 180 / PI;
        float gz_r = g.gyro.z * 180 / PI;
        
        // Convert accelerometer data to g
        float ax_r = a.acceleration.x / 9.81;
        float ay_r = a.acceleration.y / 9.81;
        float az_r = a.acceleration.z / 9.81;
        
        // Update filter with new IMU data
        filter.updateIMU(gx_r, gy_r, gz_r, ax_r, ay_r, az_r);
        
        // Get orientation
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();
        
        // Convert orientation to mouse movement
        int dx = map(yaw, -90, 90, -10, 10);  // Adjust sensitivity as needed
        int dy = map(pitch, -90, 90, -10, 10);
        
        // Send mouse movement via BLE
        bleMouse.move(dx, dy);
    }
    
    delay(10);  // Small delay for stability
}