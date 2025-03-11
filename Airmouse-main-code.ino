#include <BleMouse.h>
#include <Adafruit_MPU6050.h>

#define LEFTBUTTON 19
#define RIGHTBUTTON 18
#define UPBUTTON 21
#define DOWNBUTTON 22
#define SPEED 10
#define DRAG_THRESHOLD 1000 // Time in milliseconds to trigger drag

Adafruit_MPU6050 mpu;
BleMouse bleMouse;

bool sleepMPU = true;
long leftButtonPressedTime = 0;
bool isDragging = false;

// Kalman filter variables
float estimatedX = 0, estimatedZ = 0;
float errorCovX = 1, errorCovZ = 1;
float processNoise = 0.1; // Adjust for smoothness
float measurementNoise = 0.5; // Adjust for sensitivity

// Kalman filter function
float kalmanFilter(float measurement, float &estimate, float &errorCov) {
  errorCov += processNoise;
  float kalmanGain = errorCov / (errorCov + measurementNoise);
  estimate = estimate + kalmanGain * (measurement - estimate);
  errorCov = (1 - kalmanGain) * errorCov;
  return estimate;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);
  pinMode(UPBUTTON, INPUT_PULLUP);
  pinMode(DOWNBUTTON, INPUT_PULLUP);
  
  bleMouse.begin();

  delay(1000);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  mpu.enableSleep(sleepMPU);
}

void loop() {
  if (bleMouse.isConnected()) {
    if (sleepMPU) {
      delay(3000);
      Serial.println("MPU6050 awakened!");
      sleepMPU = false;
      mpu.enableSleep(sleepMPU);
      delay(500);
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gyroX = g.gyro.x * -SPEED;
    float gyroZ = g.gyro.z * -SPEED;

    float filteredX = kalmanFilter(gyroX, estimatedX, errorCovX);
    float filteredZ = kalmanFilter(gyroZ, estimatedZ, errorCovZ);

    bleMouse.move(filteredZ, filteredX);

    // Left Click / Drag Logic
    if (!digitalRead(LEFTBUTTON)) {
      if (leftButtonPressedTime == 0) {
        leftButtonPressedTime = millis();
      }
      if (!isDragging && millis() - leftButtonPressedTime > DRAG_THRESHOLD) {
        Serial.println("Dragging started");
        bleMouse.press(MOUSE_LEFT);
        isDragging = true;
      }
    } else {
      if (isDragging) {
        Serial.println("Dragging stopped");
        bleMouse.release(MOUSE_LEFT);
        isDragging = false;
      } else if (leftButtonPressedTime > 0) {
        Serial.println("Left click");
        bleMouse.click(MOUSE_LEFT);
      }
      leftButtonPressedTime = 0;
    }

    // Right Click Logic
    if (!digitalRead(RIGHTBUTTON)) {
      Serial.println("Right click");
      bleMouse.click(MOUSE_RIGHT);
      delay(500);
    }

    // Scroll Up Logic
    if (!digitalRead(UPBUTTON)) {
      Serial.println("Scrolling up");
      bleMouse.move(0, 0, 1); // Scroll up
      delay(300); // Debounce delay
    }

    // Scroll Down Logic
    if (!digitalRead(DOWNBUTTON)) {
      Serial.println("Scrolling down");
      bleMouse.move(0, 0, -1); // Scroll down
      delay(300); // Debounce delay
    }
  }
}
