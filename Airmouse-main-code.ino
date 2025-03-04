#include <BleMouse.h>
#include <Adafruit_MPU6050.h>

#define LEFTBUTTON 19
#define RIGHTBUTTON 18
#define SPEED 15

Adafruit_MPU6050 mpu;
BleMouse bleMouse;

bool sleepMPU = true;
bool dragging = false;

float angleX = 0, angleZ = 0;
float biasX = 0, biasZ = 0;
float previousAngleX = 0, previousAngleZ = 0;

float P[2][2] = {{1, 0}, {0, 1}};

const float dt = 0.01;
const float R = 0.01;
const float Q = 0.005;
const float emaAlpha = 0.3;

unsigned long lastLeftPress = 0;
bool leftPressed = false;
bool rightPressed = false;

void setup() {
  Serial.begin(115200);
  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);

  bleMouse.begin();
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  mpu.enableSleep(sleepMPU);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

float kalmanUpdate(float angle, float bias, float rate, float measurement) {
  angle += (rate - bias) * dt;
  P[0][0] += Q;
  P[1][1] += Q;

  float S = P[0][0] + R;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  float y = measurement - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  P[0][0] -= K[0] * P[0][0];
  P[1][0] -= K[1] * P[0][0];

  return angle;
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

    float gyroX = g.gyro.x;
    float gyroZ = g.gyro.z;

    angleX = kalmanUpdate(angleX, biasX, gyroX, g.gyro.x);
    angleZ = kalmanUpdate(angleZ, biasZ, gyroZ, g.gyro.z);

    angleX = emaAlpha * (previousAngleX + gyroX * dt) + (1 - emaAlpha) * previousAngleX;
    angleZ = emaAlpha * (previousAngleZ + gyroZ * dt) + (1 - emaAlpha) * previousAngleZ;
    previousAngleX = angleX;
    previousAngleZ = angleZ;

    if (abs(angleX * SPEED) >= 1 || abs(angleZ * SPEED) >= 1) {
      bleMouse.move(angleZ * -SPEED, angleX * -SPEED);
    }

    if (!digitalRead(LEFTBUTTON)) {
      if (!leftPressed) {
        leftPressed = true;
        lastLeftPress = millis();
      }
      if (millis() - lastLeftPress > 500) {
        if (!dragging) {
          Serial.println("Start Dragging");
          bleMouse.press(MOUSE_LEFT);
          dragging = true;
        }
      }
    } else {
      if (leftPressed) {
        if (!dragging) {
          Serial.println("Left Click");
          bleMouse.click(MOUSE_LEFT);
        } else {
          Serial.println("Stop Dragging");
          bleMouse.release(MOUSE_LEFT);
          dragging = false;
        }
      }
      leftPressed = false;
    }

    if (!digitalRead(RIGHTBUTTON)) {
      if (!rightPressed) {
        rightPressed = true;
        Serial.println("Right Click");
        bleMouse.click(MOUSE_RIGHT);
      }
    } else {
      rightPressed = false;
    }

    delay(5);
  }
}
