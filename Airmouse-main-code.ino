#include <BleMouse.h>
#include <Adafruit_MPU6050.h>

#define LEFTBUTTON 19
#define RIGHTBUTTON 18
#define SCROLLBUTTON 25
#define SPEED 10

Adafruit_MPU6050 mpu;
BleMouse bleMouse;

bool sleepMPU = true;

void setup() {
  Serial.begin(115200);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);
  pinMode(SCROLLBUTTON, INPUT_PULLUP);

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

    static unsigned long lastReadTime = 0;
    sensors_event_t a, g, temp;
    if (millis() - lastReadTime > 50) {  // Read sensor data every 50ms
      lastReadTime = millis();
      mpu.getEvent(&a, &g, &temp);
    }

    bleMouse.move(g.gyro.z * -SPEED, g.gyro.x * -SPEED);

    if (!digitalRead(LEFTBUTTON)) {
      Serial.println("Left click");
      bleMouse.click(MOUSE_LEFT);
      delay(150);  // Reduced delay for better response
    }

    if (!digitalRead(RIGHTBUTTON)) {
      Serial.println("Right click");
      bleMouse.click(MOUSE_RIGHT);
      delay(150);  // Reduced delay for better response
    }

    if (!digitalRead(SCROLLBUTTON) && g.gyro.y > 3) {  // Higher threshold for natural scrolling
      Serial.println("Scroll down");
      bleMouse.move(0, 0, -2);
      delay(100);  // Small delay for better scrolling experience
    }

    if (!digitalRead(SCROLLBUTTON) && g.gyro.y < -3) {
      Serial.println("Scroll up");
      bleMouse.move(0, 0, 2);
      delay(100);
    }
  }
}
