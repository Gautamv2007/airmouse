AirMouse: The Next Generation of Input Devices

The AirMouse is a next-generation input device designed to replace conventional mice, which require a surface to function. Instead, it uses motion sensing to provide a seamless, more intuitive experience.

This project utilizes the MPU6050 sensor to calculate rotation angles and rotational speed using its built-in gyroscope and accelerometer.
Required Libraries

To implement this project, the following libraries are needed:

1. ESP32-BLE-Mouse - GitHub Link
2. Adafruit MPU6050 - GitHub Link
3. Adafruit BusIO (Dependency) - GitHub Link
4. Adafruit Unified Sensor Driver (Dependency) - GitHub Link

The first two libraries are directly included in the code, while the latter two are required dependencies.
Enhancements & Features

1. Kalman Filtering: We have implemented Kalman filtering to smooth noisy sensor data, enhancing accuracy by        predicting and correcting measurements.

2. Button Controls: The device features four buttons for left click, right click, scroll up, and scroll down, making navigation effortless.

3. Rechargeable Battery: A TP4056 lithium battery charging module has been integrated to enable convenient recharging, making it a fully standalone product.

This innovation eliminates the need for a physical surface, offering a futuristic alternative to traditional mice.