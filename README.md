# stm32_quadcopter-quadroid-
quadcopter flight controller project based on stm32f103 microcontroller.this flight controller can be controlled by android application.thus ESP8266 wifi module are used to communicate contoller and application.
The goal of this project is to design a semi-autonomous quad copter for aerial photography which is capable of self-sustained flight via wireless communication through wi-fi utilizing a microcontroller. The quad copter was designed to be small enough so that costs would be minimized, hence small motor and propellers are being used, while an ARM Cortex M3 32bit controller, accelerometer, altitude sensor, and gyroscope are communicating each other to maintain the stability. The quad copter is controlled by an android device. The system uses the wi-fi capability of the android device for this. An intuitive application in the android device makes it easier to control the quad copter. 

MCU      -  STM32F103
acc&gyro -  MPU6050
ALT      -  BMP180
WIFI     -  ESP8266

battery  -  11v LIPO 1500mAh
frame    -  zmr250
bldc     -  2400kv
prop     -  5030
