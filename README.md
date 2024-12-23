# TinyBot-Legged

This is a 4 legged robot with micro servos. The servos used in this robot are TowerPro SG90 9g Micro servos.

![alt text](https://github.com/ahmetakif/TinyBot-Legged/blob/master/image.png?raw=true)

Mechanical design:
- This robot is designed in SolidWorks to be 3D printed with PLA material.
- 23 pieces can be 3D printed to complete this robot.
- All of the parts can be 3D printed with 20% infill.
- Support material can be used if the slicing software suggests. (I am using CURA)
- Parts can be printed with rafts if there is an adhesion issue.

Electronics:
- Raspberry Pi Zero W
- Arduino Nano (Connected to the Raspberry Pi Zero via the USB port.)
- MPU6050 IMU Gyro & Acceleration sensor module
- 12x TowerPro SG90 9g Micro servos
- 2x 3000mAh 3.85v Li-ion smartphone (LG G4) batteries. (One of the batteries power the Arduino and servos and the other one powers the Raspberry Pi.)
- Usb 5v step-up voltage convertor

Software:
- The arduino gets command from the raspberry pi via the usb serial communication port.
- The arduino handles the inverse kinematics calculations.
- Raspberry pi controls the robot with an appropriate gait function and bezier curve.
- Raspberry pi also controls the robot with the help of the IMU sensor data which it receives from the arduino.
- The python script in the raspberry pi also controls the PID balance algorithm.

Note: This project is work in progress. There will be upgrades both with the mechanical design and the software.
