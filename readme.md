# Welcome to DC Motor and L298N Motorcontroller, AS5600 Rotary Controller with Micropython

[Hardware](#hardware)  
[How it works](#how_it_works)  
[3D printing parts - CAD files](#3D_printing_parts_CAD_files)  
[TODO](#todo)

![image1](dc_motor_micropython.jpg)


# Hardware
- DC Motor
- 3D printing mounts for:
  - AS5600
  - DC Motor
- L298N Motor driver
- AS5600 rotary encoder
- ESP32 S2 mini
- Breadboard + some cables
- 9 V / 12 V power supply for the DC Motor

# How it works

- DC Motor driver as class
- AS5600 class
- Motor controller class
- initial min_duty at the beginning of the program
- set the motor to for example: 50 rpm or 5 rounds then stop
  
- Motor turn: left / right / stop with speed 100 / -100 / 0
- RPM messurement over angle measurement to different times

# 3D printing parts - CAD files

AS5600_brick_mount
dc_motor_brick_mount

# TODO
finish coding  
present some results