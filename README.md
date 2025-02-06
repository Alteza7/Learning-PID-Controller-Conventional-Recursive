# PID Learning STM32CubeIDE
My PID Learning

## STM32F103C8T6 PID Controller Project

This project is for learning purposes, focusing on controlling a DC motor's speed using a PID controller.
It includes both recursive and conventional PID, with an encoder for speed measurement.

## Features:
PID Control: Implements both recursive and conventional PID controllers.
Encoder-Based Speed Measurement: Calculates RPM based on encoder pulses.
Configuration:
PID Parameters

Modify the following values in main.c:
```
float Kp = 1.0; // Proportional gain
float Ki = 0.5; // Integral gain
float Kd = 0.1; // Derivative gain
```

Encoder Setup

Adjust PPR (Pulses Per Revolution) in main.c:
```
int16_t PPR = 100;
```
