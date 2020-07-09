# ESP32_Robot
This is ESP32 Based Balancing Robot
This is a fork of the "High speed balancing robot based on ESP32" created by Wouter Klop,
Original code is here:
https://gitlab.com/kloppertje/balancingrobot

The article that explains how this works is here:
http://elexperiment.nl/2018/11/high-speed-balancing-robot-introduction/

This fork contains several changes, one of them is a simple ability to change the direction of the motors individually:
```cpp
#define reverseLeftMotor true
#define reverseRightMotor false
```
Another change is the auto current control.

yet another noticible change is the use of two sets of PID valus for "satading" and for "going", this allowed the robot to stay without any movement. (I was not able to achive standing without movement only using one set of PID values).
Set this line to: 0 for automatic switching

```cpp
uint8_t pidTypeApply = 1; // PID to use 0=auto, 1= go PIDs, 2= stay PIDs
```


This is the curcuit:
<img src="circuit.jpg">


