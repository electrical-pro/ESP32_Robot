# This is ESP32 Based Balancing Robot

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
Another change is the auto current control. Allows ESP32 to set current through the motors.
This feature requires you to remove trimmer potentiometer on the DRV8825 module and solder a wire (see the circuit)

Yet another noticeable change is the use of two sets of PID values for "standing" and for "going", this allowed the robot to stay without any movement. (I was not able to achieve perfect standing without movement just with one set of PID values).

Set this to "0" for automatic switching (Experimental)

```cpp
uint8_t pidTypeApply = 1; // PID to use 0=auto, 1= go PIDs, 2= stay PIDs
```
Both PID value needs to be tuned separately which is "additional headache", so at first, is the best just set it to "1" and use one set of PIDs.
Not sure that I implemented this feature correctly, but I am satisfied with the results.

Also I added INA219 module to monitor current and voltage. Good for telemetry.
Also when the charger is connected it measures the capacity of the battery.
INA219 uses I2C bus just like MPU6050

There are many small tweaks.

This is the curcuit:
<img src="circuit.jpg">

I must warn you that I am not a professional programmer, so there are probably much smarter ways to make all these changes. But this works for me and I am happy with the results. The great thing about open source is that you can change the code the way you like it.
Thanks Wouter Klop for making this possible.


