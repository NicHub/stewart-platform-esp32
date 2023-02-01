# STEWART PLATFORM ON ESP32

<a href="https://archiveprogram.github.com/arctic-vault/">
<table style="border:none">
<tr>
<td>
<img style="width:64px; float:right; margin-right:10px;" src="https://github.githubassets.com/images/modules/profile/achievements/arctic-code-vault-contributor-default.png" />
</td>
<td>
This repository<br/>is archived in the<br/><em>GitHub Arctic Code Vault</em>.
</td>
</tr>
</table>
</a>

<p align="center">
<img height=400px src="https://raw.githubusercontent.com/NicHub/stewart-platform-esp32/master/doc/stewart-plateform-ouilogique.jpg" />
</p>

## ABSTRACT

This is an ESP32 implementation of a six-degree-of-freedom hexapod — also called _Stewart Platform_, _Gough-Stewart Platform_ or _Parallel manipulator_.
The actuators are PWM rotary servo motors and the program is written in _Arduino C_ compiled with [_PlatformIO_](https://platformio.org/).
This project is done in the frame of the _P19 project_ at the [Microclub](https://microclub.ch).

Currently the platform can be operated:

-   With a _Nunchuck_.
-   With _Arduino C_ code (see [`Hexapod_Demo.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Demo.cpp)).
-   With _G-Code_ sent by a computer to the ESP through the serial port (see [`python/gcode2serial.py`](https://github.com/NicHub/stewart-platform-esp32/blob/master/gcode2serial/gcode2serial.py)).

The calculation of the kinematics is performed in [`Hexapod_Kinematics.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Kinematics.cpp) and can be also be compiled in a desktop C++ program to cross-check calculations (i.e. without actuating the servos).
I managed to compile it with `g++` on _macOS Mojave_ and _macOS Monterey_.
See [`hexapod_desktop_app/hexapod_desktop_app.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/hexapod_desktop_app/hexapod_desktop_app.cpp).

## HOME PAGE OF THE PROJECT

The home page is a work in progress, but the video shows what the platform can do.
The original implementation used an analog joystick, but the current version uses a Wii Nunchuck.

<https://ouilogique.com/plateforme-de-stewart-esp32/>

## GEOMETRY SETTINGS

Geometry settings are defined in
[`Hexapod_Config_1.h`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Config_1.h).

The meaning of the parameters is explained in
[`doc/hexapod-parameters.pdf`](https://github.com/NicHub/stewart-platform-esp32/blob/master/doc/hexapod-parameters.pdf).

The two base plates are identical and the DXF file to reproduce them is
in [`doc/hexapod-base-plate.dxf`](https://raw.githubusercontent.com/NicHub/stewart-platform-esp32/master/doc/hexapod-base-plate.dxf).

<p align="center">
<img height=400px src="https://raw.githubusercontent.com/NicHub/stewart-platform-esp32/master/doc/hexapod-base-plate.png" />
</p>

## PREREQUISITES

Create a file called `src/WifiSettings.h` containing:

```C++
#pragma once

const char *ssid = "";            // SSID of your WiFi router.
const char *password = "";        // Password of your WiFi router.
const char *ap_ssid = "STEW32-";  // SSID of the ESP32 WiFi network in soft-AP mode (15 char max).
const char *ap_password = "";     // Password of the ESP32 WiFi network in soft-AP mode.
                                  // Must be 8 char min or empty for no password.
```

## COMPONENTS & WIRING

### ESP32

WeMos ESP32 WROOM <https://www.banggood.com/fr/WeMos-ESP32-WiFi-Bluetooth-Development-Board-Ultra-Low-Power-Consumption-Dual-Core-ESP-32-ESP-32S-p-1175488.html>

### PCA9685

<https://www.mouser.ch/ProductDetail/adafruit/3416/?qs=F5EMLAvA7ICYzX4Av%252bhRHw==>

### External power supply

5 V / 10 A power supply. <https://aliexpress.com/af/32810906485.html>

### Rods

M3x100mm (140mm total) <https://aliexpress.com/af/32775630549.html>

### Servo horn arm

Tritanium color <https://aliexpress.com/af/32843432977.html>

### Servos

-   I currently use clones of the _Tower Pro MG996R_ servos, but they are bad and I don’t recommend them.
    <https://fr.aliexpress.com/item//32636102294.html>
-   I formerly used clones of the _Tower Pro MG90s_ Servos, but they were also bad and too small for this application.
    <https://www.banggood.com/6X-Towerpro-MG90S-Metal-Gear-RC-Micro-Servo-p-1072260.html>
-   In the future, I will probalbly use _Parallax 900-00005_ servos.
    It seems that these are the one used by _fullmotiondynamics_ in their videos.
    <https://www.parallax.com/product/900-00005>

> Pins are defined in [`Hexapod_Config_1.h`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Config_1.h).

| servo | ESP32 pin |
| :---- | :-------- |
| 0     | 13        |
| 1     | 15        |
| 2     | 27        |
| 3     | 14        |
| 4     | 33        |
| 5     | 25        |

### Nunchuck

-   Nunchuck <https://fr.aliexpress.com/item//32880983134.html>
-   Cable extension <https://fr.aliexpress.com/item//32841281892.html>

> The Nunchuck library uses `Wire.h` and standard I²C connections internaly.
>
> Pins are seen from left to right and top to bottom looking at the Nunchuck connector.
>
> Cut the cable extension to connect to the ESP32.

| color  | signal | ESP32 pin                                              |
| :----- | :----- | :----------------------------------------------------- |
| white  | SCL    | IO22                                                   |
| NC     |        |                                                        |
| red    | GND    | GND                                                    |
| green  | VCC    | VCC                                                    |
| black  | ATT    | IO4 (Not required, gives VCC if Nunchuck is connected) |
| yellow | SDA    | IO21                                                   |

## EXTERNAL LIBRARIES

The external libraries are installed automatically during the first build because they are declared under `lib_deps` in `platformio.ini`.

## I²C ADDRESSES

| Default address | New address | Device              |
| :-------------- | :---------- | :------------------ |
| 0x40            | 0x41        | PCA9685             |
| 0x70            | =           | PCA9685 (broadcast) |
| 0x40            | =           | Nunchuck (accel)    |
| 0x52            | =           | Nunchuck (joystick) |
| 0x68            | =           | GY-91 (MPU9250)     |
| 0x76            | =           | GY-91 (BMP280)      |

## CREDITS

### My sources of inspiration

-   San-José State University / Full Motion Dynamics:
    -   <https://www.youtube.com/watch?v=j4OmVLc_oDw>
    -   ~~fullmotiondynamics.com~~

### Kinematics

-   Hexapod kinematics of this project was originaly based on _6dof-stewduino, by Philippe Desrosiers_, althought I reworked it in depth:
    -   <https://github.com/xoxota99/stewy>
-   He derived his implementation from the work of _Daniel Waters_:
    -   <https://www.youtube.com/watch?v=1jrP3_1ML9M>
-   Kinematics calculation is also explained in this document by an unknown author from the _Wokingham U3A Math Group_:
    -   <https://memememememememe.me/assets/posts/stewart-platform-math/MathsOfStewartPlatformV5.pdf>
    -   <http://www.wokinghamu3a.org.uk/groups/mathematics/>
-   The project _memememememememe_ was also an excellent source of inspiration.
    They share the code for _RPi_ and a simulator in the _Processing_ langage:
    -   <https://memememememememe.me/post/stewart-platform-math/>
    -   <https://github.com/thiagohersan/memememe>

### Serial buffer for G-Code

-   Derived from _MarginallyClever_:
    -   <https://github.com/MarginallyClever/GcodeCNCDemo/tree/master/GcodeCNCDemo4AxisCNCShield>

## FURTHER READINGS

-   PID Control System Analysis and Design, By YUN LI, KIAM HEONG ANG, and GREGORY C.Y. CHONG
    -   See Table 1, p.33 <http://eprints.gla.ac.uk/3815/1/IEEE_CS_PID_01580152.pdf>
-   Understanding PID Control, Part 1: What is PID Control?
    -   <https://www.youtube.com/watch?v=wkfEZmsQqiA>
-   Modern Robotics, Chapter 7: Kinematics of Closed Chains
    -   <https://youtu.be/5wCK6XGC3ig>
