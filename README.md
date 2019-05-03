
# S T E W A R T    P L A T F O R M    O N    E S P 3 2

<p align="center">
<img height=400px src="https://raw.githubusercontent.com/NicHub/stewart-platform-esp32/master/doc/stewart-plateform-ouilogique.jpg" />
</p>

## ABSTRACT

This is an implementation of a 6-degrees of freedom hexapod — also called *Stewart Platform* — on the ESP32. The actuators are PWM rotary servo motors and the program is written in *Arduino C* for *Platform IO*. This project is done in the frame of the *P19 project* at the [Microclub](https://microclub.ch).

Currently the platform can be operated:

 - with a *Nunchuck*
 - with *Arduino C* code (see [`Hexapod_Demo.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Demo.cpp))
 - with *G-Code* through serial (see [`python/gcode2serial.py`](https://github.com/NicHub/stewart-platform-esp32/blob/master/python/gcode2serial.py))

The kinematics calculation is done in [`Hexapod_Kinematics.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Kinematics.cpp) and can be also be compiled in a desktop C++ program to cross check calculations (i.e. without actuating the servos). I managed to compile it with `g++` on *macOS Mojave*. See [`hexapod_app/hexapod_app.cpp`](https://github.com/NicHub/stewart-platform-esp32/blob/master/hexapod_app/hexapod_app.cpp).

## HOME PAGE OF THE PROJECT

The home page is a work in progress, but the video shows what the platform can do. The original implementation used an analog joystick, but the current version uses a Wii Nunchuck.

<https://ouilogique.com/plateforme-de-stewart-esp32/>

## GEOMETRY SETTINGS

Geometry settings are defined in [`Hexapod_Config_1.h`](https://github.com/NicHub/stewart-platform-esp32/blob/master/src/Hexapod_Config_1.h). The meaning of the parameters is also explained in [`doc/hexapod-parameters.pdf`](https://github.com/NicHub/stewart-platform-esp32/blob/master/doc/hexapod-parameters.pdf).

## COMPONANTS & WIRING

### ESP32

WeMos ESP32 WROOM <https://www.banggood.com/fr/WeMos-ESP32-WiFi-Bluetooth-Development-Board-Ultra-Low-Power-Consumption-Dual-Core-ESP-32-ESP-32S-p-1175488.html>

### External power supply

I currently use a 5 V / 10 A power supply, but 5 V is not enough. I need to upgrade to 7 V. <https://aliexpress.com/af/32810906485.html>

### Rods

M3x100mm (140mm total) <https://aliexpress.com/af/32775630549.html>

### Servo horn arm

Tritanium color <https://aliexpress.com/af/32843432977.html>

### Servos

 - I currently use clones of the *Tower Pro MG996R* servos, but they are bad and I don’t recommend them. <https://fr.aliexpress.com/item//32636102294.html>
 - I formerly used clones of the *Tower Pro MG90s* Servos, but they were also bad and too small for this application. <https://www.banggood.com/6X-Towerpro-MG90S-Metal-Gear-RC-Micro-Servo-p-1072260.html>
 - In the future, I will probalbly use *Parallax 900-00005* servos. It seems that these are the one used by *fullmotiondynamics* in their videos. <https://www.parallax.com/product/900-00005>

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

 - Nunchuck <https://fr.aliexpress.com/item//32880983134.html>
 - Cable extension <https://fr.aliexpress.com/item//32841281892.html>

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

    platformio lib install 4744 # ESP32Servo
    platformio lib install 1465 # WiiChuck

## CREDITS

### Primary source of inspiration

 - San-José State University / Full Motion Dynamics:
    - <https://www.youtube.com/watch?v=j4OmVLc_oDw>
    - <http://fullmotiondynamics.com>

### Kinematics

 - Hexapod kinematics of this project was originaly based on *6dof-stewduino, by Philippe Desrosiers*, althought I reworked it in depth:
    - <https://github.com/xoxota99/stewy>
 - He derived his implementation from the work of *Daniel Waters*:
    - <https://www.youtube.com/watch?v=1jrP3_1ML9M>
 - Kinematics calculation is also explained in this document by an unknown author from the *Wokingham U3A Math Group*:
    - <https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf>
    - <http://www.wokinghamu3a.org.uk/groups/mathematics/>
 - The project *memememememememe* was also an excellent source of inspiration. They share the code for *RPi* and a simulator in the *Processing* langage:
    - <https://memememememememe.me/post/stewart-platform-math/>
    - <https://github.com/thiagohersan/memememe>

### Serial buffer for G-Code

 - Derived from *MarginallyClever*:
   - <https://github.com/MarginallyClever/GcodeCNCDemo/tree/master/GcodeCNCDemo4AxisCNCShield>
