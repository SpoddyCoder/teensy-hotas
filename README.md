# HOTAS USB Joystick Controller for Teensy

8 axes, 16 button, 4 hat switch joystick controller for Teensy microcontroller for use with "Hands On Throttle And Stick" input devices.

Originally conceived to replace the MCU on a Thrustmaster HOTAS X, 
but should work with any hardware capable of delivering analog signals for the axes and digital signals for the switches.

Based on an original 2016 mod by Pat Daderko, building upon Paul Stoffregen's Extreme Joystick Test.
Updated and expanded for 2023 by SpoddyCoder.

## Features

* 8 analogue axis inputs
    * X, Y, Z
    * RX, RY, RZ
    * S0, S1
    * Configurable deadzones for each
* 4 hat-switch inputs
    * Up, Down, Left, Right
* 16 button inputs
* Switchable throttle control mode
    * Standard 0-100%
        * For standard flight sims
    * Centre zero
        * Optional deadzone for the mid throttle position
        * For space games like Elite
* Throttle power indiactor
    * Red = full throttle
    * Green = zero throttle
    * Orange = 50% throttle (centre zero mode enabled)
* Configurable filter type for the analog inputs
    * No filter
    * Simple moving average
    * One euro filter
    * Kalman filter
    * Each has configurable paramaters to balance smoothing against input lag
* Optional serial debug

## Pre-requisites

* Teensy 3.1 or 3.2
    * It should be easy to modify the board files for Teensy 4. PR's welcome.
* Flight stick hardware
    * See `teensy-hotas.ino` for Teensy pin numbers
    * See reference section below for a full tutorial using a Thrustmaster HOTAS X
* Arduino IDE with Teensyduino installed
    * https://www.pjrc.com/teensy/td_download.html
* Tested on
    * Windows 11, Arduino IDE v2.1.0, Teensy 3.2

## Install

* Replace the teensy3 board files with the modified ones, which add the Hotas X USB interface
    * Copy `board_files` to the location of the teensy3 board files on your machine (see Help)
    * You may want to backup the existing files before overwriting them
    * Diff between original files and modified...
        * https://github.com/SpoddyCoder/teensy-hotas/commit/bcc00d9c30cdf59a3ea81ee41fbe30af6508d4a3
* Open `teensy-hotas.ino` in the Arduino IDE
* Select the targets;
    * Tools -> Board -> Teensy 3.2/3.1
    * Tools -> Port -> your Teensy device
    * Tools -> USB Type -> Hotas X
* Compile and upload to your Teensy

## Configure

All configurable paramaters are `#define`d in the `teensy-hotas.ino`.

## Help

* On Windows the location for the teensy `board.txt` and `cores` files you'll need to overwrite;
    * `C:\Users\{USERNAME}\AppData\Local\Arduino15\packages\teensy\hardware\avr\1.58.0`
* The Arduino IDE doesn't pick up updates to board.txt file after you edit it
    * Clear cache by deleting this dir...
    * `C:\Users\{USERNAME}\AppData\Roaming\arduino-ide`
* The default windows controllers control panel can be a little buggy I think. 
    * Sometimes it will show Hotas X, but clicking properties doesn't show the axis + inputs.
    * Ignore that shit. Use a raw joystick tester instead.

## Contributing

Pull requests are welcome.

### TODO's

* Finish centre-zero throttle mode and deadzone handling
* Add limit deadzones for the rest of the axes
* Calibration mode that allows measurement of min-max ranges for each axis
    * button or button combination puts the stick into/out of calibration mode
    * user moves the hardware to range limits and the software sets scaling factor for each axis
    * results saved to EEPROM
    * scaling factor used to scale the send value from MIN_AVAL to MAX_AVAL, as expected by the controller class

## References + Attribution

* Based on an original mod by Pat Daderko (DogP), 2016
* Using Paul Stoffregen's Extreme Joystick Test
* https://www.reddit.com/r/hotas/comments/49hlg8/tflight_hotas_x_mcu_replacement_teensy_mod/
* https://forum.pjrc.com/threads/33371-Thrustmaster-T-Flight-Hotas-X-Teensy-MCU-mod-%28USB-Flight-Joystick-Throttle%29
