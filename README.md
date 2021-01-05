# Magnetic Loop Antenna Tuner

The current repository contains the Arduino firmware for the 
Magnetic Loop Antenna tuner control box that is being described on the follwoing page: http://www.microfarad.de/magnetic-loop.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.

## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Circuit Diagram

The circuit diagram for this device can be found under the */doc* folder or can be downloaded using the follwoing link:
https://github.com/microfarad-de/antenna-tuner/raw/master/doc/antenna-tuner-schematic.png
