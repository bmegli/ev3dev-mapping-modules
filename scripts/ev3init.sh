#!/usr/bin/env bash

# Load the MicroInfnity CruizCore XG1300L gyroscope I2C driver manually, it can't be autodetected
echo mi-xg1300l 0x01 > /sys/bus/i2c/devices/i2c-5/new_device

# Change input 1 mode to other-uart, for XV11-LIDAR uart communication
echo other-uart > /sys/class/lego-port/port0/mode

# Same for input 2 (second lidar)
echo other-uart > /sys/class/lego-port/port1/mode

# Change output port B mode to dc-motor, this is for XV11-LIDAR motor control
echo dc-motor > /sys/class/lego-port/port5/mode

# Same for output port C (second lidar)cd 
echo dc-motor > /sys/class/lego-port/port6/mode
