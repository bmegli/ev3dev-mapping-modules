#!/usr/bin/env bash

# Change input 1 mode to other-uart, for XV11-LIDAR uart communication
echo other-uart > /sys/class/lego-port/port0/mode

# Change output port B mode to dc-motor, this is for XV11-LIDAR motor control
echo dc-motor > /sys/class/lego-port/port5/mode
